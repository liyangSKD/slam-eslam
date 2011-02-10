#include "PoseEstimator.hpp"
#include <algorithm>
#include <envire/tools/GridAccess.hpp>

#include <stdexcept>
#include <set>

#include <omp.h>

using namespace eslam;

PoseEstimator::PoseEstimator(asguard::odometry::Wheel& odometry, const eslam::Configuration &config, const asguard::Configuration& asguardConfig )
    : ParticleFilter<Particle>(config.seed), config(config), asguardConfig(asguardConfig),  odometry(odometry), env(NULL)
{
}

PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::cloneMaps()
{
    // this function will make sure that no two particles will point to the same map
    // this works by cloning maps if they are referenced more than once
    std::set<envire::EnvironmentItem*> used;

    for( std::vector<Particle>::iterator it = xi_k.begin(); it != xi_k.end(); it++ )
    {
	envire::EnvironmentItem* grid = it->grid.getMap();
	if( !used.insert( grid ).second )
	{
	    it->grid.copy( it->grid ); 	
	}
    }
}

void PoseEstimator::setEnvironment(envire::Environment *env, envire::MLSMap::Ptr map, bool useShared )
{
    assert(env);
    assert(map);

    this->env = env;
    this->useShared = useShared;

    for( std::vector<Particle>::iterator it = xi_k.begin(); it != xi_k.end(); it++ )
	it->grid.setMap( map );

    if( !useShared )
	cloneMaps();
}

void PoseEstimator::init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos, double zsigma) 
{
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_x(rand_gen, boost::normal_distribution<>(mu.position.x(),sigma.position.x()) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_y(rand_gen, boost::normal_distribution<>(mu.position.y(),sigma.position.y()) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_theta(rand_gen, boost::normal_distribution<>(mu.orientation,sigma.orientation) );

    for(int i=0;i<numParticles;i++)
    {
	xi_k.push_back( 
		Particle( 
		    Eigen::Vector2d(rand_x(), rand_y()), 
		    rand_theta(),
		    zpos,
		    zsigma
		    ));
    }
}

void PoseEstimator::project(const asguard::BodyState& state, const Eigen::Quaterniond& orientation)
{
    Eigen::Transform3d dtrans = orientation * odometry.getPoseDelta().toTransform();
    const double z_delta = dtrans.translation().z();
    //const double z_var = 1e-3;
    const double z_var = odometry.getPositionError()(2,2) * 2.0;

    for(size_t i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample2D();

	Particle &p( xi_k[i] );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;

	p.zPos += z_delta;
	p.zSigma = sqrt( p.zSigma*p.zSigma + z_var );
    }
}

double PoseEstimator::weightingFunction( double stdev )
{
    double x = stdev;
    const double alpha = config.weightingFactor, beta = 1.0, gamma = 0.05;
    if( x < alpha )
	return 1.0;
    if( x < beta )
    {
	double a = (1.0-gamma)/(alpha-beta);
	double b = 1.0-alpha*a;
	return a*x + b; 
    }
    if( x >= beta )
	return gamma;

    return 0.0;
}

void PoseEstimator::update(const asguard::BodyState& state, const Eigen::Quaterniond& orientation)
{
    updateWeights(state, orientation);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
	if( !useShared )
	    cloneMaps();
    }
}

template <class T, int N> 
    bool compareElement(const T& a, const T& b)
{
    return a[N] < b[N];
}

void PoseEstimator::updateWeights(const asguard::BodyState& state, const Eigen::Quaterniond& orientation)
{
    if( !env )
	throw std::runtime_error("No environment attached.");

    
    // calculate foot positions and rotate them using pitch/roll
    typedef std::vector<Eigen::Vector3d> vec3array;
    std::vector<vec3array> cpoints(4);

    // get the orientation first and remove any rotation around the z axis
    Eigen::Vector3d projy = orientation * Eigen::Vector3d::UnitY(); 
    Eigen::Quaterniond ocomp = Eigen::AngleAxisd( -atan2( -projy.x(), projy.y() ), Eigen::Vector3d::UnitZ()) * orientation;

    // store ocomp for calculation of median
    zCompensatedOrientation = ocomp;

    for(int i=0;i<4;i++)
    {
	for(int j=0;j<5;j++) 
	{
	    Eigen::Vector3d f = asguardConfig.getFootPosition( state, static_cast<asguard::wheelIdx>(i), j );
	    cpoints[i].push_back( ocomp * f );	
	}
	// remove the two wheels with the highest z value
	std::sort( cpoints[i].begin(), cpoints[i].end(), compareElement<Eigen::Vector3d,2> );
	cpoints[i].resize( 3 );
    }

    size_t total_points = 0;
    size_t data_particles = 0;
    double sum_data_weights = 0.0;

    // now update the weights of the particles by calculating the variance of the contact points 
#ifdef USE_OPENMP
#warning "using OpenMP"
#pragma omp parallel for
#endif
    for(size_t i=0;i<xi_k.size();i++)
    {
	Particle &pose(xi_k[i]);
	Eigen::Vector3d pos( pose.position.x(), pose.position.y(), pose.zPos );
	Eigen::Transform3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( pose.orientation, Eigen::Vector3d::UnitZ() );

	pose.cpoints.clear();
	// store some debug information in the particle
	pose.meas_pos = pos; 
	pose.meas_theta = pose.orientation;
	size_t found_points = 0;

	for(int wi=0;wi<4;wi++)
	{
	    // find the contact points with the lowest zdiff per wheel
	    ContactPoint p;
	    bool contact = true;
	    for(std::vector<Eigen::Vector3d>::iterator it=cpoints[wi].begin();it!=cpoints[wi].end();it++)
	    {
		const Eigen::Vector3d &cpoint(*it);

		Eigen::Vector3d gp = t*cpoint;
		const double cp_stdev = config.measurementError;
		double zpos, zstdev = sqrt(pose.zSigma*pose.zSigma + cp_stdev*cp_stdev);
		if( !pose.grid.get( gp, zpos, zstdev ) )
		{
		    contact = false;
		    break;
		}

		const double zdiff = gp.z() - zpos;

		if( zdiff < p.zdiff )
		{
		    const double zvar = pose.zSigma * pose.zSigma + zstdev * zstdev + cp_stdev * cp_stdev;
		    p = ContactPoint( gp-Eigen::Vector3d::UnitZ()*zdiff, zdiff, zvar );
		}
	    }

	    if( contact )
	    {
		found_points++;
		pose.cpoints.push_back( p );
	    }
	}

	if( found_points > 0 ) 
	{
	    // calculate the z-delta with the highest combined probability
	    // of the individual contact points
	    double d1=0, d2=0; 
	    for(std::vector<ContactPoint>::iterator it=pose.cpoints.begin();it!=pose.cpoints.end();it++)
	    {
		ContactPoint &p(*it);
		d1 += p.zdiff/p.zvar;
		d2 += 1.0/p.zvar;
		//std::cout << "p.zdiff: " << p.zdiff << " p.zvar: " << p.zvar << " stdev: " << sqrt(p.zvar) << std::endl;
	    }
	    const double delta = d1 / d2;

	    // calculate the joint probability of the individual foot contact points using the
	    // most likely z-height from the previous calculation of delta
	    double pz = 1.0;
	    for(std::vector<ContactPoint>::iterator it=pose.cpoints.begin();it!=pose.cpoints.end();it++)
	    {
		ContactPoint &p(*it);
		const double odiff = (p.zdiff - delta)/sqrt(p.zvar);

		const double zk = exp(-(odiff*odiff)/(2.0));
		pz *= zk;
	    }
	    const double cp_stdev = config.measurementError;
	    const double zd = delta/sqrt(pose.zSigma*pose.zSigma + cp_stdev*cp_stdev);
	    pz *= exp( -(zd*zd)/2.0 );
	    //pz = 1.0;

	    if(false)
	    {
		std::cout 
		    << "points: " << found_points
		    << "\tzPos:" << pose.zPos
		    << "\tzSigma:" << pose.zSigma
		    << "\tdelta:" << delta 
		    << "\td1: " << d1
		    << "\td2: " << d2
		    << "\tpz: " << pz 
		    << std::endl;
	    }	
	    pose.zPos += -delta;
	    pose.zSigma = 1.0/sqrt(d2);

	    // use some measurement of the variance as the weight 
	    xi_k[i].weight *= pz;
	    xi_k[i].mprob = pz;
	    xi_k[i].floating = false;
	    
	    data_particles ++;
	    sum_data_weights += pow(pz,1/found_points);
	}
	else
	{
	    // slowly reduce likelyhood of particles with no measurements
	    // and mark them as floating
	    xi_k[i].floating = true;
	    xi_k[i].mprob = 1.0;
	    //xi_k[i].w *= 0.99;
	}
	total_points += found_points;
    }

    const double floating_weight = data_particles>0 ? sum_data_weights/data_particles : 1.0;
    //std::cout << "fw: " << floating_weight << " sum_data_weights: " << sum_data_weights << " data_particles: " << data_particles << std::endl;

    for(std::vector<Particle>::iterator it=xi_k.begin();it!=xi_k.end();it++)
    {
	//if((*it).x.cpoints.size() < 4)
	//{
	double factor = (*it).mprob * pow(config.discountFactor*floating_weight, 4-(*it).cpoints.size());
	//if( (*it).x.floating )
	    //factor *= 0.8;
	
	(*it).weight *= factor;
	//}
    }

    static int iter = 0;
    std::cerr << "iteration: " << iter++ << "\tfound: " << total_points << "\tmax: " << xi_k.size() << "       \r";
}

base::Pose PoseEstimator::getCentroid()
{
    // calculate the weighted mean for now
    base::Pose2D mean;
    double zMean = 0;
    for(size_t i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	//if( !particle.x.floating )
	{
	    mean.position += particle.position * particle.weight;
	    mean.orientation += particle.orientation * particle.weight;
	    zMean += particle.zPos * particle.weight;
	}
    }

    // and convert into a 3d position
    base::Pose result( 
	    Eigen::Vector3d( mean.position.x(), mean.position.y(), zMean ), 
	    Eigen::AngleAxisd( mean.orientation, Eigen::Vector3d::UnitZ() ) * zCompensatedOrientation );

    return result;
}
