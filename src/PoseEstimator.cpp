#include "PoseEstimator.hpp"
#include <algorithm>
#include <envire/GridAccess.hpp>

#include <stdexcept>

#include <omp.h>

using namespace eslam;

PoseEstimator::PoseEstimator(base::odometry::Sampling2D& odometry)
    : odometry(odometry), env(NULL)
{
}

PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::setEnvironment(envire::Environment *env)
{
    this->env = env;
    ga = std::auto_ptr<envire::MLSAccess>( new envire::MLSAccess(env));
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
		    PoseParticle( 
			Eigen::Vector2d(rand_x(), rand_y()), 
			rand_theta(),
			zpos,
			zsigma
		    ), 0 ));
    }
}

void PoseEstimator::project(const asguard::BodyState& state)
{
    for(int i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample2D();
	base::Pose2D &p( xi_k[i].x );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;
    }
}

double PoseEstimator::weightingFunction( double stdev )
{
    double x = stdev;
    const double alpha = config.filter.weightingFactor, beta = 1.0, gamma = 0.05;
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
    if( eff < config.filter.minEffective )
    {
	resample();
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
	    Eigen::Vector3d f = config.getFootPosition( state, static_cast<asguard::wheelIdx>(i), j );
	    cpoints[i].push_back( ocomp * f );	
	}
	// remove the two wheels with the highest z value
	std::sort( cpoints[i].begin(), cpoints[i].end(), compareElement<Eigen::Vector3d,2> );
	cpoints[i].resize( 3 );
    }

    int total_points = 0;

    // now update the weights of the particles by calculating the variance of the contact points 
#ifdef USE_OPENMP
#warning "using OpenMP"
#pragma omp parallel for
#endif
    for(int i=0;i<xi_k.size();i++)
    {
	PoseParticle &pose(xi_k[i].x);
	Eigen::Transform3d t = 
	    Eigen::Translation3d( Eigen::Vector3d(pose.position.x(), pose.position.y(), pose.zPos) ) 
	    * Eigen::AngleAxisd( pose.orientation, Eigen::Vector3d::UnitZ() );

	pose.cpoints.clear();
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
		double zpos, zstdev = pose.zSigma;
		if( !ga->getElevation( gp, zpos, zstdev ) )
		{
		    contact = false;
		    break;
		}

		const double zdiff = gp.z() - zpos;

		if( zdiff < p.zdiff )
		{
		    const double zvar = pose.zSigma * pose.zSigma + zstdev * zstdev;
		    p = ContactPoint( gp, zdiff, zvar );
		}
	    }

	    if( contact )
	    {
		found_points++;
		pose.cpoints.push_back( p );
	    }
	}

	if( found_points > 1 ) // need to have at least two for the weighting to make sense
	{
	    // calculate the z-delta with the highest combined probability
	    // of the individual contact points
	    double d1=0, d2=0; 
	    for(std::vector<ContactPoint>::iterator it=pose.cpoints.begin();it!=pose.cpoints.end();it++)
	    {
		ContactPoint &p(*it);
		d1 += p.zdiff/p.zvar;
		d2 += 1.0/p.zvar;
		std::cout << "p.zdiff: " << p.zdiff << " p.zvar: " << p.zvar << " stdev: " << sqrt(p.zvar) << std::endl;
	    }
	    const double delta = d1 / d2;

	    // calculate the joint probability of the individual foot contact points using the
	    // most likely z-height from the previous calculation of delta
	    double pz = 1.0;
	    for(std::vector<ContactPoint>::iterator it=pose.cpoints.begin();it!=pose.cpoints.end();it++)
	    {
		ContactPoint &p(*it);
		const double odiff = (p.zdiff - delta)/p.zvar;

		const double zk = exp(-(odiff*odiff)/(2.0));
		pz *= zk;
	    }
	    pz = 1.0;

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
	    xi_k[i].w *= pz;
	    xi_k[i].x.floating = false;
	}
	else
	{
	    // slowly reduce likelyhood of particles with no measurements
	    // and mark them as floating
	    xi_k[i].x.floating = true;
	    xi_k[i].w *= 0.99;
	}
	total_points += found_points;
    }

    static int iter = 0;
    std::cout << "iteration: " << iter++ << "\tfound: " << total_points << "\tmax: " << xi_k.size() << "       \r";
}

base::Pose PoseEstimator::getCentroid()
{
    // calculate the weighted mean for now
    base::Pose2D mean;
    double zMean = 0;
    for(int i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	//if( !particle.x.floating )
	{
	    mean.position += particle.x.position * particle.w;
	    mean.orientation += particle.x.orientation * particle.w;
	    zMean += particle.x.zPos * particle.w;
	}
    }

    // and convert into a 3d position
    base::Pose result( 
	    Eigen::Vector3d( mean.position.x(), mean.position.y(), zMean ), 
	    Eigen::AngleAxisd( mean.orientation, Eigen::Vector3d::UnitZ() ) * zCompensatedOrientation );

    return result;
}
