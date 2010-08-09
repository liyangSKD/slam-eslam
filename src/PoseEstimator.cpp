#include "PoseEstimator.hpp"
#include <algorithm>
#include <envire/GridAccess.hpp>

#include <stdexcept>

#include <omp.h>

using namespace eslam;

PoseEstimator::PoseEstimator(base::odometry::Sampling2D& odometry)
    : odometry(odometry), env(NULL), ga(NULL)
{
}

PoseEstimator::~PoseEstimator()
{
    if( ga )
	delete ga;
}

void PoseEstimator::setEnvironment(envire::Environment *env)
{
    this->env = env;

    if( ga )
	delete ga;

    // get a gridaccess object for direct access to the DEMs
    ga = new envire::PointcloudAccess(env);
}

void PoseEstimator::init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma) 
{
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_x(rand_gen, boost::normal_distribution<>(mu.position.x(),sigma.position.x()) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_y(rand_gen, boost::normal_distribution<>(mu.position.y(),sigma.position.y()) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_theta(rand_gen, boost::normal_distribution<>(mu.orientation,sigma.orientation) );

    for(int i=0;i<numParticles;i++)
    {
	xi_k.push_back( Particle( PoseParticle( Eigen::Vector2d(rand_x(), rand_y()), rand_theta()), 0 ));
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
	    Eigen::Translation3d( Eigen::Vector3d(pose.position.x(), pose.position.y(), 0) ) 
	    * Eigen::AngleAxisd( pose.orientation, Eigen::Vector3d::UnitZ() );

	pose.cpoints.clear();
	size_t found_points = 0;

	double sum_xsq = 0, sum_x = 0, sum_z = 0;
	for(int wi=0;wi<4;wi++)
	{
	    // find the contact points with the lowest zdiff per wheel
	    ContactPoint p;
	    bool contact = true;
	    for(std::vector<Eigen::Vector3d>::iterator it=cpoints[wi].begin();it!=cpoints[wi].end();it++)
	    {
		Eigen::Vector3d gp = t*(*it);
		double zdiff = gp.z();
		if( !ga->getElevation( gp ) )
		{
		    contact = false;
		    break;
		}
		    
		zdiff -= gp.z();

		if( zdiff < p.zdiff )
		   p = ContactPoint( gp, zdiff );
	    }

	    if( contact )
	    {
		found_points++;
		pose.cpoints.push_back( p );

		sum_z += p.point.z();
		sum_x += p.zdiff;
		sum_xsq += p.zdiff*p.zdiff;
	    }
	}

	int n = found_points;
	if( n > 1 ) // need to have at least two for the weighting to make sense
	{
	    pose.mean = sum_x/n;
	    pose.zPos = sum_z/n;
	    double var = sqrt(sum_xsq/n - (sum_x/n)*(sum_x/n));

	    // use some measurement of the variance as the weight 
	    xi_k[i].w *= weightingFunction( var );
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
	if( !particle.x.floating )
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
