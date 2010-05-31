#include "PoseEstimator.hpp"
#include <algorithm>
#include <envire/GridAccess.hpp>

#include <stdexcept>

using namespace eslam;

PoseEstimator::PoseEstimator()
    : odometry(config), env(NULL), ga(NULL)
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
    ga = new envire::GridAccess(env);
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
    odometry.state.update( state );
    if( !odometry.state.isValid() )
	return;

    for(int i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample();
	base::Pose2D &p( xi_k[i].x );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;
    }
}

void PoseEstimator::update(const asguard::BodyState& state, const Eigen::Quaterniond& orientation)
{
    if( !env )
	throw std::runtime_error("No environment attached.");

    
    // calculate foot positions and rotate them using pitch/roll
    std::vector<Eigen::Vector3d> cpoints;

    // get the orientation first and remove any rotation around the z axis
    Eigen::Vector3d projy = orientation * Eigen::Vector3d::UnitY(); 
    Eigen::Quaterniond ocomp = Eigen::AngleAxisd( -atan2( -projy.x(), projy.y() ), Eigen::Vector3d::UnitZ()) * orientation;

    for(int i=0;i<4;i++)
    {
	for(int j=0;j<5;j++) 
	{
	    if( state.getWheelContact( static_cast<asguard::wheelIdx>(i), j ).contact > 0.5 )
	    {
		// if there is a contact convert to local frame and store in vector
		Eigen::Vector3d f = config.getFootPosition( state, static_cast<asguard::wheelIdx>(i), j );
		cpoints.push_back( ocomp * f );	
	    }
	}
    }

    // now update the weights of the particles by calculating the variance of the contact points 
    for(int i=0;i<xi_k.size();i++)
    {
	PoseParticle &pose(xi_k[i].x);
	Eigen::Transform3d t = 
	    Eigen::Translation3d( Eigen::Vector3d(pose.position.x(), pose.position.y(), 0) ) 
	    * Eigen::AngleAxisd( pose.orientation, Eigen::Vector3d::UnitZ() );

	pose.cpoints.clear();
	double sum_xsq = 0, sum_x = 0;
	for(std::vector<Eigen::Vector3d>::iterator it=cpoints.begin();it!=cpoints.end();it++)
	{
	    Eigen::Vector3d gp = t*(*it);
	    double x = gp.z();
	    ga->getElevation( gp ); // this will set the z component of gp to the dem value
	    pose.cpoints.push_back( gp );
	    x -= gp.z();
	    sum_x += x;
	    sum_xsq += x*x;
	}

	int n = cpoints.size();
	if( n > 0 )
	{
	    double var = sum_xsq/n - (sum_x/n)*(sum_x/n);

	    // use some measurement of the variance as the weight 
	    if(std::isnan(var))
	    {
		xi_k[i].w = 0.0;
	    }
	    else
		xi_k[i].w = 1.0/(.1+var);
	}
    }
}

