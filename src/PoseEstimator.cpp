#include "PoseEstimator.hpp"
#include <algorithm>

using namespace eslam;

PoseEstimator::PoseEstimator()
    : odometry(config)
{
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
	xi_k.push_back( Particle( base::Pose2D( Eigen::Vector2d(rand_x(), rand_y()), rand_theta()), 0 ));
    }
}

void PoseEstimator::project(const asguard::BodyState& state)
{
    odometry.updateBodyState( state );
    if( !odometry.isValid() )
	return;

    for(int i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample();
	base::Pose2D &p( xi_k[i].x );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;
    }
}

/*
void PoseEstimator::updateWeights()
{
    for(int i=0;i<xi_k.size();i++)
    {
	// TODO implement
    };
}
*/
