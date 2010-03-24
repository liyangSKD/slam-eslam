#include "PoseEstimator.hpp"

using namespace eslam;

void PoseEstimator::init(int numParticles, const Pose2D& mu, const Pose2D& sigma) 
{
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_x(rand_gen, boost::normal_distribution<>(mu.x,sigma.x) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_y(rand_gen, boost::normal_distribution<>(mu.y,sigma.y) );
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand_theta(rand_gen, boost::normal_distribution<>(mu.theta,sigma.theta) );

    for(int i=0;i<numParticles;i++)
    {
	xi_k.push_back( Particle( Pose2D(rand_x(), rand_y(), rand_theta()), 0 ));
    }
}


void PoseEstimator::sampleState()
{
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	rand(rand_gen, boost::normal_distribution<>(0,1) );

    // Very simple model for now
    double w = 0.35; // distance between left and right wheel
    double wheel_radius = 0.200; 

    double d1 = (u_k.delta[0] + u_k.delta[1])/2.0*wheel_radius; // averaged left side distance 
    double d2 = (u_k.delta[2] + u_k.delta[3])/2.0*wheel_radius;  // same for right side

    for(int i=0;i<xi_k.size();i++)
    {
	// apply noise, assuming the wheel never goes more than 
	// what the odometry says, but might do less
	double d1n = d1 - (fabs(rand())/d1);
	double d2n = d2 - (fabs(rand())/d2);

	double r = (d1n+d2n)/(d2n-d1n)*w/2.0; // radius of circle travelling on
	double dtheta = d1n/(r-w/2.0); // change in angle

	double dx = r*(1-cos(dtheta)); // displacement in x coord
	double dy = r*sin(dtheta); // displacement in y coord

	Pose2D &xt(xi_k[i].x);
	Eigen::Vector2d dpos = Eigen::Rotation2D<double>(xt.theta)*Eigen::Vector2d(dx, dy);

	xt.x += dpos.x();
	xt.y += dpos.y();
	xt.theta += dtheta;
    }
}


void PoseEstimator::updateWeights()
{
    for(int i=0;i<xi_k.size();i++)
    {
	// TODO implement
    };
}

