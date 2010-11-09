#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include <asguard/Odometry.hpp>

#include <particle_filter/PoseEstimator.hpp>
#include <envire/Core.hpp>

namespace eslam 
{

class EmbodiedSlamFilter
{
    asguard::Configuration& config;
    asguard::odometry::Wheel odometry;
    eslam::PoseEstimator filter;

    /** current odometry pose */
    base::Pose odPose;

    /** pose of last update step */
    base::Pose udPose;
public:
    EmbodiedSlamFilter(asguard::Configuration &config)
	: config(config), odometry(config), filter(odometry, config) {};

    void init( envire::Environment* env, const base::Pose& pose )
    {
	filter.setEnvironment( env );

	const double angle = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
	filter.init(
		config.filter.particleCount, 
		base::Pose2D(Eigen::Vector2d(pose.position.x(),pose.position.y()),angle), 
		base::Pose2D(Eigen::Vector2d(config.filter.initialError,config.filter.initialError),config.filter.initialError),
		pose.position.z(),
		1.0 // sigma_z
		);

	odPose = pose;
    }

    bool update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation )
    {
	odPose = base::Pose( odPose.toTransform() * odometry.getPoseDelta().toTransform() );

	odometry.update( bs, orientation );
	filter.project( bs );

	Eigen::Transform3d pdelta( udPose.toTransform().inverse() * odPose.toTransform() );
        const double max_angle = config.filter.updateThreshAngle;
        const double max_dist = config.filter.updateThreshDistance;
	if( Eigen::AngleAxisd( pdelta.rotation() ).angle() > max_angle || pdelta.translation().norm() > max_dist )
	{
	    filter.update( bs, orientation );
	    udPose = odPose;

	    return true;
	}
	else
	    return false;
    }

    std::vector<eslam::PoseEstimator::Particle>& getParticles()
    {
	return filter.getParticles();
    }

    base::Pose getCentroid()
    {
	return filter.getCentroid();
    }

    base::Pose getOdometryPose()
    {
	return odPose;
    }
};

}

#endif
