#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include <asguard/Odometry.hpp>

#include <particle_filter/PoseEstimator.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

#include <base/samples/laser_scan.h>

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

    /** pose of last mapping step */
    base::Pose mapPose;

    boost::shared_ptr<envire::MultiLevelSurfaceGrid> sharedMap;

public:
    EmbodiedSlamFilter(asguard::Configuration &config);

    boost::shared_ptr<envire::MultiLevelSurfaceGrid> getMapTemplate( envire::Environment* env );
    void init( envire::Environment* env, const base::Pose& pose, bool useSharedMap = true );
    void updateMap( const Eigen::Transform3d& pose, const base::samples::LaserScan& scan, envire::MultiLevelSurfaceGrid* mlsGrid );
    bool update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation, const base::samples::LaserScan& scan );
    bool update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation );

    std::vector<eslam::PoseEstimator::Particle>& getParticles();
    base::Pose getCentroid();
    base::Pose getOdometryPose();
};

}

#endif
