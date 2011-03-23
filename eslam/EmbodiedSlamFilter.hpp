#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include "PoseEstimator.hpp"
#include "Configuration.hpp"

#include <asguard/Odometry.hpp>
#include <asguard/Transformation.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

#include <base/samples/laser_scan.h>

namespace eslam 
{

class EmbodiedSlamFilter
{
    eslam::Configuration eslamConfig;
    asguard::Configuration asguardConfig;
    // TODO: replace this with the proper frame stack
    asguard::Transformation trans;
    asguard::odometry::Configuration odometryConfig;

    asguard::odometry::Wheel odometry;
    eslam::PoseEstimator filter;

    /** current odometry pose */
    base::Pose odPose;

    /** pose of last update step */
    base::Pose udPose;

    /** pose of last mapping step */
    base::Pose mapPose;

    envire::MLSMap* sharedMap;

    // store pointers to processing pipeline
    envire::FrameNode *scanFrame;
    envire::FrameNode *scannerFrame;
    envire::LaserScan *scanNode;
    envire::TriMesh *pcNode;
    envire::ScanMeshing *smOp;
    envire::MLSProjection *mlsOp;
    envire::MultiLevelSurfaceGrid* scanMap;

public:
    EmbodiedSlamFilter(
	const asguard::Configuration& asguardConfig,
	const asguard::odometry::Configuration& odometryConfig, 
	const eslam::Configuration& eslamConfig );

    envire::MLSMap* createMapTemplate( envire::Environment* env, const base::Pose& origin = base::Pose() );
    envire::MultiLevelSurfaceGrid* createGridTemplate( envire::Environment* env );
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
