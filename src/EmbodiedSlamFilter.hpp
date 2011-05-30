#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include "PoseEstimator.hpp"
#include "Configuration.hpp"

#include <asguard/Odometry.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

#include <base/samples/laser_scan.h>

namespace eslam 
{

class EmbodiedSlamFilter
{
    eslam::Configuration eslamConfig;
    asguard::Configuration asguardConfig;
    asguard::odometry::Configuration odometryConfig;

    asguard::odometry::Wheel odometry;
    eslam::PoseEstimator filter;

    /** pose of last update an mapping step */
    base::Affine3d udPose, mapPose;

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
    void updateMap( const Eigen::Affine3d& pose, const base::samples::LaserScan& scan, envire::MultiLevelSurfaceGrid* mlsGrid );

    bool update( const Eigen::Affine3d& body2odometry, const base::samples::LaserScan& scan, const Eigen::Affine3d& laser2body );
    bool update( const Eigen::Affine3d& body2odometry, const asguard::BodyState& bs );

    std::vector<eslam::PoseEstimator::Particle>& getParticles();
    base::Affine3d getCentroid();
};

}

#endif
