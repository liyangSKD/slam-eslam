#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include "PoseEstimator.hpp"
#include "Configuration.hpp"

#include <odometry/ContactOdometry.hpp>
#include <odometry/Configuration.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>
#include <envire/operators/DistanceGridToPointcloud.hpp>

#include <base/samples/LaserScan.hpp>

#include <base/samples/DistanceImage.hpp>

#include "SurfaceHash.hpp"

namespace eslam 
{

class EmbodiedSlamFilter
{
    eslam::Configuration eslamConfig;
    odometry::Configuration odometryConfig;

    odometry::FootContact odometry;
    eslam::PoseEstimator filter;

    /** pose of last update an mapping step */
    base::Affine3d udPose, mapPose, stereoPose;

    envire::MLSMap* sharedMap;
    SurfaceHash hash;

    // store pointers to processing pipeline
    envire::FrameNode *scanFrame;
    envire::FrameNode *scannerFrame;
    envire::LaserScan *scanNode;
    envire::TriMesh *pcNode;
    envire::ScanMeshing *smOp;
    envire::MLSProjection *mlsOp;
    envire::MultiLevelSurfaceGrid* scanMap;

    envire::FrameNode *distFrame;
    envire::DistanceGrid *distGrid;
    envire::ImageRGB24 *textureGrid;
    envire::DistanceGridToPointcloud *distOp;
    envire::TriMesh *distPc;
    envire::MLSProjection *distMlsOp;

public:
    EmbodiedSlamFilter(
	const odometry::Configuration& odometryConfig, 
	const eslam::Configuration& eslamConfig );

    envire::MLSMap* createMapTemplate( envire::Environment* env, const base::Pose& origin = base::Pose() );
    envire::MultiLevelSurfaceGrid* createGridTemplate( envire::Environment* env );
    void init( envire::Environment* env, const base::Pose& pose, bool useSharedMap = true, const SurfaceHashConfig& hashConfig = SurfaceHashConfig() );

    void processMap( envire::MLSGrid* scanMap, bool match, bool update );
    bool update( const Eigen::Affine3d& body2odometry, const base::samples::LaserScan& scan, const Eigen::Affine3d& laser2body );
    bool update( const Eigen::Affine3d& body2odometry, const base::samples::DistanceImage& dimage, const Eigen::Affine3d& camera2body, const base::samples::frame::Frame* timage = NULL );
    bool update( const Eigen::Affine3d& body2odometry, const odometry::BodyContactState& bs, const std::vector<terrain_estimator::TerrainClassification>& ltc );
    bool update( envire::Featurecloud *stereo_features );

    std::vector<eslam::PoseEstimator::Particle>& getParticles();
    size_t getBestParticleIndex() const;
    base::Affine3d getCentroid();
};

}

#endif
