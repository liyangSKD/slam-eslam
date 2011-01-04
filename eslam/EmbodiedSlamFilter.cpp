#include "EmbodiedSlamFilter.hpp"

#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

using namespace eslam;

EmbodiedSlamFilter::EmbodiedSlamFilter(asguard::Configuration &config)
    : config(config), odometry(config), filter(odometry, config), sharedMap(NULL) {};

envire::MultiLevelSurfaceGrid* EmbodiedSlamFilter::getMapTemplate( envire::Environment* env )
{
    const double size = 20;
    const double resolution = 0.05;
    envire::MultiLevelSurfaceGrid* gridTemplate = 
	    new envire::MultiLevelSurfaceGrid( size/resolution, size/resolution, resolution, resolution );
    envire::FrameNode *gridNode = new envire::FrameNode( Eigen::Transform3d( Eigen::Translation3d( -size/2.0, -size/2.0, 0 ) ) ); 
    env->addChild( env->getRootNode(), gridNode );
    env->setFrameNode( gridTemplate, gridNode );

    envire::MultiLevelSurfaceGrid::SurfacePatch p( 0, 1.0, 0, true );
    for( int x=-20; x<20; x++ )
    {
	for( int y=-20; y<20; y++ )
	{
	    gridTemplate->insertTail( size/resolution/2.0 + x, size/resolution/2.0 + y, p );
	}
    }

    return gridTemplate;
}

void EmbodiedSlamFilter::init( envire::Environment* env, const base::Pose& pose, bool useSharedMap )
{
    const double angle = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    filter.init(
	    config.filter.particleCount, 
	    base::Pose2D(Eigen::Vector2d(pose.position.x(),pose.position.y()),angle), 
	    //base::Pose2D(Eigen::Vector2d(config.filter.initialError,config.filter.initialError),config.filter.initialError),
	    base::Pose2D(Eigen::Vector2d(1e-3,1e-3),1e-3),
	    pose.position.z(),
	    //1.0 // sigma_z
	    1e-3
	    );

    odPose = pose;
    udPose = mapPose = base::Pose( Eigen::Vector3d(1000,0,0), Eigen::Quaterniond::Identity() ); 

    if( useSharedMap )
    {
	// see if there is a MLSGrid in the environment and use that as a sharedmap
	// otherwise create a new map
	std::vector<envire::MultiLevelSurfaceGrid*> grids = env->getItems<envire::MultiLevelSurfaceGrid>();
	if( !grids.empty() )
	{
	    // for now use the first grid found...
	    sharedMap = grids.front();
	}
	else
	    sharedMap = getMapTemplate( env );
    }

    // either use the shared map to init, or create a grid template for the per particle maps
    if( sharedMap )
	filter.setEnvironment( env, sharedMap, useSharedMap );
    else
	filter.setEnvironment( env, getMapTemplate( env ), useSharedMap );

    // setup environment for converting scans
    scanFrame = new envire::FrameNode();
    env->addChild( env->getRootNode(), scanFrame );

    scanNode = new envire::LaserScan();
    env->setFrameNode( scanNode, scanFrame );
    pcNode = new envire::TriMesh();
    env->setFrameNode( pcNode, scanFrame );

    smOp = new envire::ScanMeshing();
    env->attachItem( smOp );
    smOp->addInput( scanNode );
    smOp->addOutput( pcNode );

    mlsOp = new envire::MLSProjection();
    mlsOp->setHorizontalPatchThickness( 0.50 );
    env->attachItem( mlsOp );
    mlsOp->addInput( pcNode );
}

bool EmbodiedSlamFilter::update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation, const base::samples::LaserScan& scan )
{
    bool result = update( bs, orientation );

    Eigen::Transform3d pdelta( mapPose.toTransform().inverse() * odPose.toTransform() );
    const double max_angle = config.filter.updateThreshAngle * .1;
    const double max_dist = config.filter.updateThreshDistance * .1;
    if( Eigen::AngleAxisd( pdelta.rotation() ).angle() > max_angle || pdelta.translation().norm() > max_dist )
    {
	// convert scan object to pointcloud
	scanNode->lines.clear();
	scanNode->addScanLine( 0, scan );
	smOp->updateAll();

	if( sharedMap )
	{
	    scanFrame->setTransform( getCentroid().toTransform() * config.laser2Body );
	    mlsOp->removeOutputs();
	    mlsOp->addOutput( sharedMap );
	    mlsOp->updateAll();
	}
	else
	{
	    std::vector<eslam::PoseEstimator::Particle> &particles( getParticles() );
	    for( std::vector<eslam::PoseEstimator::Particle>::iterator it = particles.begin(); it != particles.end(); it++ )
	    {
		eslam::PoseEstimator::Particle &p( *it );

		scanFrame->setTransform( p.getPose( orientation ) * config.laser2Body );
		mlsOp->removeOutputs();
		mlsOp->addOutput( p.grid.get() );
		mlsOp->updateAll();
	    }
	}

	mapPose = odPose;
    }
    return result;
}

bool EmbodiedSlamFilter::update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation )
{
    odPose = base::Pose( odPose.toTransform() * odometry.getPoseDelta().toTransform() );

    odometry.update( bs, orientation );
    filter.project( bs, orientation );

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

std::vector<eslam::PoseEstimator::Particle>& EmbodiedSlamFilter::getParticles()
{
    return filter.getParticles();
}

base::Pose EmbodiedSlamFilter::getCentroid()
{
    return filter.getCentroid();
}

base::Pose EmbodiedSlamFilter::getOdometryPose()
{
    return odPose;
}

