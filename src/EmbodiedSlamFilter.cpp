#include "EmbodiedSlamFilter.hpp"

#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

using namespace eslam;

EmbodiedSlamFilter::EmbodiedSlamFilter(asguard::Configuration &config)
    : config(config), odometry(config), filter(odometry, config) {};

boost::shared_ptr<envire::MultiLevelSurfaceGrid> EmbodiedSlamFilter::getMapTemplate( envire::Environment* env )
{
    const double size = 20;
    const double resolution = 0.05;
    boost::shared_ptr<envire::MultiLevelSurfaceGrid> gridTemplate(
	    new envire::MultiLevelSurfaceGrid( size/resolution, size/resolution, resolution, resolution ) );
    envire::FrameNode *gridNode = new envire::FrameNode( Eigen::Transform3d( Eigen::Translation3d( -size/2.0, -size/2.0, 0 ) ) ); 
    env->addChild( env->getRootNode(), gridNode );
    env->setFrameNode( gridTemplate.get(), gridNode );

    return gridTemplate;
}

void EmbodiedSlamFilter::init( envire::Environment* env, const base::Pose& pose, bool useSharedMap )
{
    const double angle = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    filter.init(
	    config.filter.particleCount, 
	    base::Pose2D(Eigen::Vector2d(pose.position.x(),pose.position.y()),angle), 
	    base::Pose2D(Eigen::Vector2d(config.filter.initialError,config.filter.initialError),config.filter.initialError),
	    pose.position.z(),
	    1.0 // sigma_z
	    );

    mapPose = odPose = pose;


    if( useSharedMap )
    {
	// see if there is a MLSGrid in the environment and use that as a sharedmap
	// otherwise create a new map
	std::vector<envire::MultiLevelSurfaceGrid*> grids = env->getItems<envire::MultiLevelSurfaceGrid>();
	if( !grids.empty() )
	{
	    // for now use the first grid found...
	    sharedMap = boost::shared_ptr<envire::MultiLevelSurfaceGrid>(grids.front());
	}
	else
	    sharedMap = getMapTemplate( env );
    }

    // either use the shared map to init, or create a grid template for the per particle maps
    if( sharedMap )
	filter.setEnvironment( env, sharedMap, useSharedMap );
    else
	filter.setEnvironment( env, getMapTemplate( env ), useSharedMap );
}

void EmbodiedSlamFilter::updateMap( const Eigen::Transform3d& pose, const base::samples::LaserScan& scan, envire::MultiLevelSurfaceGrid* mlsGrid )
{
    envire::Environment* env = mlsGrid->getEnvironment();

    envire::FrameNode *scanFrame = new envire::FrameNode( pose * config.laser2Body );
    env->addChild( env->getRootNode(), scanFrame );
    envire::LaserScan *scanNode = new envire::LaserScan();
    scanNode->addScanLine( 0, scan );
    env->setFrameNode( scanNode, scanFrame );

    envire::TriMesh *pcNode = new envire::TriMesh();
    env->setFrameNode( pcNode, scanFrame );

    envire::ScanMeshing *smOp = new envire::ScanMeshing();
    env->attachItem( smOp );
    smOp->addInput( scanNode );
    smOp->addOutput( pcNode );

    smOp->updateAll();

    envire::MLSProjection *mlsOp = new envire::MLSProjection();
    mlsOp->setHorizontalPatchThickness( 0.20 );
    env->attachItem( mlsOp );
    mlsOp->addInput( pcNode );
    mlsOp->addOutput( mlsGrid );

    mlsOp->updateAll();

    // we can remove the scanmeshing operator and the laserscan now
    env->detachItem( smOp );
    delete smOp;
    env->detachItem( scanNode );
    delete scanNode;
    env->detachItem( mlsOp );
    delete mlsOp;
    env->detachItem( pcNode );
    delete pcNode;
    env->detachItem( scanFrame );
    delete scanFrame;
}


bool EmbodiedSlamFilter::update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation, const base::samples::LaserScan& scan )
{
    bool result = update( bs, orientation );

    Eigen::Transform3d pdelta( mapPose.toTransform().inverse() * odPose.toTransform() );
    const double max_angle = config.filter.updateThreshAngle * .1;
    const double max_dist = config.filter.updateThreshDistance * .1;
    if( Eigen::AngleAxisd( pdelta.rotation() ).angle() > max_angle || pdelta.translation().norm() > max_dist )
    {
	if( sharedMap )
	{
	    updateMap( getCentroid().toTransform(), scan, sharedMap.get() );
	}
	else
	{
	    std::vector<eslam::PoseEstimator::Particle> &particles( getParticles() );
	    for( std::vector<eslam::PoseEstimator::Particle>::iterator it = particles.begin(); it != particles.end(); it++ )
	    {
		eslam::PoseParticle &p( it->x );
		updateMap( p.getPose( orientation ), scan, p.grid.get() );
	    }
	}

	std::cout << "map update" << std::endl;
	mapPose = odPose;
    }
    return result;
}

bool EmbodiedSlamFilter::update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation )
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

	std::cout << "measurement update" << std::endl;
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

