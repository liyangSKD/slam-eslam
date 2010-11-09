#ifndef __ESLAM_EMBODIEDSLAMFILTER__
#define __ESLAM_EMBODIEDSLAMFILTER__

#include <asguard/Odometry.hpp>

#include <particle_filter/PoseEstimator.hpp>

#include <envire/Core.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

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

    envire::MultiLevelSurfaceGrid* sharedMap;

public:
    EmbodiedSlamFilter(asguard::Configuration &config)
	: config(config), odometry(config), filter(odometry, config), sharedMap(NULL) {};

    void init( envire::Environment* env, const base::Pose& pose, bool useSharedMap = true )
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
	    {
		const double size = 20;
		const double resolution = 0.05;
		sharedMap = new envire::MultiLevelSurfaceGrid( size/resolution, size/resolution, resolution, resolution );
		envire::FrameNode *gridNode = new envire::FrameNode( Eigen::Transform3d( Eigen::Translation3d( -size/2.0, -size/2.0, 0 ) ) ); 
		env->addChild( env->getRootNode(), gridNode );
		env->setFrameNode( sharedMap, gridNode );
	    }
	}
    }

    void updateMap( const Eigen::Transform3d& pose, const base::samples::LaserScan& scan, envire::MultiLevelSurfaceGrid* mlsGrid )
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
	// we can remove the scanmeshing operator and the laserscan now
	env->detachItem( smOp ); 
	delete smOp;
	env->detachItem( scanNode ); 
	delete scanNode;
	
	envire::MLSProjection *mlsOp = new envire::MLSProjection();
	env->attachItem( mlsOp );
	mlsOp->addInput( pcNode );
	mlsOp->addOutput( mlsGrid );

	mlsOp->updateAll();
    }


    bool update( const asguard::BodyState& bs, const Eigen::Quaterniond& orientation, const base::samples::LaserScan& scan )
    {
	update( bs, orientation );
	if( sharedMap )
	{
	    updateMap( getCentroid().toTransform(), scan, sharedMap );
	}
	else
	{
	    throw std::runtime_error("not yet implemented");
	}
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
