#include "EmbodiedSlamFilter.hpp"

#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/operators/ScanMeshing.hpp>

#include <envire/tools/Numeric.hpp>
#include <terrain_estimator/TerrainConfiguration.hpp>

using namespace eslam;
using namespace envire;

EmbodiedSlamFilter::EmbodiedSlamFilter(
	const odometry::Configuration& odometryConfig, 
	const eslam::Configuration& eslamConfig )
:   eslamConfig( eslamConfig ),
    odometryConfig( odometryConfig ),
    odometry( odometryConfig ), 
    filter( odometry, eslamConfig ), 
    sharedMap(NULL),
    distGrid(NULL),
    textureGrid(NULL)
{};

MLSGrid* EmbodiedSlamFilter::createGridTemplate( envire::Environment* env )
{
    const double size = eslamConfig.gridSize;
    const double resolution = eslamConfig.gridResolution;
    envire::MLSGrid* gridTemplate = 
	    new envire::MLSGrid( size/resolution, size/resolution, resolution, resolution, -size/2.0, -size/2.0  );
    envire::FrameNode *gridNode = new envire::FrameNode(); 
    env->addChild( env->getRootNode(), gridNode );
    env->setFrameNode( gridTemplate, gridNode );

    gridTemplate->setHorizontalPatchThickness( eslamConfig.gridPatchThickness );
    gridTemplate->setGapSize( eslamConfig.gridGapSize );

    return gridTemplate;
}

MLSMap* EmbodiedSlamFilter::createMapTemplate( envire::Environment* env, const base::Pose& origin )
{
    envire::MLSGrid* gridTemplate = 
	createGridTemplate( env );

    MLSMap* mapTemplate = new MLSMap();
    FrameNode *mapNode = new envire::FrameNode(); 
    env->addChild( env->getRootNode(), mapNode );
    env->addChild( mapNode, gridTemplate->getFrameNode() );
    env->setFrameNode( mapTemplate, mapNode );
    mapTemplate->addGrid( gridTemplate );

    {
	double angle = 0.0;
	const bool aligned = true;
	if( !aligned )
	    angle = origin.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
	Eigen::Vector3d pos = origin.position;
	pos.z() = 0;
	FrameNode *gridFrame = gridTemplate->getFrameNode();
	gridFrame->setTransform( Eigen::Translation3d( pos ) *
		Eigen::AngleAxisd( angle, Eigen::Vector3d::UnitZ() ) *
		gridFrame->getTransform() );
    }

    return mapTemplate;
}


void EmbodiedSlamFilter::init( envire::Environment* env, const base::Pose& pose, bool useSharedMap, const SurfaceHashConfig& hashConfig )
{
    bool useHash = hashConfig.useHash;
    if( useSharedMap )
    {
	// see if there is a MLSGrid in the environment and use that as a sharedmap
	// otherwise create a new map
	std::vector<envire::MLSGrid*> grids = env->getItems<envire::MLSGrid>();
	if( !grids.empty() )
	{
	    MLSGrid *gridTemplate = grids.front();

	    // for now use the first grid found...
	    MLSMap* mapTemplate = new MLSMap();
	    FrameNode *mapNode = new envire::FrameNode(); 
	    env->addChild( env->getRootNode(), mapNode );
	    if( gridTemplate->getFrameNode() == env->getRootNode() )
		env->setFrameNode( gridTemplate, mapNode );
	    else
		env->addChild( mapNode, gridTemplate->getFrameNode() );
	    env->setFrameNode( mapTemplate, mapNode );
	    mapTemplate->addGrid( gridTemplate );

	    if( useHash )
	    {
		// create a surfaceHash
		hash.setConfiguration( hashConfig );
		hash.create( gridTemplate );
	    }

	    sharedMap = mapTemplate;
	}
	else
	{
	    throw std::runtime_error("The provided environment does not contain an mls grid.");
	    //sharedMap = createMapTemplate( env );
	}
    }

    const double angle = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    if( useHash )
    {
	filter.init( 
		eslamConfig.particleCount, 
		&hash );
    }
    else
    {
	filter.init(
		eslamConfig.particleCount, 
		base::Pose2D(Eigen::Vector2d(pose.position.x(),pose.position.y()),angle), 
		base::Pose2D(Eigen::Vector2d(eslamConfig.initialError,eslamConfig.initialError),eslamConfig.initialError),
		//base::Pose2D(Eigen::Vector2d(1e-3,1e-3),1e-3),
		pose.position.z(),
		eslamConfig.initialError + 1e-3 // z-sigma
		);
    }

    udPose = mapPose = stereoPose = Eigen::Translation3d(1000,0,0) * Eigen::Affine3d::Identity();

    // either use the shared map to init, or create a grid template for the per particle maps
    if( sharedMap )
	filter.setEnvironment( env, sharedMap, useSharedMap );
    else
	filter.setEnvironment( env, createMapTemplate( env, pose ), useSharedMap );

    // setup environment for converting scans
    scanMap = createGridTemplate( env ); 
    scanFrame = new envire::FrameNode(); // yaw compensated body frame
    scannerFrame = new envire::FrameNode(); // transform from laser scanner to scanframe
    env->addChild( env->getRootNode(), scanFrame );
    env->addChild( scanFrame, scanMap->getFrameNode() );
    env->addChild( scanFrame, scannerFrame );

    scanNode = new envire::LaserScan();
    env->setFrameNode( scanNode, scannerFrame );
    pcNode = new envire::TriMesh();
    env->setFrameNode( pcNode, scannerFrame );

    smOp = new envire::ScanMeshing();
    smOp->setMaxRange( eslamConfig.maxSensorRange );
    env->attachItem( smOp );
    smOp->addInput( scanNode );
    smOp->addOutput( pcNode );

    mlsOp = new envire::MLSProjection();
    env->attachItem( mlsOp );
    mlsOp->addInput( pcNode );
    mlsOp->addOutput( scanMap );
    mlsOp->useUncertainty( true );
    mlsOp->useNegativeInformation( eslamConfig.gridUseNegativeInformation );

    // camera related setup
    distFrame = new envire::FrameNode();
    env->addChild( scanFrame, distFrame );
    distPc = new envire::TriMesh();
    env->setFrameNode( distPc, distFrame );
    distOp = new envire::DistanceGridToPointcloud();
    distOp->setMaxDistance( eslamConfig.maxSensorRange );
    env->attachItem( distOp );
    distOp->addOutput( distPc );

    distMlsOp = new envire::MLSProjection();
    env->attachItem( distMlsOp );
    distMlsOp->addInput( distPc );
    distMlsOp->addOutput( scanMap );
    distMlsOp->useUncertainty( true );
}

void EmbodiedSlamFilter::processMap( MLSGrid* scanMap, bool match, bool update )
{
    static size_t update_idx = 0;

    std::vector<eslam::PoseEstimator::Particle> &particles( getParticles() );
    for( size_t i=0; i< particles.size(); i++ )
    {
	eslam::PoseEstimator::Particle &p( particles[i] );
	envire::MLSMap *pmap = p.grid.getMap();
	envire::MLSGrid *pgrid = pmap->getActiveGrid().get();

	scanFrame->setTransform( envire::Transform( 
		    Eigen::Translation3d( p.position.x(), p.position.y(), 0 ) *
		    Eigen::AngleAxisd( p.orientation, Eigen::Vector3d::UnitZ() )
		    ) );

 	if( update )
 	{
	    Transform tf = scanFrame->relativeTransform( pgrid->getFrameNode() );
 	    // creating a new map is then a matter of looking if either x
 	    // or y is gone over a portion of the map size.
 	    const double newMapThreshold = eslamConfig.gridSize / 2.0 * eslamConfig.gridThreshold;
	    if( fabs(tf.translation().x()) > newMapThreshold || fabs(tf.translation().y()) > newMapThreshold )
	    {
		// see if we need to select a new grid
		pmap->selectActiveGrid( scanFrame, newMapThreshold );
		pgrid = pmap->getActiveGrid().get();
	    }
	}

	Eigen::Affine3d C_s2p = scanMap->getEnvironment()->relativeTransform( scanMap->getFrameNode(), pgrid->getFrameNode() );

	// merge the scan with the map of the current particle
	envire::MLSGrid::SurfacePatch offsetPatch( p.zPos, p.zSigma );
	offsetPatch.update_idx = update_idx;
	if( match )
	{
	    const size_t sampling = 10;
	    const float sigma = 0.2;
	    float weight = pgrid->match( *scanMap, C_s2p, offsetPatch, sampling, sigma );
	    const float visualWeighting = 0.1;
	    p.weight *= pow( weight, visualWeighting );
	}
	if( update )
	{
	    pgrid->merge( *scanMap, C_s2p, offsetPatch );
	    // mark as modified to trigger updates
	    pgrid->itemModified();
	}
    }

    if( update )
	update_idx++;
}

bool EmbodiedSlamFilter::update( 
	const Eigen::Affine3d& body2odometry, const base::samples::DistanceImage& dimage, 
	const Eigen::Affine3d& camera2body, const base::samples::frame::Frame* timage )
{
    if( eslamConfig.mappingCameraThreshold.test( stereoPose.inverse() * body2odometry * camera2body ) )
    {
	Eigen::Quaterniond orientation( body2odometry.linear() );

	if( !distGrid )
	{
	    // create new grid using the parameters from the distance image
	    distGrid = new envire::DistanceGrid( dimage );

	    // distGrid has just been created and needs to be attached
	    distOp->addInput( distGrid );
	    distGrid->setFrameNode( distPc->getFrameNode() );
	}
	distGrid->copyFromDistanceImage( dimage );

	// if there is a texture image add it to the processing chain
	if( timage )
	{
	    // create new imagegrid object if not available
	    if( !textureGrid )
	    {
		// copy the scaling properties from distanceImage
		// TODO this is a hack!
		textureGrid = new envire::ImageRGB24( 
			dimage.width, dimage.height, 
			dimage.scale_x, dimage.scale_y, 
			dimage.center_x, dimage.center_y );

		distOp->addInput( textureGrid );
		textureGrid->setFrameNode( distPc->getFrameNode() );
	    }
	    textureGrid->copyFromFrame( *timage );
	}

	// run the actual operation
	distOp->updateAll();

	// assume a 2 deg rotation error for the laser2Body transform
	const double scanAngleSigma = 5.0/180.0*M_PI;
	Eigen::Matrix<double,6,1> lcov;
	lcov << scanAngleSigma,0,0, 0,0,0;
	envire::TransformWithUncertainty dist2bodyU( camera2body, lcov.array().square().matrix().asDiagonal());

	// the covariance for the body to world transform comes from
	// a 1 deg error for pitch and roll
	// TODO: actually the errors should be in global frame, and not in body
	// frame... fix later
	const double pitchRollSigma = 3.0/180.0*M_PI;
	Eigen::Matrix<double,6,1> pcov;
	pcov << pitchRollSigma,pitchRollSigma,0, 0,0,0;
	envire::TransformWithUncertainty body2World( 
		Eigen::Affine3d( base::removeYaw(orientation) ), pcov.array().square().matrix().asDiagonal());

	distFrame->setTransform( body2World * dist2bodyU );

	scanMap->clear();
	distMlsOp->updateAll();

	processMap( scanMap, false, true );

	stereoPose = body2odometry * camera2body;

	return true;
    }

    return false;
}

bool EmbodiedSlamFilter::update( const Eigen::Affine3d& body2odometry, const base::samples::LaserScan& scan, const Eigen::Affine3d& laser2body )
{
    if( eslamConfig.mappingThreshold.test( mapPose.inverse() * body2odometry * laser2body ) )
    {
	Eigen::Quaterniond orientation( body2odometry.linear() );

	// convert scan object to pointcloud
	scanNode->lines.clear();
	scanNode->addScanLine( 0, scan );
	smOp->updateAll();

	// assume a 2 deg rotation error for the laser2Body transform
	const double scanAngleSigma = 5.0/180.0*M_PI;
	Eigen::Matrix<double,6,1> lcov;
	lcov << scanAngleSigma,0,0, 0,0,0;
	envire::TransformWithUncertainty laser2bodyU( laser2body, lcov.array().square().matrix().asDiagonal());

	// the covariance for the body to world transform comes from
	// a 1 deg error for pitch and roll
	// TODO: actually the errors should be in global frame, and not in body
	// frame... fix later
	const double pitchRollSigma = 3.0/180.0*M_PI;
	Eigen::Matrix<double,6,1> pcov;
	pcov << pitchRollSigma,pitchRollSigma,0, 0,0,0;
	envire::TransformWithUncertainty body2World( 
		Eigen::Affine3d( base::removeYaw(orientation) ), pcov.array().square().matrix().asDiagonal());

	scannerFrame->setTransform( body2World * laser2bodyU );
	scanMap->clear();
	mlsOp->updateAll();

	bool match = eslamConfig.useVisualUpdate; 
	bool update = !sharedMap;
	processMap( scanMap, match, update );

	mapPose = body2odometry * laser2body;

	return true;
    }
    return false;
}

bool EmbodiedSlamFilter::update( const Eigen::Affine3d& body2odometry, const odometry::BodyContactState& bs, const std::vector<terrain_estimator::TerrainClassification>& ltc )
{
    Eigen::Quaterniond orientation( body2odometry.linear() );

    odometry.update( bs, orientation );
    filter.project( bs, orientation );

    if( eslamConfig.measurementThreshold.test( udPose.inverse() * body2odometry ) || ltc.size() > 0 )
    {
	filter.update( bs, orientation, ltc );
	udPose = body2odometry;

	return true;
    }
    else
	return false;
}

std::vector<eslam::PoseEstimator::Particle>& EmbodiedSlamFilter::getParticles()
{
    return filter.getParticles();
}

size_t EmbodiedSlamFilter::getBestParticleIndex() const
{
    return filter.getBestParticleIndex();
}

base::Affine3d EmbodiedSlamFilter::getCentroid()
{
    return filter.getCentroid().toTransform();
}

