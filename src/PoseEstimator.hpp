#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include "PoseParticle.hpp"
#include "Configuration.hpp"

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Pose.hpp>
#include <odometry/ContactOdometry.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MLSMap.hpp>

#include <eslam/ContactModel.hpp>
#include "SurfaceHash.hpp"

#include <limits>

namespace eslam
{

class GridAccess
{
    // caching the transform for faster access
    base::Affine3d C_global2local;
    typedef boost::shared_ptr<envire::MLSMap> MapPtr;
    MapPtr map;

public:
    static void detachItem( envire::MLSMap* item )
    {
	if(item && item->isAttached() ) 
	{
	    envire::Environment* env = item->getEnvironment();
	    std::list<envire::Layer*> grids = env->getChildren(item);
	    item->detach();

	    // look at all the grids in the map, and see if they 
	    // have other maps that they are attached to.
	    // detach them if this is not the case. Also remove any unused
	    // FrameNodes
	    for( std::list<envire::Layer*>::iterator it = grids.begin(); 
		    it != grids.end(); it++ )
	    {
		envire::Layer* grid = *it;
		if( grid->getParents().empty() )
		{
		    envire::FrameNode* fn = 
			dynamic_cast<envire::MultiLevelSurfaceGrid*>(grid)->getFrameNode();
		    grid->detach();
		    while( fn && fn->getMaps().empty() )
		    {
			envire::FrameNode* parent = fn->getParent();
			fn->detach();
			fn = parent;
		    }
		}
	    }
	}
    }

    void setMap( MapPtr _map ) 
    {
	envire::Environment *env = _map->getEnvironment();
	C_global2local =
	    env->relativeTransform( 
		    env->getRootNode(),
		    _map->getFrameNode() );

	this->map = _map;
    }

    envire::MLSMap* getMap()
    {
	return map.get();
    }

    const envire::MLSMap* getMap() const
    {
	return map.get();
    }

    void copy( const GridAccess& other )
    {
	envire::MLSMap::Ptr new_map = other.map->cloneDeep();
	envire::Environment *env = other.map->getEnvironment();
	env->setFrameNode( new_map.get(), other.map->getFrameNode() );
	setMap( MapPtr(new_map.get(), &GridAccess::detachItem) );
    }

    bool get( const base::Vector3d& position, envire::MLSGrid::SurfacePatch& patch )
    {
	if( map )
	{
	    if( map->getPatch( C_global2local * position, patch, 3.0 ) )
		return true;
	}
	return false;
    }
};

class PoseParticleGA : public PoseParticle
{
public:
    PoseParticleGA( const PoseParticle& p )
	: PoseParticle( p ) {} 
    PoseParticleGA( const base::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: PoseParticle( position, orientation, zpos, zsigma, floating ) {} 

    GridAccess grid;
};

class PoseEstimator :
    public ParticleFilter<PoseParticleGA>
{
public:
    PoseEstimator(odometry::FootContact& odometry, const eslam::Configuration &config);
    ~PoseEstimator();

    void init( int numParticles, SurfaceHash *hash );
    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos = 0, double zsigma = 0);
    void project(const odometry::BodyContactState& state, const base::Quaterniond& orientation);
    void update(const odometry::BodyContactState& state, const base::Quaterniond& orientation, const std::vector<terrain_estimator::TerrainClassification>& ltc );

    void setEnvironment(envire::Environment *env, envire::MLSMap::Ptr map, bool useShared );
    void cloneMaps();

    base::Pose getCentroid();

private:
    void updateWeights(const odometry::BodyContactState& state, const base::Quaterniond& orientation);

    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand_uni;
    base::Pose2D samplePose2D( const base::Pose2D& mu, const base::Pose2D& sigma );
    void sampleFromHash( double replace_percentage, const odometry::BodyContactState& state, const base::Quaterniond& orientation );

    eslam::Configuration config;
    ContactModel contactModel;
    odometry::FootContact &odometry;

    SurfaceHash *hash;
    
    envire::Environment *env;
    bool useShared;

    base::Quaterniond zCompensatedOrientation;
    double max_weight;
};

}
#endif
