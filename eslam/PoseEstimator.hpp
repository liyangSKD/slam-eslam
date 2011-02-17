#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include "PoseParticle.hpp"
#include "Configuration.hpp"

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/pose.h>
#include <asguard/BodyState.hpp>
#include <asguard/Odometry.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MLSMap.hpp>

#include <limits>

namespace eslam
{

class GridAccess
{
    // caching the transform for faster access
    Eigen::Transform3d C_global2local;
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

    void copy( const GridAccess& other )
    {
	envire::MLSMap::Ptr new_map = other.map->cloneDeep();
	envire::Environment *env = other.map->getEnvironment();
	env->setFrameNode( new_map.get(), other.map->getFrameNode() );
	setMap( MapPtr(new_map.get(), &GridAccess::detachItem) );
    }

    bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev)
    {
	if( map )
	{
	    typedef envire::MultiLevelSurfaceGrid::SurfacePatch Patch;
	    Patch p( position.z(), zstdev );
	    Patch* res = map->getPatch( C_global2local * position, p, 3.0 );
	    if( res )
	    {
		zpos = res->mean;
		zstdev = res->stdev;
		return true;
	    }
	}
	return false;
    }
};

class PoseParticleGA : public PoseParticle
{
public:
    PoseParticleGA( const Eigen::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: PoseParticle( position, orientation, zpos, zsigma, floating ) {} 

    GridAccess grid;
};

class PoseEstimator :
    public ParticleFilter<PoseParticleGA>
{
public:
    PoseEstimator(asguard::odometry::Wheel& odometry, const eslam::Configuration &config, const asguard::Configuration& asguardConfig );
    ~PoseEstimator();

    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos = 0, double zsigma = 0);
    void project(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);
    void update(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);

    void setEnvironment(envire::Environment *env, envire::MLSMap::Ptr map, bool useShared );
    void cloneMaps();

    base::Pose getCentroid();

private:
    void updateWeights(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);
    double weightingFunction( double stdev );

    eslam::Configuration config;
    asguard::Configuration asguardConfig;
    asguard::odometry::Wheel &odometry;
    
    envire::Environment *env;
    bool useShared;

    Eigen::Quaterniond zCompensatedOrientation;
};

}
#endif
