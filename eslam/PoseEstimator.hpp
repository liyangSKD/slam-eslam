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
    envire::MLSMap::Ptr map;

public:
    void setMap( envire::MLSMap::Ptr _map ) 
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
	envire::MLSMap::Ptr new_map = other.map->clone();
	envire::Environment *env = other.map->getEnvironment();
	env->setFrameNode( new_map.get(), other.map->getFrameNode() );
	setMap( new_map );
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
