#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include "PoseParticle.hpp"

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/pose.h>
#include <asguard/BodyState.hpp>
#include <asguard/Odometry.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

#include <limits>

namespace eslam
{

class GridAccess
{
    Eigen::Transform3d C_global2local;
    boost::shared_ptr<envire::MultiLevelSurfaceGrid> mlsGrid;

public:
    void setGrid( boost::shared_ptr<envire::MultiLevelSurfaceGrid> mlsGrid ) 
    {
	envire::Environment *env = mlsGrid->getEnvironment();
	C_global2local =
	    env->relativeTransform( 
		    env->getRootNode(),
		    mlsGrid->getFrameNode() );

	this->mlsGrid = mlsGrid;
    }

    envire::MultiLevelSurfaceGrid* get()
    {
	return mlsGrid.get();
    };

    bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev)
    {
	if( mlsGrid )
	    return mlsGrid->get( C_global2local * position, zpos, zstdev );
	else
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
    PoseEstimator(base::odometry::Sampling2D& odometry, asguard::Configuration &config );
    ~PoseEstimator();

    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos = 0, double zsigma = 0);
    void project(const asguard::BodyState& state);
    void update(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);

    void setEnvironment(envire::Environment *env, boost::shared_ptr<envire::MultiLevelSurfaceGrid> grid, bool useShared );
    void cloneMaps();

    base::Pose getCentroid();

private:
    void updateWeights(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);
    double weightingFunction( double stdev );

    asguard::Configuration &config;
    base::odometry::Sampling2D &odometry;
    
    envire::Environment *env;
    bool useShared;

    Eigen::Quaterniond zCompensatedOrientation;
};

}
#endif
