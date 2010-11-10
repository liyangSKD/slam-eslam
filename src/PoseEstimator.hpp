#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include <vector>

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/pose.h>
#include <asguard/Configuration.hpp>
#include <asguard/Odometry.hpp>

#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

#include <limits>

namespace eslam
{

struct ContactPoint
{
    ContactPoint() : 
	point( Eigen::Vector3d(0,0,0)), 
	zdiff(std::numeric_limits<double>::infinity()) {}

    ContactPoint(const Eigen::Vector3d& point, double zdiff, double zvar) :
	point(point),
	zdiff(zdiff),
	zvar(zvar) 
    {}

    Eigen::Vector3d point;
    double zdiff;
    double zvar;
};

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

struct PoseParticle : public base::Pose2D
{
    PoseParticle( const Eigen::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: base::Pose2D( position, orientation ), zPos(zpos), zSigma(zsigma), floating(floating) {};

    Eigen::Transform3d getPose( const Eigen::Quaterniond& _orientation )
    {
	// get the orientation first and remove any rotation around the z axis
	Eigen::Vector3d projy = _orientation * Eigen::Vector3d::UnitY(); 
	Eigen::Quaterniond ocomp = Eigen::AngleAxisd( -atan2( -projy.x(), projy.y() ), Eigen::Vector3d::UnitZ()) * _orientation;

	Eigen::Vector3d pos( position.x(), position.y(), zPos );
	Eigen::Transform3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( orientation, Eigen::Vector3d::UnitZ() )
	    * ocomp;
	
	return t;
    }

    double zPos;
    double zSigma;

    double mprob;
    bool floating;

    GridAccess grid;

    // debug information
    std::vector<ContactPoint> cpoints;
    Eigen::Vector3d meas_pos;
    double meas_theta;
};

class PoseEstimator :
    public ParticleFilter<PoseParticle>
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

struct PoseDistribution
{
    typedef eslam::ParticleFilter<eslam::PoseParticle>::Particle particle;

    base::Time time;
    std::vector<particle> particles;
    Eigen::Quaterniond orientation;
    asguard::BodyState bodyState;
};

}
#endif
