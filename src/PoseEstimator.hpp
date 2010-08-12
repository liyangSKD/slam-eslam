#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include <vector>

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/pose.h>
#include <asguard/Configuration.hpp>
#include <asguard/Odometry.hpp>

#include <envire/Core.hpp>
#include <envire/GridAccess.hpp>

#include <limits>

namespace eslam
{

struct ContactPoint
{
    ContactPoint() : 
	point( Eigen::Vector3d(0,0,0)), 
	zdiff(std::numeric_limits<double>::infinity()) {}

    ContactPoint(const Eigen::Vector3d& point, double zdiff) :
	point(point),
	zdiff(zdiff) {}

    Eigen::Vector3d point;
    double zdiff;
};

struct PoseParticle : public base::Pose2D
{
    PoseParticle( const Eigen::Vector2d& position, double orientation )
	: base::Pose2D( position, orientation ) {};

    std::vector<ContactPoint> cpoints;
    double zPos;
    double zSigma;

    bool floating;
};

class PoseEstimator :
    public ParticleFilter<PoseParticle>
{
public:
    PoseEstimator(base::odometry::Sampling2D& odometry);
    ~PoseEstimator();

    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma);
    void project(const asguard::BodyState& state);
    void update(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);

    void setEnvironment(envire::Environment *env);

    base::Pose getCentroid();

private:
    void updateWeights(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);
    double weightingFunction( double stdev );

    asguard::Configuration config;
    base::odometry::Sampling2D &odometry;
    
    envire::Environment *env;

    std::auto_ptr<envire::PointcloudAccess> ga;

    Eigen::Quaterniond zCompensatedOrientation;
};

}
#endif
