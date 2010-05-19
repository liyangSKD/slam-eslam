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

namespace eslam
{

class PoseEstimator :
    public ParticleFilter<base::Pose2D>
{
public:
    PoseEstimator();

    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma);
    void project(const asguard::BodyState& state);

private:
    asguard::Configuration config;
    asguard::SamplingOdometry2D odometry;
};

}
#endif
