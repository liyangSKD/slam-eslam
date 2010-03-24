#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include <vector>

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace eslam
{

struct Pose2D
{
    Pose2D();
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {};

    double x;
    double y;
    double theta;
};

struct WheelOdometry
{
    double wheel_pos[4];
    double delta[4];
};

typedef std::vector<Eigen::Vector3d> ContactPoints;

class PoseEstimator :
    public ParticleFilter<Pose2D, WheelOdometry, ContactPoints>
{
public:
    PoseEstimator() {};

    void init(int numParticles, const Pose2D& mu, const Pose2D& sigma);

protected:
    void sampleState();
    void updateWeights();
};

}
#endif
