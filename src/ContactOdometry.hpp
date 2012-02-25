#ifndef CONTACT_ODODOMETRY_HPP__
#define CONTACT_ODODOMETRY_HPP__

#include <eslam/ContactModel.hpp>
#include <asguard/Configuration.hpp>
#include <base/odometry.h>

namespace odometry
{

class FootContact : 
    public base::odometry::Gaussian3D,
    public base::odometry::Sampling3D,
    public base::odometry::Sampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FootContact(const asguard::odometry::Configuration& config);
    void update(const eslam::BodyContactPoints& state, const Eigen::Quaterniond& orientation);

    base::Pose getPoseDelta();
    Eigen::Matrix3d getPositionError();
    Eigen::Matrix3d getOrientationError();
    Matrix6d getPoseError();

public:
    base::Pose getPoseDeltaSample();
    base::Pose2D getPoseDeltaSample2D();

private:
    Eigen::Quaterniond orientation;
    eslam::BodyContactPoints state;

    /** Odometry configuration */
    asguard::odometry::Configuration config;

    GaussianSamplingPose3D sampling;
};

}

#endif
