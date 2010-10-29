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
#include <envire/tools/GridAccess.hpp>

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

struct PoseParticle : public base::Pose2D
{
    PoseParticle( const Eigen::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: base::Pose2D( position, orientation ), zPos(zpos), zSigma(zsigma), floating(floating) {};

    double zPos;
    double zSigma;

    double mprob;
    bool floating;

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

    void setEnvironment(envire::Environment *env);

    base::Pose getCentroid();

private:
    void updateWeights(const asguard::BodyState& state, const Eigen::Quaterniond& orientation);
    double weightingFunction( double stdev );

    asguard::Configuration &config;
    base::odometry::Sampling2D &odometry;
    
    envire::Environment *env;

    std::auto_ptr<envire::MLSAccess> ga;

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
