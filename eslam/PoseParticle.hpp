#ifndef __ESLAM_POSEPARTICLE_HPP__
#define __ESLAM_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/eigen.h>
#include <base/pose.h>
#include <base/time.h>
#include <asguard/BodyState.hpp>

#include <vector>

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

    base::Vector3d point;
    double zdiff;
    double zvar;
};

struct PoseParticle
{
    PoseParticle() {};
    PoseParticle( const Eigen::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: position(position), orientation(orientation), zPos(zpos), zSigma(zsigma), floating(floating), weight(0) {};

    Eigen::Transform3d getPose( const Eigen::Quaterniond& _orientation )
    {
	Eigen::Vector3d pos( position.x(), position.y(), zPos );
	Eigen::Transform3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( orientation, Eigen::Vector3d::UnitZ() )
	    * base::removeYaw( _orientation );
	
	return t;
    }

    base::Vector2d position;
    double orientation;

    double zPos;
    double zSigma;

    double mprob;
    bool floating;

    // debug information
    std::vector<ContactPoint> cpoints;
    base::Vector3d meas_pos;
    double meas_theta;

    // particle weight
    double weight;
};

struct PoseDistribution
{
    base::Time time;
    std::vector<PoseParticle> particles;
    base::Quaterniond orientation;
    asguard::BodyState bodyState;
};

}

#endif
