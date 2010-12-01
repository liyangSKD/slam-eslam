#ifndef __ESLAM_POSEPARTICLE_HPP__
#define __ESLAM_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <base/eigen.h>
#include <base/time.h>
#include <asguard/BodyState.hpp>

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
    PoseParticle( const Eigen::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: position(position), orientation(orientation), zPos(zpos), zSigma(zsigma), floating(floating), weight(0) {};

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
