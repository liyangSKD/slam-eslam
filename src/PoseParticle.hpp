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

#include <envire/tools/GaussianMixture.hpp>
#include <odometry/ContactState.hpp>

namespace eslam
{

struct ContactPoint
{
    ContactPoint() : 
	point( Eigen::Vector3d(0,0,0)), 
	zdiff(std::numeric_limits<double>::infinity()),
        zvar(std::numeric_limits<double>::infinity()),
	prob(1.0) {}

    ContactPoint(const base::Vector3d& point, double zdiff, double zvar) :
	point(point),
	zdiff(zdiff),
	zvar(zvar),
        prob(1.0) {}

    bool operator<( const ContactPoint& other ) const 
    {
	return zdiff < other.zdiff;
    }

    base::Vector3d point;
    double zdiff;
    double zvar;
    double prob;
};

struct SlipPoint
{
    base::Vector3d position;
    base::Vector3d color;
    double prob;
};

struct PoseParticle
{
    PoseParticle() {};
    PoseParticle( const base::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0, bool floating = true )
	: position(position), orientation(orientation), zPos(zpos), zSigma(zsigma), floating(floating), weight(0) {};

    base::Affine3d getPose( const base::Quaterniond& _orientation )
    {
	base::Vector3d pos( position.x(), position.y(), zPos );
	base::Affine3d t = 
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
    std::vector<SlipPoint> spoints;
    base::Vector3d meas_pos;
    double meas_theta;

    // particle weight
    double weight;
};

struct PoseDistribution
{
    // we need to force the GMM model to use the base types
    // here instead of the generic eigen types
    struct BaseAdapter
    {
	enum { Dimension = 2 };
	typedef double Scalar;
	typedef base::Vector2d Vector;
	typedef base::Matrix2d Matrix;
    };

    typedef envire::GaussianMixture<double, 2, BaseAdapter> GMM;
    // Force instanciation of some of the templated code. This is needed for
    // gccxml (and therefore orogen)
    //
    // It is harmless outside these contexts
    struct gccxml_workaround {
	GMM::Parameter field;
    };

    base::Time time;
    std::vector<PoseParticle> particles;
    GMM gmm;
    base::Quaterniond orientation;
    odometry::BodyContactState bodyState;
};

}

#endif
