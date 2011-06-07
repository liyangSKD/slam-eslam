#ifndef __ESLAM_CONFIGURATION_HPP__
#define __ESLAM_CONFIGURATION_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eslam 
{

struct UpdateThreshold
{
    UpdateThreshold() {};
    UpdateThreshold( double distance, double angle )
	: distance( distance ), angle( angle ) {};

    bool test( double distance, double angle )
    {
	return distance > this->distance || angle > this->angle;
    }

    bool test( const Eigen::Affine3d& pdelta )
    {
	return test( Eigen::AngleAxisd( pdelta.linear() ).angle(), pdelta.translation().norm() );
    }

    double distance;
    double angle;
};

struct Configuration
{
    Configuration() :
	seed( 42u ),
	particleCount( 250 ), 
	minEffective( 50 ), 
	initialError(0.1), 
	weightingFactor( 0.1 ),
	measurementError( 0.1 ),
	discountFactor( 0.9 ),
	spreadThreshold( 0.9 ),
	spreadTranslationFactor( 0.1 ),
	spreadRotationFactor( 0.05 ),
	slipFactor( 0.05 ),
	measurementThreshold( 0.1, 10*M_PI/180.0 ),
	mappingThreshold( 0.02, 5*M_PI/180.0 ),
	mappingCameraThreshold( 1.0, 30*M_PI/180.0 )
    {};

    unsigned long seed;
    size_t particleCount;
    size_t minEffective;
    double initialError;
    double weightingFactor;
    double measurementError;
    double discountFactor;
    double spreadThreshold;
    double spreadTranslationFactor;
    double spreadRotationFactor;
    double slipFactor;
    UpdateThreshold measurementThreshold;
    UpdateThreshold mappingThreshold;
    UpdateThreshold mappingCameraThreshold;
};

}
#endif
