#ifndef __ESLAM_CONFIGURATION_HPP__
#define __ESLAM_CONFIGURATION_HPP__

namespace eslam 
{

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
	updateThreshDistance( 0.1 ),
	updateThreshAngle( 10*M_PI/180.0 )
    {};

    unsigned long seed;
    size_t particleCount;
    size_t minEffective;
    double initialError;
    double weightingFactor;
    double measurementError;
    double discountFactor;
    double updateThreshDistance;
    double updateThreshAngle;
};

}
#endif
