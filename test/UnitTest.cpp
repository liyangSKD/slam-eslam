#define BOOST_TEST_MODULE FilterTest 
#include <boost/test/included/unit_test.hpp>

#include <boost/random/normal_distribution.hpp>

#include "ParticleFilter.hpp"

using namespace std;

struct State
{
    double pos;
    double v;
};

typedef double Input;
typedef double Measurement;

class SingleValueTracking :
    public ParticleFilter<State, Input, Measurement>
{
public:
    SingleValueTracking() : 
	dt(0.1) {};

    void init(int numParticles) 
    {
	double sigma = 1;
	boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	    rand(rand_gen, boost::normal_distribution<>(0,sigma) );

	for(int i=0;i<numParticles;i++)
	{
	    State s = {rand(), rand()};
	    xi_k.push_back( Particle( s, 0 ) );
	}
    };

protected:
    double dt;

    void sampleState()
    {
	double sigma = 1;
	boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	    rand(rand_gen, boost::normal_distribution<>(0,sigma) );

	for(int i=0;i<xi_k.size();i++)
	{
	    State &xt(xi_k[i].x);
	    xt.v = xt.v + rand();
	    xt.pos = xt.pos + xt.v * dt;
	}
    };

    void updateWeights()
    {
	for(int i=0;i<xi_k.size();i++)
	{
	    double sigma = 1;
	    double mu = xi_k[i].x.pos;
	    double x = z_k; 
	    double val = 1.0/sqrt(2.0*M_PI*pow(sigma,2.0))*exp(-pow(x-mu,2.0)/(2.0*pow(sigma,2.0))); 
	    xi_k[i].w = val;
	};
    };
};

BOOST_AUTO_TEST_CASE( tracking_filter )
{
    SingleValueTracking filter; 
    filter.init(100);

    double v=0,pos=0;
    double t = 0;
    double dt = 0.1;
    double a = 0;
    
    while(t < 10)
    {
	if( sin(t) > 0.5 )
	    a = 1;
	else if( sin(t) < -0.5 )
	    a = -1;
	else
	    a = 0;
	
	// TODO generally add noise 
	v += a*dt;
	pos += v*dt;
	t += dt;

	filter.project( a );
	filter.update( pos );
	filter.resample();
    }
}

