#ifndef __PARTICLE_FILTER_HPP__
#define __PARTICLE_FILTER_HPP__

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

template <class State_, class Input_, class Measurement_>
class ParticleFilter
{
public:
    typedef State_ State;
    typedef Input_ Input;
    typedef Measurement_ Measurement;

    struct Particle
    {
	Particle() {};
	Particle(State x, double weight) : x(x), w(weight) {};

	State x;
	double w;
    };

    ParticleFilter() :
	rand_gen( 42u )
    {
    };

    void project( Input input )
    {
	u_k = input;
	sampleState();
    };

    void update( Measurement measurement )
    {
	z_k = measurement;
	updateWeights();
    };

    void resample()
    {
	double sumWeights = 0;
	for(int n=0;n<xi_k.size();sumWeights+=xi_k[n++].w);

	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > 
	    rand(rand_gen, boost::uniform_real<>(0,sumWeights) );

	std::vector<Particle> xi_kp;
	for(int n=0;n<xi_k.size();n++)
	{
	    double sum=0;
	    double r_n = rand();

	    for(int i=0;i<xi_k.size();i++)
	    {
		sum += xi_k[i].w;
		if( r_n < sum )
		{
		    xi_kp.push_back( xi_k[i] );
		    break;
		}
	    }
	}

	xi_k = xi_kp;
    };

protected:
    virtual void sampleState() = 0;
    virtual void updateWeights() = 0;

    Input u_k;
    Measurement z_k;

    std::vector<Particle> xi_k;

    size_t particleCount;

    boost::minstd_rand rand_gen;
};

#endif
