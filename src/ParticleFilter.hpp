#ifndef __PARTICLE_FILTER_HPP__
#define __PARTICLE_FILTER_HPP__

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

namespace eslam 
{

template <class State_>
class ParticleFilter
{
public:
    typedef State_ State;

    struct Particle
    {
	Particle() : w(0.0) {};
	Particle(State x, double weight) : x(x), w(weight) {};

	State x;
	double w;
    };

    ParticleFilter() :
	rand_gen( 42u )
    {
    };

    double normalizeWeights()
    {
	double sumWeights = 0;
	for(size_t n=0;n<xi_k.size();sumWeights+=xi_k[n++].w);

	double effective = 0;
	if( sumWeights <= 0.0 )
	{
	    for(size_t n=0;n<xi_k.size();n++)
	    {
		double &w(xi_k[n++].w);
		w = 1.0/xi_k.size();
		effective += w*w;
	    }
	}
	else{
	    for(size_t n=0;n<xi_k.size();n++)
	    {
		double &w(xi_k[n++].w);
		w /= sumWeights;
		effective += w*w;
	    }
	}

	return 1.0/effective;
    };

    void resample()
    {
	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > 
	    rand(rand_gen, boost::uniform_real<>(0,1.0) );

	std::vector<Particle> xi_kp;
	for(size_t n=0;n<xi_k.size();n++)
	{
	    double sum=0;
	    double r_n = rand();

	    for(size_t i=0;i<xi_k.size();i++)
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

    std::vector<Particle>& getParticles()
    {
	return xi_k;
    };


protected:
    std::vector<Particle> xi_k;

    size_t particleCount;

    boost::minstd_rand rand_gen;
};

}
#endif
