#ifndef __ESLAM_EXPECTATION_MAXIMIZATION__
#define __ESLAM_EXPECTATION_MAXIMIZATION__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <iterator>

namespace eslam
{

template <class _Scalar, int _Dimension>
    class GaussianMixture
{
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum { Dimension = _Dimension };
    typedef _Scalar Scalar;
    typedef typename Eigen::Matrix<Scalar, Dimension, 1> Vector;
    typedef typename Eigen::Matrix<Scalar, Dimension, Dimension> Matrix;

    struct Parameter
    {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Parameter() {}
	Parameter( Scalar weight, const Vector& mean, const Matrix& cov )
	    : weight( weight ), mean( mean ), cov( cov ) {}

	Scalar eval( const Vector& x ) const
	{
	    Vector d = x - mean;
	    Eigen::Matrix<Scalar,1,1> e = d.transpose() * cov.inverse() * d;
	    Scalar res = 
		pow( 2.0 * M_PI, -Dimension/2.0 )
		* sqrt( cov.determinant() )
		* exp( -0.5 * e[0] );

	    return res;
	}

	Scalar weight;
	Vector mean;
	Matrix cov;
    };

    typedef std::vector<Parameter> Parameters;
    Parameters params;

    /** normalize the weights of the gaussians, so that the sum is 1.0 
     */
    void normalize()
    {
	Scalar sum = 0.0;
	for( size_t n=0; n<params.size(); n++ )
	    sum += params[n].weight; 

	for( size_t n=0; n<params.size(); n++ )
	{
	    if( sum > 1e-5 )
		params[n].weight /= sum; 
	    else
		params[n].weight = 1.0 / params.size();
	}
    }

    Scalar eval( const Vector& s ) const
    {
	Scalar v = 0;
	for( size_t j=0; j<params.size(); j++ )
	    v += params[j].eval( s );

	return v;
    }
};

template <class _Scalar, int _Dimension>
class GaussianMixtureSampling
{
public:
    enum { Dimension = _Dimension };
    typedef _Scalar Scalar;
    typedef GaussianMixture<_Scalar, _Dimension> GMM;
    typedef typename GMM::Matrix Matrix;
    typedef typename GMM::Vector Vector;
    GMM& gmm;

    boost::minstd_rand rand_gen;
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand_uni;

    GaussianMixtureSampling( GMM& gmm, long seed = 42u )
	: gmm( gmm ), rand_gen( seed ),
	  rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
	  rand_uni(rand_gen, boost::uniform_real<>(0,1.0) )
    {
    }

    Vector sample()
    {
	Vector sample;
	for( int i=0; i<Dimension; i++ )
	    sample[i] = rand_norm();

	Scalar r = rand_uni();
	Scalar s = 0.0;
	for( size_t j=0; j<gmm.params.size(); j++ )
	{
	    typename GMM::Parameter &param( gmm.params[j] );
	    s += param.weight;
	    if( s >= r )
		return param.mean + param.cov.llt().matrixL() * sample;
	}

	throw std::runtime_error("distribution not normalized.");
    }
};

template <class _Scalar, int _Dimension>
class ExpectationMaximization
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum { Dimension = _Dimension };
    typedef _Scalar Scalar;
    typedef GaussianMixture<_Scalar, _Dimension> GMM;
    typedef typename GMM::Matrix Matrix;
    typedef typename GMM::Vector Vector;

    GMM gmm;
    std::vector<Vector> samples;
    std::vector<Scalar> weights;

    void run( Scalar delta, size_t max_iter )
    {
	size_t iter = 0;
	Scalar ll_diff = 1e5;
	Scalar last_ll = 0; 

	while( iter <  max_iter && ll_diff > delta )
	{
	    Scalar ll = step();
	    ll_diff = abs( ll - last_ll );
	    last_ll = ll;
	}

	std::cout << "EM: steps " << iter << "diff_ll: " << ll_diff << std::endl;
    }

    Scalar step()
    {
	//
	// algorithm is based on the document
	// EM Demystified: An Expectation-Maximization Tutorial
	// https://www.ee.washington.edu/techsite/papers/documents/UWEETR-2010-0002.pdf
	//

	const size_t k = gmm.params.size();
	const size_t n = samples.size();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> gamma((int)n, (int)k);

	// evaluate the current gmm
	Scalar loglhood = 0;
	for( size_t i = 0; i < n; i++ )
	{
	    Scalar sum = 0;
	    for( size_t j = 0; j < k; j++ )
	    {
		typename GMM::Parameter &param( gmm.params[j] );
		Scalar val = param.weight * param.eval( samples[i] );
		//if( weights.size() )
		//    val = pow( val, weights[i] );
		gamma(i,j) = val;
		sum += val;
	    }
	    loglhood += log( sum );
	    gamma.row(i) /= sum;
	}
	loglhood /= n;

	// update the parameters of the gaussians
	for( size_t j = 0; j < k; j++ )
	{
	    Scalar n_m = gamma.col(j).sum();
	    typename GMM::Parameter &param( gmm.params[j] );
	    param.weight = n_m / n;

	    Vector mean = Vector::Zero();
	    for( size_t i = 0; i < n; i++ )
		mean += gamma(i,j) * samples[i];
	    param.mean = 1.0/n_m * mean;

	    Matrix cov = Matrix::Zero();
	    for( size_t i = 0; i < n; i++ )
	    {
		Vector c = samples[i] - param.mean;
		cov += gamma(i,j) * c * c.transpose();
	    }
	    param.cov = 1.0/n_m * cov;
	}

	return loglhood;
    }

    void initialize( size_t numClasses, const std::vector<Vector>& s, const std::vector<Scalar>& w = std::vector<Scalar>() )
    {
	// copy samples and weights
	samples.resize( s.size() );
	std::copy( s.begin(), s.end(), samples.begin() );

	weights.resize( w.size() );
	std::copy( w.begin(), w.end(), weights.begin() );

	// if weights are given, they need to be the same size as the samples
	assert( !w.size() || w.size() == s.size() );

	Matrix Sxx = Matrix::Zero();
	Vector Sx = Vector::Zero();

	// for now use global mean and covariance of the samples as initialization
	for( typename std::vector<Vector>::const_iterator it = samples.begin(); it != samples.end(); it++ )
	{
	    const Vector &x( *it );
	    Sx += x;
	    Sxx += x * x.transpose();
	}

	Vector mean = Sx / samples.size();
	Matrix cov = Sxx / samples.size() - mean*mean.transpose(); 

	// create num classes in the gmm wich are all initialized the same
	typename GMM::Parameter p( 1.0, Vector::Zero(), Matrix::Identity() );
	gmm.params.resize( numClasses, p );
	for( size_t i=0; i<gmm.params.size(); i++ )
	{
	    gmm.params[i].mean[0] += i*2.0;
	}

	gmm.normalize();
    }
};

}

#endif
