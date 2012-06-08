#include "PoseEstimator.hpp"
#include <algorithm>
#include <envire/tools/GridAccess.hpp>

#include <stdexcept>
#include <set>

#include <omp.h>
#include <boost/bind.hpp>

using namespace eslam;

PoseEstimator::PoseEstimator( odometry::FootContact& odometry, const eslam::Configuration &config, const asguard::Configuration& asguardConfig )
    : ParticleFilter<Particle>(config.seed), 
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    config(config), 
    contactModel(),  
    odometry(odometry), 
    hash(NULL),
    env(NULL), 
    max_weight(0)
{
    contactModel.setConfiguration( config.contactModel );
}

PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::cloneMaps()
{
    // this function will make sure that no two particles will point to the same map
    // this works by cloning maps if they are referenced more than once
    std::set<envire::EnvironmentItem*> used;

    for( std::vector<Particle>::iterator it = xi_k.begin(); it != xi_k.end(); it++ )
    {
	envire::EnvironmentItem* grid = it->grid.getMap();
	if( !used.insert( grid ).second )
	{
	    it->grid.copy( it->grid ); 	
	}
    }
}

void PoseEstimator::setEnvironment(envire::Environment *env, envire::MLSMap::Ptr map, bool useShared )
{
    assert(env);
    assert(map);

    this->env = env;
    this->useShared = useShared;

    boost::shared_ptr<envire::MLSMap> pMap( map.get(), &GridAccess::detachItem );

    for( std::vector<Particle>::iterator it = xi_k.begin(); it != xi_k.end(); it++ )
	it->grid.setMap( pMap );

    if( !useShared )
	cloneMaps();
}

base::Pose2D PoseEstimator::samplePose2D( const base::Pose2D& mu, const base::Pose2D& sigma )
{
    double x = rand_norm(), y = rand_norm(), theta = rand_norm();

    return base::Pose2D( 
	    base::Vector2d( 
		x * sigma.position.x() + mu.position.x(),
		y * sigma.position.y() + mu.position.y() ),
	    theta * sigma.orientation + mu.orientation );
}

void PoseEstimator::init( int numParticles, SurfaceHash* hash ) 
{
    this->hash = hash;
    for(int i=0;i<numParticles;i++)
    {
	PoseParticle* pp = hash->sample(); 
	if( pp )
	    xi_k.push_back( Particle( *pp ) );
	else
	    throw std::runtime_error( "could not sample from pose hash." );
    }
}

void PoseEstimator::init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos, double zsigma) 
{
    for(int i=0;i<numParticles;i++)
    {
	base::Pose2D sample = samplePose2D( mu, sigma );

	xi_k.push_back( 
		Particle( 
		    sample.position, 
		    sample.orientation,
		    zpos,
		    zsigma
		    ));
    }
}

/**
 * this is a piecewise linear function, with 
 * @param x as the function input
 * @param alpha minimum threshold any value of x below this value will be 1.0
 * @param beta any x above beta will be set to gamma
 * @param gamma result if x is above beta
 *
 * the interval between alpha and beta is linear, so that the function is
 * continous.
 */
double weightingFunction( double x, double alpha = 0.1, double beta = 0.9, double gamma = 0.05 )
{
    if( x < alpha )
	return 1.0;
    if( x < beta )
    {
	double a = (1.0-gamma)/(alpha-beta);
	double b = 1.0-alpha*a;
	return a*x + b; 
    }
    if( x >= beta )
	return gamma;

    return 0.0;
}

void PoseEstimator::sampleFromHash( double replace_percentage, const odometry::BodyContactState& state, const base::Quaterniond& orientation )
{
    assert( hash );
    // use the hash function to spawn new particles 
    // if we have a single map

    // get the lowest wheel points for each wheel
    contactModel.setContactPoints( state, orientation );
    std::vector<base::Vector3d> points;
    points = contactModel.getLowestPointPerGroup();

    // and generate the surface params based on those
    SurfaceParam params;
    params.fromPoints( points );

    // get the particles with the lowest weight
    typedef std::pair<float, unsigned long> weight_index;
    std::vector<weight_index> widxs( xi_k.size() );
    for(size_t i=0;i<xi_k.size();i++)
    {
	widxs[i] = weight_index( xi_k[i].weight, i );
    }
    std::sort( widxs.begin(), widxs.end() );

    // and replace x percent of them with newly sampled
    // ones
    double relevance_factor = pow(hash->getRelevance( params ),3);
    size_t replace_count = widxs.size() * replace_percentage * relevance_factor;
    if( relevance_factor < 0.8 )
	replace_count = 0;

    std::cout << "replacing : " << replace_count << " relevance: " << relevance_factor << std::endl;
    double weight = getWeightAvg() * hash->config.avgFactor * relevance_factor;
    //std::cerr << "resampling " << replace_count << " particles using hash...";
    for(size_t i=0;i<replace_count;i++)
    {
	PoseParticle* pp = hash->sample( params ); 
	if( pp )
	{
	    Particle &pose(xi_k[widxs[i].second]);

	    pose.position = pp->position;
	    pose.orientation = pp->orientation;
	    pose.zPos = pp->zPos;
	    pose.zSigma = 0.5;
	    pose.floating = true;
	    pose.weight = weight; 

	    //std::cout << xi_k[i].position << std::endl;
	}
    }
    //std::cerr << "done." << std::endl;
}

void PoseEstimator::project(const odometry::BodyContactState& state, const base::Quaterniond& orientation)
{
    zCompensatedOrientation = base::removeYaw( orientation );
    Eigen::Affine3d dtrans = orientation * odometry.getPoseDelta().toTransform();
    const double z_delta = dtrans.translation().z();
    
    //const double z_var = 1e-3;
    const double z_var = odometry.getPositionError()(2,2) * 2.0;

    double spread = weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 );

    for(size_t i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample2D();
	if( rand_uni() < config.slipFactor )
	{
	    delta.position.y() *= rand_uni();
	}

	Particle &p( xi_k[i] );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;

	p.zPos += z_delta;
	p.zSigma = sqrt( p.zSigma*p.zSigma + z_var );

	// the particle with the lowest weight
	// is below a threshold. depending on what sort
	// of mode we are in, do different things.
	if( spread > 0 && !hash ) 
	{
	    // otherwise just spread out the particles and see if we can 
	    // recover this way.
	    const double trans_fac = config.spreadTranslationFactor * spread;
	    const double rot_fac = config.spreadRotationFactor * spread;
	    base::Pose2D sample = samplePose2D( 
		    base::Pose2D(), 
		    base::Pose2D( base::Vector2d( trans_fac, trans_fac ), rot_fac ) );

	    p.position += sample.position;
	    p.orientation += sample.orientation;
	}
    }

    static int count = 0;
    if( hash && (((count++) % hash->config.period) == 0) )
	sampleFromHash( hash->config.percentage, state, orientation );
}

void PoseEstimator::update(const odometry::BodyContactState& state, const base::Quaterniond& orientation, const std::vector<terrain_estimator::TerrainClassification>& ltc )
{
    contactModel.setTerrainClassification( ltc );
    updateWeights(state, orientation);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
	if( !useShared )
	    cloneMaps();
    }
}

void PoseEstimator::updateWeights(const odometry::BodyContactState& state, const base::Quaterniond& orientation)
{
    if( !env )
	throw std::runtime_error("No environment attached.");

    contactModel.setContactPoints( state, orientation );

    size_t total_points = 0;
    size_t data_particles = 0;
    double sum_data_weights = 0.0;

    double last_max_weight = max_weight;
    max_weight = 0;

    // now update the weights of the particles by calculating the variance of the contact points 
#ifdef USE_OPENMP
#warning "using OpenMP"
#pragma omp parallel for
#endif
    for(size_t i=0;i<xi_k.size();i++)
    {
	Particle &pose(xi_k[i]);
	base::Vector3d pos( pose.position.x(), pose.position.y(), pose.zPos );
	base::Affine3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( pose.orientation, Eigen::Vector3d::UnitZ() );

	// store some debug information in the particle
	pose.meas_pos = pos; 
	pose.meas_theta = pose.orientation;

	if( contactModel.evaluatePose( 
		t, 
		pow(pose.zSigma,2) + pow(config.measurementError,2), 
		boost::bind( &GridAccess::get, pose.grid, _1, _2 ) ) )
	{
	    // update z position and sigma
	    double zVar = pow( pose.zSigma, 2 );
	    contactModel.updateZPositionEstimate( pose.zPos, zVar );
	    pose.zSigma = sqrt( zVar );

	    // use some measurement of the variance as the weight 
	    const double weight = contactModel.getWeight();
	    xi_k[i].weight *= weight;
	    xi_k[i].mprob = weight;
	    xi_k[i].floating = false;

	    // store the current maximum weight
	    max_weight = std::max( max_weight, weight );
	    
	    data_particles ++;
	    const size_t found_points = contactModel.getContactPoints().size();
	    sum_data_weights += pow( weight, 1.0/found_points );
	    total_points += found_points;
	}
	else
	{
	    // slowly reduce likelyhood of particles with no measurements
	    // and mark them as floating
	    xi_k[i].floating = true;
	    xi_k[i].mprob = 1.0;
	    //xi_k[i].w *= 0.99;
	}

	// make logging of debug data optional, since it really makes the logs quite big
	pose.cpoints.swap( contactModel.getContactPoints() );
	if( config.logDebug )
	    std::copy( contactModel.getSlipPoints().begin(), contactModel.getSlipPoints().end(), std::back_inserter( pose.spoints ) );

	contactModel.getSlipPoints().clear();
    }

    const double floating_weight = data_particles>0 ? sum_data_weights/data_particles : 1.0;
    //std::cout << "fw: " << floating_weight << " sum_data_weights: " << sum_data_weights << " data_particles: " << data_particles << std::endl;

    for(std::vector<Particle>::iterator it=xi_k.begin();it!=xi_k.end();it++)
    {
	//if((*it).x.cpoints.size() < 4)
	//{
	double factor = (*it).mprob * pow(config.discountFactor*floating_weight, 4-(*it).cpoints.size());
	//if( (*it).x.floating )
	    //factor *= 0.8;
	
	(*it).weight *= factor;
	//}
	//
	if( !config.logDebug )
	    (*it).cpoints.clear();
    }

    if( total_points == 0 )
	max_weight = last_max_weight * config.discountFactor;

    static int iter = 0;
    std::cerr << "iteration: " << iter++ << "\tfound: " << total_points << "\tmax: " << xi_k.size() << "       \r";
}

base::Pose PoseEstimator::getCentroid()
{
    // calculate the weighted mean for now
    base::Pose2D mean;
    double zMean = 0.0;
    double sumWeights = 0.0;
    for(size_t i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	//if( !particle.x.floating )
	{
	    mean.position += particle.position * particle.weight;
	    mean.orientation += particle.orientation * particle.weight;
	    zMean += particle.zPos * particle.weight;
	    sumWeights += particle.weight;
	}
    }
    mean.position /= sumWeights;
    mean.orientation /= sumWeights;
    zMean /= sumWeights;

    // and convert into a 3d position
    base::Pose result( 
	    base::Vector3d( mean.position.x(), mean.position.y(), zMean ), 
	    Eigen::AngleAxisd( mean.orientation, Eigen::Vector3d::UnitZ() ) * zCompensatedOrientation );

    return result;
}
