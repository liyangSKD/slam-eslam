#ifndef ESLAM_SURFACEHASH_HPP__ 
#define ESLAM_SURFACEHASH_HPP__ 

#include <base/pose.h>
#include <envire/maps/MLSGrid.hpp>

#include "PoseParticle.hpp"
#include "Configuration.hpp"

namespace eslam
{

template <class T>
struct Buckets
{
    int count;
    double min_val, max_val;
    std::vector<T> buckets;

    Buckets( int count, double min_val, double max_val, const T& initial = T() )
	: count( count ), min_val( min_val ), max_val( max_val ), buckets( count, initial )
    {
    }

    int bucketIndex( double value ) const
    {
	int idx = (value - min_val) / (max_val - min_val) * count;
	return std::min( count-1, std::max( 0, idx ) );
    }

    T& operator[]( double value )
    {
	return buckets[ bucketIndex( value ) ];
    }

    size_t size() const
    {
	size_t sum = 0;
	for( typename std::vector<T>::const_iterator it = buckets.begin(); it != buckets.end(); it++ )
	{
	    sum += it->size();
	    //std::cout << sum << " " << std::endl; 
	}
	//std::cout << std::endl; 
	return sum;
    }
    
};

struct SurfaceParam
{
    double slope_x;
    double slope_y;

    /** 
     * calculates the slope and roughness parameters from 
     * a set of points. The function is optimized for low 
     * point counts. The minimum number of points is 3.
     */ 
    void fromPoints( const std::vector<base::Vector3d>& points )
    {
	double x=0, y=0, z=0, xx=0, yy=0, xy=0, xz=0, yz=0;
	// make a linear equation system based on the points
	// and solve for coefficients
	// (from: http://stackoverflow.com/questions/1400213/3d-least-squares-plane)
	for( size_t i=0; i<points.size(); i++ )
	{
	    const base::Vector3d &p( points[i] );
	    x += p.x();
	    y += p.y();
	    z += p.z();
	    xx += p.x()*p.x();
	    yy += p.y()*p.y();
	    xy += p.x()*p.y();
	    xz += p.x()*p.z();
	    yz += p.y()*p.z();
	}

	Eigen::Matrix3d A;
	A << xx, xy, x,
	  xy, yy, y,
	  x, y, points.size();

	Eigen::Vector3d b( xz, yz, z );

	Eigen::Vector3d res = A.ldlt().solve( b );
	// the coefficients are now so that 
	// the plane equation is
	// z = (-a/c)x + (-b/c)y + (-d/c)
	
	slope_x = res.x();
	slope_y = res.y();
	/*
	// assume the slope to be the norm of the gradient vector
	slope = sqrt( pow( res.x(), 2 ) + pow( res.y(), 2 ) );
	*/

	/*
	// for the roughness, we remove the slope from the points
	// and calculate the sum of squares
	double sxx = 0;
	for( size_t i=0; i<points.size(); i++ )
	{
	    const base::Vector3d &p( points[i] );
	    double z_diff = p.z() - Eigen::Vector3d( p.x(), p.y(), 1 ).dot( res );
	    sxx += pow( z_diff, 2 );
	}
	roughness = sxx / points.size();
	*/
    }
};

struct SurfaceHash
{
    typedef Buckets< std::vector<PoseParticle> > SlopeYHash;
    typedef Buckets< SlopeYHash > SlopeXHash;

    boost::shared_ptr<SlopeXHash> hash;
    std::vector<PoseParticle> poses;
    
    SurfaceHashConfig config;

    void setConfiguration( const SurfaceHashConfig& c )
    {
	config = c;
    }

    PoseParticle* sample()
    {
	size_t idx = rand() % poses.size();
	return &poses[ idx ];
    }

    double getRelevance( const SurfaceParam& param ) const 
    {
	std::cout << hash->size() << std::endl;
	const std::vector<PoseParticle> &ps( (*hash)[param.slope_x][param.slope_y] );
	return 1.0 - 1.0 * ps.size() / poses.size();
    }

    PoseParticle* sample( const SurfaceParam& param )
    {
	std::vector<PoseParticle> &ps( (*hash)[param.slope_x][param.slope_y] );
	if( ps.size() > 0 )
	{
	    size_t idx = rand() % ps.size();
	    return &ps[ idx ];
	}

	// TODO instead of giving up, we could try returning 
	// close matches
	return NULL;
    }

    void create( envire::MLSGrid *gridTemplate )
    {
	// create metadata which includes some information on slope and
	// roughness
	hash = boost::shared_ptr<SlopeXHash>(
		new SlopeXHash( config.slopeBins, -1.0, 1.0, SlopeYHash( config.slopeBins, -1.0, 1.0 ) ) );

	const double base = 0.5;

	std::vector<base::Vector3d> opoints;
	opoints.push_back( base::Vector3d( base/2.0, 0, 0 ) );
	opoints.push_back( base::Vector3d( -base/2.0, 0, 0 ) );
	opoints.push_back( base::Vector3d( base/2.0, -base, 0 ) );
	opoints.push_back( base::Vector3d( -base/2.0, -base, 0 ) );

	std::vector<base::Vector3d> points = opoints;

	Eigen::Affine3d grid2world = 
	    gridTemplate->getFrameNode()->relativeTransform( gridTemplate->getEnvironment()->getRootNode() );
	double yaw_offset = base::getYaw( Eigen::Quaterniond( grid2world.linear()) );

	std::cerr << "starting hashing... ";

	const size_t angle_segments = config.angularSteps;
	for( size_t a = 0; a < angle_segments; a ++ )
	{
	    double angle = a * 2.0 * M_PI / angle_segments;
	    Eigen::Matrix3d rot = Eigen::AngleAxisd( 2.0*M_PI/angle_segments, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
	    for( size_t n = 0; n < points.size(); n++ )
		points[n] = rot * points[n];

	    for( size_t m = 0; m < gridTemplate->getWidth(); m ++ )
	    {
		for( size_t n = 0; n < gridTemplate->getHeight(); n ++ )
		{
		    double x, y;
		    gridTemplate->fromGrid( m, n, x, y );
		    double mean_z = 0;

		    std::vector<base::Vector3d> gpoints;
		    for( size_t i = 0; i < points.size(); i++ )
		    {
			base::Vector3d &p( points[i] );
			size_t mx, nx;
			gridTemplate->toGrid( x + p.x(), y + p.y(), mx, nx );

			envire::MLSGrid::const_iterator it = gridTemplate->beginCell( mx, nx );
			if( it != gridTemplate->endCell() )
			{
			    double zval = it->mean;
			    mean_z += zval;
			    gpoints.push_back( opoints[i] + base::Vector3d( 0, 0, zval ) );
			}
		    }
		    mean_z /= gpoints.size();

		    if( gpoints.size() >= 3 )
		    {
			SurfaceParam params;
			params.fromPoints( gpoints );

			Eigen::Vector3d pose = grid2world * Eigen::Vector3d( x, y, mean_z );

			PoseParticle particle( base::Vector2d( pose.x(), pose.y() ), angle + yaw_offset, pose.z() + 0.18 );

			(*hash)[params.slope_x][params.slope_y].push_back( particle );
			poses.push_back( particle );

			//std::cout << angle << " " << m << " " << n << " " 
			//	<< params.slope << " " << params.roughness << std::endl;
		    }
		}
	    }
	    std::cerr << a << " ";
	}
	std::cerr << " done. size: " << poses.size() << std::endl;
    }
};

}

#endif
