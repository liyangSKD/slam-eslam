#include "ContactModel.hpp"

using namespace eslam;

template <class T, int N> 
    bool compareElement(const T& a, const T& b)
{
    return a[N] < b[N];
}

ContactModel::ContactModel( const asguard::Configuration& asguardConfig ) 
    : candidate_group( GROUP_SIZE ),
    asguardConfig( asguardConfig ),
    m_useTerrainUpdate( true ),
    m_useShapeUpdate( true )
{
}

void ContactModel::generateCandidatePoints( const asguard::BodyState& state, const base::Quaterniond& orientation )
{
    // get the orientation first and remove any rotation around the z axis
    base::Quaterniond zCompensatedOrientation = base::removeYaw( orientation );

    for(int i=0;i<4;i++)
    {
	candidate_group[i].clear();
	for(int j=0;j<5;j++) 
	{
	    base::Vector3d f = asguardConfig.getFootPosition( state, static_cast<asguard::wheelIdx>(i), j );
	    candidate_group[i].push_back( zCompensatedOrientation * f );	
	}
	// remove the two wheels with the highest z value
	std::sort( candidate_group[i].begin(), candidate_group[i].end(), compareElement<base::Vector3d,2> );
	candidate_group[i].resize( 3 );
    }
}


bool ContactModel::evaluatePose( const base::Affine3d& pose, double measVar, boost::function<bool (const base::Vector3d&, envire::MLSGrid::SurfacePatch&)> map )
{
    contact_points.clear();

    for( size_t group_idx = 0; group_idx < candidate_group.size(); group_idx++ )
    {
	// find the contact points with the lowest zdiff per wheel
	ContactPoint p;
	bool contact = true;
	for( std::vector<base::Vector3d>::iterator it = candidate_group[group_idx].begin(); it != candidate_group[group_idx].end(); it++ )
	{
	    // get contact point candidate and transform it into world
	    // coordinates using the supplied pose transform
	    const base::Vector3d &contact_point(*it);
	    base::Vector3d contact_point_w = pose * contact_point;

	    // get the relevant surface patch based on the grid point
	    typedef envire::MultiLevelSurfaceGrid::SurfacePatch Patch;
	    Patch patch( contact_point_w.z(), sqrt(measVar) );
	    if( !map( contact_point_w, patch ) )
	    {
		contact = false;
		break;
	    }

	    // find the zdiff, which is the difference between contact
	    // point z-value and environment z-value
	    const double zdiff = contact_point_w.z() - patch.mean;

	    // the point with the lowest zdiff value is assumed to be the
	    // right contact point for the group
	    if( zdiff < p.zdiff )
	    {
		const double zvar = pow(patch.stdev, 2) + measVar;
		p = ContactPoint( contact_point_w - base::Vector3d::UnitZ() * zdiff, zdiff, zvar );

		// also include terrain classification information if it exists
		if( !terrain_classification.empty() )
		{
		    // look for the terrain classification data which
		    // corresponds to the current group_idx (wheel in asguard case)
		    for( size_t i = 0; i < terrain_classification.size(); i++ )
		    {
			if( static_cast<size_t>(terrain_classification[i].wheel_idx) == group_idx )
			{
			    // use the RGB value from the terrain patch to get the terrain class
			    terrain_estimator::TerrainClassification visual_tc =
				terrain_estimator::TerrainClassification::fromRGB( patch.color );

			    // get the joint probability from from visual and proprioceptive 
			    // classification
			    double prob = terrain_classification[i].jointProbability( visual_tc );

			    //std::cout << patch.color.transpose() << std::endl;
			    //std::cout << prob << std::endl;

			    // store this information with the contact point
			    p.prob *= prob;

			    // create debug slip point
			    SlipPoint sp;
			    sp.position = contact_point_w;
			    sp.color = terrain_classification[i].toRGB();
			    sp.prob = prob;
			    slip_points.push_back( sp );
			}
		    }
		}
	    }
	}

	if( contact )
	{
	    contact_points.push_back( p );
	}
    }

    if( contact_points.size() > 0 ) 
    {
	evaluateWeight( measVar );
	return true;
    }

    return false;
}

void ContactModel::evaluateWeight( double measVar )
{
    // calculate the z-delta with the highest combined probability
    // of the individual contact points
    double d1=0, d2=0; 
    for(std::vector<ContactPoint>::iterator it=contact_points.begin(); it!=contact_points.end(); it++)
    {
	ContactPoint &p(*it);
	d1 += p.zdiff/p.zvar;
	d2 += 1.0/p.zvar;
	//std::cout << "p.zdiff: " << p.zdiff << " p.zvar: " << p.zvar << " stdev: " << sqrt(p.zvar) << std::endl;
    }
    const double delta = d1 / d2;

    // calculate the joint probability of the individual foot contact points using the
    // most likely z-height from the previous calculation of delta
    double pz = 1.0;
    for(std::vector<ContactPoint>::iterator it=contact_points.begin(); it!=contact_points.end(); it++)
    {
	ContactPoint &p(*it);
	const double odiff = (p.zdiff - delta)/sqrt(p.zvar);

	const double zk = exp(-(odiff*odiff)/(2.0));
	if( m_useShapeUpdate )
	    pz *= zk;

	// also include the probability stored in the contact point itself
	if( m_useTerrainUpdate )
	    pz *= p.prob;
    }

    const double zd = delta/sqrt( measVar );
    pz *= exp( -(zd*zd)/2.0 );

    m_weight = pz;
    m_zDelta = -delta;
    m_zVar = 1.0/d2;

    /*
       if(false)
       {
       std::cout 
       << "points: " << found_points
       << "\tzPos:" << pose.zPos
       << "\tzSigma:" << pose.zSigma
       << "\tdelta:" << delta 
       << "\td1: " << d1
       << "\td2: " << d2
       << "\tpz: " << pz 
       << std::endl;
       }	
       */
}

ChittaContactModel::ChittaContactModel( const asguard::Configuration& asguardConfig ) 
    :  ContactModel( asguardConfig )
{
}

void ChittaContactModel::evaluateWeight( double measVar )
{
    std::sort( contact_points.begin(), contact_points.end() );

    m_zDelta = -contact_points[0].zdiff;
    m_zVar = measVar;
    double z_t = 0.0;

    for( size_t i=1; i<contact_points.size(); i++ )
    {
	z_t += pow( contact_points[i].zdiff + m_zDelta, 2 );
    }

    // no scaling factor needed here
    m_weight = exp( -z_t / (2.0*measVar) );
}
