#include "ContactModel.hpp"

using namespace eslam;

template <class T, int N> 
    bool compareElement(const T& a, const T& b)
{
    return a[N] < b[N];
}

ContactModel::ContactModel() 
    : m_useShapeUpdate( true ),
    m_useTerrainUpdate( true ),
    m_minContacts( 1 )
{
}


void ContactModel::setContactPoints( const BodyContactState& state, const base::Quaterniond& orientation )
{
    // generate a copy of the provided contact points, transform them using the
    // given orientation and possibly augment probabilities based on group
    // information.

    // copy the contact points
    contactState = state;

    // Clear the lowestPointsPerGroup set. It is updated lazily if
    // getLowestPointPerGroup is called
    lowestPointsPerGroup.clear();

    // get the orientation first and remove any rotation around the z axis
    base::Quaterniond yawCompensatedOrientation = base::removeYaw( orientation );

    std::vector<BodyContactPoint> &contactPoints( contactState.points );
    for(size_t i=0; i<contactPoints.size(); i++)
	// store the foot position in yaw compensated rotation frame 
	contactPoints[i].position = yawCompensatedOrientation * contactPoints[i].position;
}

BodyContactState const& ContactModel::getContactState() const
{
    return contactState;
}

void ContactModel::lowestPointHeuristic(bool update_probabilities)
{
    lowestPointsPerGroup.clear();
    std::vector<BodyContactPoint> &contactPoints( contactState.points );
    std::vector<std::pair<double, int> > group;

    for(size_t i=0; i<contactPoints.size(); i++)
    {
	if( contactPoints[i].groupId >= 0 )
	{
	    // add to group
	    group.push_back( std::make_pair( contactPoints[i].position.z(), i ) );
            if (update_probabilities)
                contactPoints[i].contact = 0;
	}

	if (group.empty())
	    lowestPointsPerGroup.push_back( contactPoints[i].position );
        else if (i+1 == contactPoints.size() 
		    || contactPoints[i+1].groupId != contactPoints[i].groupId )
	{
	    // Group finished, sort by z value 
	    std::sort( group.begin(), group.end() );

            BodyContactPoint& contact = contactPoints[group[0].second];
	    lowestPointsPerGroup.push_back( contact.position );
            if (update_probabilities)
                contact.contact = 1;
	    group.clear();
	}
    }
}

void ContactModel::updateContactStateUsingLowestPointHeuristic()
{
    lowestPointHeuristic(true);
}

const std::vector<base::Vector3d>& ContactModel::getLowestPointPerGroup()
{
    if (lowestPointsPerGroup.empty())
        lowestPointHeuristic(false);

    return lowestPointsPerGroup;
}

bool ContactModel::evaluatePose( 
	const base::Affine3d& pos_and_heading, 
	double measVar, 
	boost::function<bool (const base::Vector3d&, envire::MLSGrid::SurfacePatch&)> map )
{
    if (measVar == 0)
        throw std::runtime_error("using a zero measurement variance leads to singularities");

    // this funtion finds and stores environment contact points
    contact_points.clear();
    std::vector<BodyContactPoint> &contactPoints( contactState.points );

    // Loop over all the contact points, and add valid contacts to the vector
    // of contact_points (which are effectively environment contact points)
    // It's a little bit twisted because of the groupId thing.  In a group,
    // only use the contact_point with the lowest z-value in this group. 
    ContactPoint p;
    bool valid = false; // validity of current contact point
    const double contact_threshold = 0.2; // fixed for now
    for(size_t i=0; i<contactPoints.size(); i++)
    {
	int groupId = contactPoints[i].groupId;

	// get contact point candidate and transform it into world
	// coordinates using the supplied pose transform
	const base::Vector3d &contact_point( contactPoints[i].position );
	base::Vector3d contact_point_w = pos_and_heading * contact_point;

	// get the relevant surface patch based on the grid point
	typedef envire::MultiLevelSurfaceGrid::SurfacePatch Patch;
	Patch patch( contact_point_w.z(), sqrt(measVar) );
	if( contactPoints[i].contact > contact_threshold && map( contact_point_w, patch ) )
	{
	    // find the zdiff, which is the difference between contact
	    // point z-value and environment z-value
	    const double zdiff = contact_point_w.z() - patch.mean;

	    // the point with the lowest zdiff value is assumed to be the
	    // right contact point for the group
	    if( !valid || zdiff < p.zdiff )
	    {
		const double zvar = pow(patch.stdev, 2) + measVar;
		p = ContactPoint( base::Vector3d(contact_point_w.x(), contact_point_w.y(), patch.mean), zdiff, zvar );
	    }
	    valid = true;
	}

	// in case of groups only evaluate at the end of a group interval
	if( valid && ( groupId == -1 
		    || i+1 == contactPoints.size() 
		    || groupId != contactPoints[i + 1].groupId) )
	{
	    // also include terrain classification information if it exists
	    if( !terrain_classification.empty() )
	    {
		// look for the terrain classification data which
		// corresponds to the current group_idx (wheel in asguard case)
		for( size_t i = 0; i < terrain_classification.size(); i++ )
		{
		    if( static_cast<size_t>(terrain_classification[i].wheel_idx) == groupId )
		    {
			// use the RGB value from the terrain patch to get the terrain class
			terrain_estimator::TerrainClassification visual_tc =
			    terrain_estimator::TerrainClassification::fromRGB( patch.getColor() );

			// get the joint probability from from visual and proprioceptive 
			// classification
			double prob = terrain_classification[i].jointProbability( visual_tc );

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
	    contact_points.push_back( p );
	    valid = false;
	}
    }

    if( contact_points.size() >= m_minContacts ) 
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

ChittaContactModel::ChittaContactModel() 
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
