#ifndef __ESLAM_CONTACTMODEL_HPP__
#define __ESLAM_CONTACTMODEL_HPP__

#include <boost/function.hpp>
#include <base/eigen.h>
#include <eslam/PoseParticle.hpp>
#include <terrain_estimator/TerrainConfiguration.hpp>

namespace eslam
{

template <class T, int N> 
    bool compareElement(const T& a, const T& b)
{
    return a[N] < b[N];
}

/** 
 * Contactmodel class that relates the kinematic configuration of a robot with
 * an environment model. As it is, the class implements the asguard model, but
 * could be generalized for other models as well.
 *
 * The class works in two steps.
 *
 * 1. canditate contact points are generated based on an orientation reading
 * from an IMU. This includes all possible contact points and uses a heuristic
 * to remove completely unlikely points.
 *
 * 2. evaluatePose is used to see how a given robot pose aligns with the given
 * environment model (e.g. map). This will also calculate the delta for the up
 * axis between the system and environment model.
 *
 * All calculations are done probabilistically. If you don't use a probabilistic
 * model, it should be possible to just supply fixed values.
 */
class ContactModel
{
protected:
    typedef std::vector<base::Vector3d> vec3array;
    std::vector<vec3array> candidate_group;

    std::vector<ContactPoint> contact_points;

    asguard::Configuration asguardConfig;

    std::vector<terrain_estimator::TerrainClassification> terrain_classification;

    double m_zDelta;
    double m_zVar;
    double m_weight;

public:
    static const int GROUP_SIZE = 4;

    /** Constructor that takes a @param asguardConfig configuration model as the
     * basis.
     */
    ContactModel( const asguard::Configuration& asguardConfig ) 
	: candidate_group( GROUP_SIZE ),
	  asguardConfig( asguardConfig )
    {
    }

    /** given a @param state configuration state of the system and an @param
     * orientation, candidate contact points are calculated in the yaw
     * compensated body frame. 
     */
    void generateCandidatePoints( const asguard::BodyState& state, const base::Quaterniond& orientation )
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

    /** 
     * Will set the optional terrain classification information, which may be
     * used additional to the contact point information.
     */
    void setTerrainClassification( const std::vector<terrain_estimator::TerrainClassification>& ltc )
    {
	terrain_classification = ltc;
    }

    const std::vector<vec3array>& getCandidatePoints() const { return candidate_group; }

    /** needs to have a prior call generateCandidatePoints. Those candidate
     * points will be evaluated for the given pose and variances on the map
     * callback.
     *
     * The signature of the map callback needs to be
     *
     * bool map( base::Vector3d const& p, double& zpos, double& zvar ),
     *
     * where p is the 3d map point to be evaluated, zpos and zvar are the return
     * values from the map, with zpos the actual z position of the map at that
     * point and zvar it's variance. map needs to return true if a map cell was
     * found and false otherwise.
     *
     * @param pose - yaw compensated body to world pose of the robot
     * @param measVar - measurement variance of the contact model alogn z-axis 
     * @param map - map callback
     *
     * @result true if any contact points have been found.
     */
    bool evaluatePose( const base::Affine3d& pose, double measVar, boost::function<bool (const base::Vector3d&, envire::MLSGrid::SurfacePatch&)> map )
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
				// since currently the map patches provide a color
				// information we also convert the
				// terrain_classification information into a color
				// information based on the terrain classes.
				// TODO this is a hack and should be handled in a more generic way
				//
				base::Vector3d prop_terrain = base::Vector3d::Zero();

				for( std::vector<terrain_estimator::TerrainProbability>::iterator it = terrain_classification[i].terrain.begin();
					it != terrain_classification[i].terrain.end(); it++)
				{
				    terrain_estimator::TerrainProbability &tp( *it );
				    switch( tp.type )
				    {
					case terrain_estimator::GRASS: prop_terrain[0] = tp.probability; break;
					case terrain_estimator::PATH: prop_terrain[1] = tp.probability; break;
					case terrain_estimator::PEBBLES: prop_terrain[2] = tp.probability; break;
					default: break;
				    }
				}

				// get the visual classification from the patch
				base::Vector3d &visual_terrain( patch.color );

				// for now use the crudest way possible to get the probability
				// for the measurement
				double prob = 1.0 - (visual_terrain - prop_terrain).norm() / sqrt( 3.0 );

				// store this information with the contact point
				p.prob *= prob;
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

    virtual void evaluateWeight( double measVar )
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
	    pz *= zk;
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

    /** relative weight of the last evaluated pose
     */
    double getWeight() const
    {
	return m_weight;
    }

    /** height difference of the last evaluated pose compared to the map.
     */
    double getZDelta() const
    {
	return m_zDelta;
    }

    /** variance in height with respect to the map for the last evaluated pose.
     */
    double getZVar() const
    {
	return m_zVar;
    }

    /** return a reference to the vector of contact points, which store the
     * contact points of the system and the found z values of the map.
     */
    std::vector<ContactPoint>& getContactPoints()
    {
	return contact_points;
    }
};

/** 
 * Contact model base on: 
 * Chitta S, Vernaza P, Geykhman R, Lee DD. Proprioceptive localization for a
 * quadrupedal robot on known terrain. In: The IEEE International Conference
 * on Robotics and Automation.; 2007. 
 */
class ChittaContactModel : public ContactModel
{
public:
    ChittaContactModel( const asguard::Configuration& asguardConfig ) 
	:  ContactModel( asguardConfig )
    {
    }

    virtual void evaluateWeight( double measVar )
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
};

}

#endif
