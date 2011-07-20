#ifndef __ESLAM_CONTACTMODEL_HPP__
#define __ESLAM_CONTACTMODEL_HPP__

#include <boost/function.hpp>
#include <base/eigen.h>
#include <eslam/PoseParticle.hpp>

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
    typedef std::vector<base::Vector3d> vec3array;
    std::vector<vec3array> candidate_group;

    std::vector<ContactPoint> contact_points;

    asguard::Configuration asguardConfig;

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
    bool evaluatePose( const base::Affine3d& pose, double measVar, boost::function<bool (base::Vector3d const&, double&, double&)> map )
    {
	contact_points.clear();

	for( std::vector<vec3array>::iterator gi = candidate_group.begin(); gi != candidate_group.end(); gi++ )
	{
	    // find the contact points with the lowest zdiff per wheel
	    ContactPoint p;
	    bool contact = true;
	    for( std::vector<base::Vector3d>::iterator it=gi->begin(); it!=gi->end(); it++ )
	    {
		const base::Vector3d &cpoint(*it);

		base::Vector3d gp = pose * cpoint;
		double zPos, zVar = measVar;
		if( !map( gp, zPos, zVar ) )
		{
		    contact = false;
		    break;
		}

		const double zdiff = gp.z() - zPos;

		if( zdiff < p.zdiff )
		{
		    const double zvar = zVar + measVar;
		    p = ContactPoint( gp-base::Vector3d::UnitZ()*zdiff, zdiff, zvar );
		}
	    }

	    if( contact )
	    {
		contact_points.push_back( p );
	    }
	}

	if( contact_points.size() > 0 ) 
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
	    }

	    const double zd = delta/sqrt( measVar );
	    pz *= exp( -(zd*zd)/2.0 );
	    //pz = 1.0;

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

	    return true;
	}

	return false;
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

}

#endif
