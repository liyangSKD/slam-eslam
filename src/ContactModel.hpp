#ifndef __ESLAM_CONTACTMODEL_HPP__
#define __ESLAM_CONTACTMODEL_HPP__

#include <boost/function.hpp>
#include <base/eigen.h>
#include <eslam/PoseParticle.hpp>
#include <terrain_estimator/TerrainConfiguration.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <asguard/Configuration.hpp>

namespace eslam
{

struct BodyContact
{
    /**
     * candidate groups are used for contact point estimation.  Each of the
     * groups represent a number of potential contact points. Which of these
     * points have contact can be resolved using a terrain model.  All points
     * are provided in body frame coordinates.
     */
    std::vector<std::vector<base::Vector3d> > candidate_groups;

    /** 
     * contact_points represent actual points of contact with the environment.
     * All points are provided in body frame coordinates.
     */
    std::vector<ContactPoint> contact_points;
};

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
    std::vector<SlipPoint> slip_points;

    asguard::Configuration asguardConfig;

    std::vector<terrain_estimator::TerrainClassification> terrain_classification;

    double m_zDelta;
    double m_zVar;
    double m_weight;

    bool m_useShapeUpdate;
    bool m_useTerrainUpdate;

public:
    static const int GROUP_SIZE = 4;

    void useShapeUpdate( bool use )
    {
	m_useShapeUpdate = use;
    }

    void useTerrainUpdate( bool use )
    {
	m_useTerrainUpdate = use;
    }	

    /** Constructor that takes a @param asguardConfig configuration model as the
     * basis.
     */
    ContactModel( const asguard::Configuration& asguardConfig );

    /** given a @param state configuration state of the system and an @param
     * orientation, candidate contact points are calculated in the yaw
     * compensated body frame. 
     */
    void generateCandidatePoints( const asguard::BodyState& state, const base::Quaterniond& orientation );

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
    bool evaluatePose( const base::Affine3d& pose, double measVar, boost::function<bool (const base::Vector3d&, envire::MLSGrid::SurfacePatch&)> map );

    virtual void evaluateWeight( double measVar );

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

    std::vector<SlipPoint>& getSlipPoints()
    {
	return slip_points;
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
    ChittaContactModel( const asguard::Configuration& asguardConfig );

    virtual void evaluateWeight( double measVar );
};

}

#endif
