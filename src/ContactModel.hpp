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

struct BodyContactPoint
{
    /** @brief position in body frame */
    base::Vector3d position;

    /** @brief contact probability in the interval between 0 and 1.0, or NaN if unknown */
    double contact;

    /** @brief slip distance of contact point */
    double slip;

    /** @brief contact group id, or -1 if not part of a group */
    int groupId;
};

struct BodyContactState
{
    /** @brief timestamp */
    base::Time time;
    
    /** @brief vector of all potential contact points of the body and their states.
     * 
     * The order of the contact points in the vector should not change over data items.
     */
    std::vector<BodyContactPoint> points;
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
    BodyContactState contactState;
    std::vector<base::Vector3d> lowestPointsPerGroup;
    std::vector<ContactPoint> contact_points;

    /*
    typedef std::vector<base::Vector3d> vec3array;
    std::vector<vec3array> candidate_group;
    asguard::Configuration asguardConfig;
    std::vector<BodyContactPoint> contactPoints;
    */

    std::vector<terrain_estimator::TerrainClassification> terrain_classification;

    double m_zDelta;
    double m_zVar;
    double m_weight;

public:
    static const int GROUP_SIZE = 4;

    /** Constructor that takes a @param asguardConfig configuration model as the
     * basis.
     */
    ContactModel(); 

    /** given a @param state configuration state of the system and an @param
     * orientation, candidate contact points are calculated in the yaw
     * compensated body frame. 
     */
    //void generateCandidatePoints( const BodyContactState& state, const base::Quaterniond& orientation );
    void setContactPoints( const BodyContactState& state, const base::Quaterniond& orientation );

    /** 
     * Will set the optional terrain classification information, which may be
     * used additional to the contact point information.
     */
    void setTerrainClassification( const std::vector<terrain_estimator::TerrainClassification>& ltc )
    {
	terrain_classification = ltc;
    }

    /**
     * will for each group return the candidate contact point with the lowest z
     * value or all candidate points if there are no groups.
     */
    const std::vector<base::Vector3d>& getLowestPointPerGroup();

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
    ChittaContactModel();

    virtual void evaluateWeight( double measVar );
};

}

#endif
