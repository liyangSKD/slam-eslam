#ifndef __ESLAM_CONTACTMODEL_HPP__
#define __ESLAM_CONTACTMODEL_HPP__

#include <boost/function.hpp>
#include <base/Eigen.hpp>
#include <eslam/PoseParticle.hpp>
#include <terrain_estimator/TerrainConfiguration.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <odometry/ContactState.hpp>
#include <eslam/Configuration.hpp>

namespace eslam
{

/** 
 * Contactmodel class that relates the kinematic configuration of a robot with
 * an environment model.  
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
    odometry::BodyContactState contactState;
    std::vector<base::Vector3d> lowestPointsPerGroup;
    std::vector<ContactPoint> contact_points;
    std::vector<SlipPoint> slip_points;

    std::vector<terrain_estimator::TerrainClassification> terrain_classification;

    double m_zDelta;
    double m_zVar;
    double m_weight;
    double m_poseVar;

    ContactModelConfiguration config;

protected:
    double matchTerrain( const Eigen::Vector3d& color, size_t group_id, const Eigen::Vector3d& position );
    void lowestPointHeuristic(bool update_probabilities);
    double contactLikelihoodRatio( double z, double sigma );

public:
    static const int GROUP_SIZE = 4;

    /**
     * set configuration parameters
     */
    void setConfiguration( const ContactModelConfiguration& config );

    /** @brief default constructor
     */
    ContactModel(); 

    /** given a @param state configuration state of the system and an @param
     * orientation, candidate contact points are calculated and stored in the yaw
     * compensated body frame. 
     */
    //void generateCandidatePoints( const BodyContactState& state, const base::Quaterniond& orientation );
    void setContactPoints( const odometry::BodyContactState& state, const base::Quaterniond& orientation );

    /** Update the internal contact probabilities using the lowest-point
     * heuristic
     */
    void updateContactStateUsingLowestPointHeuristic();

    /** Returns the contact state as stored internally by the contact model.
     */
    odometry::BodyContactState const& getContactState() const;

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
     * bool map( base::Vector3d const& p, SurfacePatch& patch )
     *
     * where p [in] is the 3d map point to be evaluated, patch is the surface
     * patch of the MLSGrid [out]. map needs to return true if a map cell was
     * found and false otherwise.
     *
     * @param pose - position and heading of the robot, composed in a pose
     * @param measVar - measurement variance of the contact model alogn z-axis 
     * @param map - map callback
     *
     * @result true if any contact points have been found.
     */
    bool evaluatePose( 
	    const base::Affine3d& pos_and_heading, 
	    double measVar, 
	    boost::function<bool (const base::Vector3d&, envire::MLSGrid::SurfacePatch&)> map );

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

    /** 
     * update the given z-position estimate (mean,var) based on 
     * the last pose evaluation.
     *
     * @param zPos [in,out] position in z of the body
     * @param zVar [in,out] variance around z
     * @result true if position was updated, false if measurement was rejected
     */
    bool updateZPositionEstimate( double& zPos, double& zVar );

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
    ChittaContactModel();

    virtual void evaluateWeight( double measVar );
};

}

#endif
