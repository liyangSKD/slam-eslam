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

    ContactModel( const asguard::Configuration& asguardConfig ) 
	: candidate_group( GROUP_SIZE ),
	  asguardConfig( asguardConfig )
    {
    }

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

    bool evaluatePose( const base::Affine3d& pose, double zVarPose, double measVar, boost::function<bool (base::Vector3d const&, double&, double&)> map )
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
		double zPos, zVar = zVarPose + measVar;
		if( !map( gp, zPos, zVar ) )
		{
		    contact = false;
		    break;
		}

		const double zdiff = gp.z() - zPos;

		if( zdiff < p.zdiff )
		{
		    const double zvar = zVar + zVarPose + measVar;
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

	    const double zd = delta/sqrt( zVarPose + measVar );
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
	    pose.zPos += -delta;
	    pose.zSigma = 1.0/sqrt(d2);

	    // use some measurement of the variance as the weight 
	    xi_k[i].weight *= pz;
	    xi_k[i].mprob = pz;
	    xi_k[i].floating = false;

	    // store the current maximum weight
	    max_weight = std::max( max_weight, pz );
	    
	    data_particles ++;
	    sum_data_weights += pow(pz,1/found_points);
	    */

	    return true;
	}

	return false;
    }

    double getWeight() const
    {
	return m_weight;
    }

    double getZDelta() const
    {
	return m_zDelta;
    }

    double getZVar() const
    {
	return m_zVar;
    }

    std::vector<ContactPoint>& getContactPoints()
    {
	return contact_points;
    }
};

}

#endif
