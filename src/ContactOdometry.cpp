#include "ContactOdometry.hpp"

using namespace odometry;

FootContact::FootContact(const asguard::odometry::Configuration& config)
    : config( config ), sampling(config)
{
}

/** this function will return the transformation from the frame that is spanned
 * up the points p1, p2 and p3 to the global frame.  p1 is the origin of the
 * frame and p1-p2 the x axis.  further, p3 will be on the plane spanned up the
 * the x and y axis of the frame.
 */
/*
bool FootContact::getTransformFromPoints( const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, Eigen::Affine3d &trans )
{
    trans = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot;
    const Eigen::Vector3d c1(p2-p1), c2(p3-p1);

    rot.col(0) = c1.normalized();
    rot.col(2) = c2.cross( c1 ).normalized();
    if( rot.col(2).norm() < 1e-5 )
	return false; // the rotation is underspecified
    else
	rot.col(1) = c1.cross(rot.col(2)).normalized(); 

    trans.matrix().topLeftCorner<3,3>() = rot;
    trans.pretranslate( p1 );
    return true;
}
*/

Eigen::Matrix3d FootContact::getPositionError()
{
    return Eigen::Matrix3d(sampling.poseCov.bottomRightCorner<3,3>());
}

Eigen::Matrix3d FootContact::getOrientationError()
{
    return Eigen::Matrix3d(sampling.poseCov.topLeftCorner<3,3>());
}

Matrix6d FootContact::getPoseError()
{
    return sampling.poseCov;
}

base::Pose FootContact::getPoseDeltaSample()
{
    return asguard::odometry::getPoseFromVector6d( sampling.sample() );
}

base::Pose2D FootContact::getPoseDeltaSample2D()
{
    return asguard::odometry::projectPoseDelta( orientation, getPoseDeltaSample() );
}

base::Pose FootContact::getPoseDelta()
{
    return asguard::odometry::getPoseFromVector6d( sampling.poseMean );
}

void FootContact::update(const eslam::BodyContactState& state, const Eigen::Quaterniond& orientation)
{

    /*
    // update state
    state.update(bs);
    this->orientation = orientation;

    std::vector<Eigen::Vector3d> p_k, p_kp;

    // go through all feets and check if they have maintained contact
    for(int ii=0;ii<4;ii++)
    {
	wheelIdx i = static_cast<wheelIdx>(ii);

	for(int j=0;j<5;j++)
	{
	    WheelContact wc_ki = state.getPrevious().getWheelContact(i, j);
	    WheelContact wc_kpi = state.getCurrent().getWheelContact(i, j);

	
	    if( wc_ki.contact > 0.5 && wc_kpi.contact > 0.5 ) 
	    {
		p_k.push_back( asguardConfig.getFootPosition( state.getPrevious(), i, j ) );
		p_kp.push_back( asguardConfig.getFootPosition( state.getCurrent(), i, j ) );
	    }
	}
    }	

    // p_k and p_kp contain foot positions that have maintained contact
    // and can be used for calculation of the pose change

    // the approach is to take all subsets of the contact pairs with 3 
    // members and align the the pose using the following method
    // - align the first point pair (translate)
    // - align the second point pair (rotate around first pair)
    // - align the third pair (rotate around line between first and second pair)
    //
    const size_t num_contacts = p_k.size();
    size_t num_samples = 0; 

    Matrix6d Sxx( Matrix6d::Zero() );
    Vector6d Sx( Vector6d::Zero() );

    for(size_t i=0;i<num_contacts;i++)
    {
	for(size_t j=0;j<num_contacts;j++)
	{
	    if( i == j )
		continue;

	    for(size_t k=0;k<num_contacts;k++)
	    {
		if( k == i || k == j )
		    continue;

		// the transformations are from the common contact frame into the
		// respective body frame (current bkp and previous bk)
		Eigen::Affine3d C_c2bk, C_c2bkp;
		
		if( getTransformFromPoints( p_k[i], p_k[j], p_k[k], C_c2bk )
			&& getTransformFromPoints( p_kp[i], p_kp[j], p_kp[k], C_c2bkp ) )
		{
		    num_samples++;

		    // get the relative transform between the frames
		    Eigen::Affine3d C_bkp2bk( C_c2bk * C_c2bkp.inverse() );

		    // construct a sample vector which is the rotation part in
		    // scaled axis and the translation part in local frame
		    Eigen::AngleAxisd aa( C_bkp2bk.rotation() );

		    Vector6d sample;
		    sample.head<3>() = aa.axis() * aa.angle();
		    sample.tail<3>() = C_bkp2bk.translation();

		    Sxx += sample * sample.transpose();
		    Sx += sample;
		}
	    }
	}
    }

    if( num_samples > 0 )
    {
	const double num_inv = 1.0/num_samples;
	Vector6d mean = Sx*num_inv;
	Matrix6d cov = Sxx*num_inv - mean*( mean.transpose());

	// TODO make sure there are no zeros here
	Matrix6d llt = cov.llt().matrixL();
	sampling.update( mean, llt );
    }
    else
    {
	// use fixed error model here
	sampling.update( Vector6d::Zero(), fixedError );
    }
    */
}
