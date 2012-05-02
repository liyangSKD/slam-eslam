#include "ParticleVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/LineWidth>
#include <algorithm>

#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Material>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <vizkit/PickHandler.hpp>
#include <Eigen/LU>

#include <vizkit/Uncertainty.hpp>

namespace vizkit 
{

class ParticlePickedCallback : public PickedCallback
{
    boost::function<void ()> f; 

public:
    ParticlePickedCallback( boost::function<void ()> callback )
	: f( callback ) {}

    virtual void picked() 
    {
	f();
    }
};


osg::Geode* getParticleGeode( const osg::Vec4& color )
{
    const double diam = 0.01;

    osg::Geode* circle = new osg::Geode;
    osg::Cylinder* cylinderShape = new osg::Cylinder( osg::Vec3(0,0,.25), diam, .5);
    osg::ShapeDrawable *cylinder = new osg::ShapeDrawable(cylinderShape);
    osg::Box* boxShape = new osg::Box( osg::Vec3(0, diam,.25), diam/4.0, diam, .5);
    osg::ShapeDrawable *box = new osg::ShapeDrawable(boxShape);
    cylinder->setColor( color );
    box->setColor( color );
    circle->addDrawable( cylinder );
    circle->addDrawable( box );

    return circle;
};

void ParticleVisualization::inspectParticle( int index )
{
    inspectIdx = index;
    setDirty();

    emit inspectParticleUpdate( index );
}

int ParticleVisualization::getInspectedParticle() const
{
    return inspectIdx;
}

ParticleVisualization::ParticleVisualization()
    : show_gmm(false), viscontacts(false), inspectIdx(-1)
{
    //particleGeode = getParticleGeode();

    ownNode = new osg::Group();
    setMainNode( ownNode );

    osg::PositionAttitudeTransform* tn =
	new osg::PositionAttitudeTransform();
    ownNode->asGroup()->addChild( tn );

    offsetNode = new osg::Group();
    tn->addChild( offsetNode );
    
    asguard = new vizkit::AsguardModel();
}

void ParticleVisualization::operatorIntern ( osg::Node* node, osg::NodeVisitor* nv )
{   
    offsetNode->asGroup()->removeChildren(0, offsetNode->asGroup()->getNumChildren());

    /*
       if( showCentroid )
       {
       osg::PositionAttitudeTransform* tn =
       new osg::PositionAttitudeTransform();
       ownNode->asGroup()->addChild( tn );
       }
       */

    if( show_gmm )
    {
	// take the gaussian mixture model and display the uncertainty ellipses
	eslam::PoseDistribution::GMM &gmm( dist.gmm );
	for( size_t i=0; i<gmm.params.size(); i++ )
	{
	    vizkit::Uncertainty *u = new vizkit::Uncertainty();
	    u->setMean( static_cast<Eigen::Vector2d>( gmm.params[i].dist.mean ) );
	    u->setCovariance( static_cast<Eigen::Matrix2d>( gmm.params[i].dist.cov ) );
	    offsetNode->asGroup()->addChild( u );
	}
    }

    std::vector<eslam::PoseParticle> &v(dist.particles);

    for(size_t i=0;i<v.size();i++)
    {
	osg::PositionAttitudeTransform* tn =
	    new osg::PositionAttitudeTransform();

	const double trans = 1.0;
	//const double fade = v[i].w * v.size();

	osg::Vec4 color = v[i].floating ?
	    osg::Vec4( .9f, .3f, .3f, trans ) :
	    osg::Vec4( .8f, .8f, .8f, trans );

	if( inspectIdx >= 0 && static_cast<size_t>(inspectIdx) == i )
	    color = osg::Vec4( .3f, .8f, .9f, trans );

	const double height = 0.2*v[i].weight*v.size();

	tn->setPosition(osg::Vec3( v[i].position.x(), v[i].position.y(), v[i].zPos ));
	tn->setScale(osg::Vec3(1.0,1.0,height));
	osg::Quat q;
	q.makeRotate(v[i].orientation,0,0,1.0);
	tn->setAttitude(q);
	osg::Node *pnode = getParticleGeode( color );
	pnode->setUserData( 
		new ParticlePickedCallback( 
		    boost::bind( &ParticleVisualization::inspectParticle, this, i ) ) );
	tn->addChild( pnode );
	offsetNode->asGroup()->addChild( tn );
    }

    if( inspectIdx >= 0 )
    {
	const size_t i = inspectIdx;
	eslam::PoseParticle &pose( v[i] );

	Eigen::Affine3d t = 
	    Eigen::Translation3d( pose.meas_pos ) 
	    * Eigen::AngleAxisd( pose.meas_theta, Eigen::Vector3d::UnitZ() );

	Eigen::Affine3d transform( t * base::removeYaw( dist.orientation ) );

	// add asguard robot
	// TODO reinstantiate visualisation of robot
	/*
	asguard::BodyState &bodyState( dist.bodyState );
	asguard->setBodyState( bodyState );
	osg::PositionAttitudeTransform *asguardPose = new osg::PositionAttitudeTransform();
	Eigen::Vector3d apos( transform.translation() );
	Eigen::Quaterniond arot( transform.rotation() );

	asguardPose->setPosition( osg::Vec3( apos.x(), apos.y(), apos.z() ) );
	asguardPose->setAttitude( osg::Quat( arot.x(), arot.y(), arot.z(), arot.w() ) );

	offsetNode->asGroup()->addChild( asguardPose );
	asguardPose->addChild( asguard );
	*/

	for(size_t j=0;j<pose.cpoints.size();j++)
	{
	    const eslam::ContactPoint &cp(pose.cpoints[j]);
	    const double radius = 0.01;
	    const double height = cp.zdiff;
	    const osg::Vec3 pos( cp.point.x(), cp.point.y(), cp.point.z() ); 

	    osg::Geode* geode = new osg::Geode;

	    osg::Cylinder* cylinderShape = new osg::Cylinder( pos+osg::Vec3(0,0,height*.5), radius*.5, height );
	    osg::ShapeDrawable *cylinder = new osg::ShapeDrawable(cylinderShape);

	    osg::Sphere* sphereShape = new osg::Sphere( pos, radius );
	    osg::ShapeDrawable *sphere = new osg::ShapeDrawable( sphereShape ); 

	    cylinder->setColor( osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f) );
	    sphere->setColor( osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f) );
	    geode->addDrawable( cylinder );
	    geode->addDrawable( sphere );

	    offsetNode->asGroup()->addChild( geode );

	}

	for(size_t j=0;j<pose.spoints.size();j++)
	{
	    const eslam::SlipPoint &cp(pose.spoints[j]);
	    const double radius = 0.05;
	    const double height = cp.prob;
	    const osg::Vec3 pos( cp.position.x(), cp.position.y(), cp.position.z() ); 

	    osg::Geode* geode = new osg::Geode;

	    osg::Sphere* sphereShape = new osg::Sphere( pos, radius );
	    osg::ShapeDrawable *sphere = new osg::ShapeDrawable( sphereShape ); 

	    sphere->setColor( osg::Vec4(cp.color.x(), cp.color.y(), cp.color.z(), 1.0 ) );
	    geode->addDrawable( sphere );

	    osg::Cylinder* cylinderShape = new osg::Cylinder( pos+osg::Vec3(0,0,height*.5), radius*.5, height );
	    osg::ShapeDrawable *cylinder = new osg::ShapeDrawable(cylinderShape);

	    cylinder->setColor( osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f) );
	    geode->addDrawable( cylinder );

	    offsetNode->asGroup()->addChild( geode );
	}
    }
}

void ParticleVisualization::updateDataIntern ( const eslam::PoseDistribution& dist )
{
    this->dist = dist;
    setDirty();
}

void ParticleVisualization::updatePoseDistribution( const eslam::PoseDistribution& data )
{
    updateData( data );
}

//VizkitQtPlugin( ParticleVisualization )

}

