#ifndef PARTICLEVIZUALISATION_H
#define PARTICLEVIZUALISATION_H
#include <Eigen/Geometry>
#include <vector>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geometry>

#include <base/pose.h>
#include <eslam/PoseParticle.hpp>
#include <osg/Geode>

namespace vizkit 
{

class ParticleVisualization : public VizPluginAdapter<eslam::PoseDistribution>
{
	Q_OBJECT

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ParticleVisualization();

	void setVisualiseContacts(bool viscontacts);
	int getInspectedParticle() const; 

	Q_INVOKABLE void updatePoseDistribution( const eslam::PoseDistribution& data );
	Q_PROPERTY( bool show_gmm READ getShowGMM WRITE setShowGMM )
    
    public slots:
	bool getShowGMM() const { return show_gmm; }
	void setShowGMM( bool show ) { show_gmm = show; emit propertyChanged("show_gmm"); }
	void inspectParticle(int index);

    signals:
	void inspectParticleUpdate(int index);
	    
    private:
	bool show_gmm;
	bool viscontacts;

	virtual void updateDataIntern ( const eslam::PoseDistribution& data );
	virtual void operatorIntern ( osg::Node* node, osg::NodeVisitor* nv );

	eslam::PoseDistribution dist;

	osg::ref_ptr<osg::Geode> particleGeode;
	osg::ref_ptr<osg::Node> offsetNode;

	int inspectIdx;
};

}
#endif 
