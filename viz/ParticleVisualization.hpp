#ifndef PARTICLEVIZUALISATION_H
#define PARTICLEVIZUALISATION_H
#include <Eigen/Geometry>
#include <vector>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geometry>

#include <base/pose.h>
#include <vizkit/AsguardModel.hpp>
#include <eslam/PoseParticle.hpp>

namespace vizkit 
{

class ParticleVisualization : public VizPluginAdapter<eslam::PoseDistribution>
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ParticleVisualization();

	void setVisualiseContacts(bool viscontacts);
	void inspectParticle(int index);
	int getInspectedParticle() const { return inspectIdx; }
	    
    private:
	bool viscontacts;

	virtual void updateDataIntern ( const eslam::PoseDistribution& data );
	virtual void operatorIntern ( osg::Node* node, osg::NodeVisitor* nv );

	eslam::PoseDistribution dist;

	osg::ref_ptr<osg::Geode> particleGeode;
	osg::ref_ptr<osg::Node> offsetNode;

	int inspectIdx;

	osg::ref_ptr<vizkit::AsguardModel> asguard;
};

}
#endif 
