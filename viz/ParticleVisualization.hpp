#ifndef PARTICLEVIZUALISATION_H
#define PARTICLEVIZUALISATION_H
#include <Eigen/Geometry>
#include <vector>
#include <enview/DataNode.h>
#include <osg/Geometry>

#include <base/pose.h>
#include <enview/AsguardModel.hpp>
#include <eslam/PoseParticle.hpp>

namespace enview {

class ParticleVisualization: public DataNode<eslam::PoseDistribution>
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ParticleVisualization();

	void setVisualiseContacts(bool viscontacts);
	void inspectParticle(size_t index);
	    
    private:
	bool viscontacts;

	virtual void updateDataIntern ( const eslam::PoseDistribution& data );
	virtual void operatorIntern ( osg::Node* node, osg::NodeVisitor* nv );

	eslam::PoseDistribution dist;

	osg::ref_ptr<osg::Geode> particleGeode;
	osg::ref_ptr<osg::Node> offsetNode;

	int inspectIdx;

	osg::ref_ptr<enview::AsguardModel> asguard;
};

}
#endif 
