#include "BodyContactStateVisualization.hpp"

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Shape>

using namespace vizkit;
using namespace odometry;

BodyContactStateVisualization::BodyContactStateVisualization()
{
}

BodyContactStateVisualization::~BodyContactStateVisualization()
{
}

void BodyContactStateVisualization::updateContactState( const odometry::BodyContactState& state )
{
    updateData( state );
}

osg::ref_ptr<osg::Node> BodyContactStateVisualization::createMainNode()
{
    return new osg::Group();
}

void BodyContactStateVisualization::updateMainNode(osg::Node* node)
{
    // clear node first
    node->asGroup()->removeChildren(0, node->asGroup()->getNumChildren());

    for(size_t j=0;j<state.points.size();j++)
    {
	const odometry::BodyContactPoint &cp(state.points[j]);
	const double radius = 0.01;
	const double visible = cp.contact;
	const osg::Vec3 pos( cp.position.x(), cp.position.y(), cp.position.z() ); 

	osg::Geode* geode = new osg::Geode;

	osg::Sphere* sphereShape = new osg::Sphere( pos, radius );
	osg::ShapeDrawable *sphere = new osg::ShapeDrawable( sphereShape ); 

	sphere->setColor( osg::Vec4(0.0f, 1.0f, 1.0f, visible) );
	geode->addDrawable( sphere );

	node->asGroup()->addChild( geode );
    }
}

void BodyContactStateVisualization::updateDataIntern(odometry::BodyContactState const& state)
{
    this->state = state;
}

//VizkitQtPlugin( BodyContactStateVisualization )

