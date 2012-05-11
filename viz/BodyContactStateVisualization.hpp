#ifndef ESLAM_BODYCONTACTSTATEVISUALIZATION_H
#define ESLAM_BODYCONTACTSTATEVISUALIZATION_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <odometry/ContactState.hpp>

namespace vizkit
{
    class BodyContactStateVisualization
        : public vizkit::Vizkit3DPlugin<odometry::BodyContactState>
        , boost::noncopyable
    {
    Q_OBJECT

    public:
        BodyContactStateVisualization();
        ~BodyContactStateVisualization();

	Q_INVOKABLE void updateContactState( const odometry::BodyContactState& state );

    protected:
	virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(odometry::BodyContactState const& plan);
        
    private:
	odometry::BodyContactState state;
    };
}
#endif
