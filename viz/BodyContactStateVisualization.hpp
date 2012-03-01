#ifndef ESLAM_BODYCONTACTSTATEVISUALIZATION_H
#define ESLAM_BODYCONTACTSTATEVISUALIZATION_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <eslam/ContactState.hpp>

namespace vizkit
{
    class BodyContactStateVisualization
        : public vizkit::Vizkit3DPlugin<eslam::BodyContactState>
        , boost::noncopyable
    {
    Q_OBJECT

    public:
        BodyContactStateVisualization();
        ~BodyContactStateVisualization();

	Q_INVOKABLE void updateContactState( const eslam::BodyContactState& state );

    protected:
	virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(eslam::BodyContactState const& plan);
        
    private:
	eslam::BodyContactState state;
    };
}
#endif
