#ifndef __VIZKIT_ESLAMWIDGET__
#define __VIZKIT_ESLAMWIDGET__

#include <Eigen/Geometry>
#include <asguard/BodyState.hpp>
#include <envire/Core.hpp>
#include <base/pose.h>

#include <vizkit/QVizkitWidget.hpp>

namespace eslam
{
    class PoseDistribution;
}

namespace vizkit
{
class EnvireVisualization;
class AsguardVisualization;
class ParticleVisualization;

class EslamWidget : public QVizkitWidget
{
public:
    EslamWidget( QWidget* parent = 0, Qt::WindowFlags f = 0 );
    ~EslamWidget();
    void setPoseDistribution( const eslam::PoseDistribution& dist );
    void setReferencePose( const base::Pose& pose, const asguard::BodyState& body_state );
    void setEnvironment( envire::Environment *env );

private:
    osg::ref_ptr<EnvireVisualization> envViz;
    osg::ref_ptr<AsguardVisualization> robotViz;
    osg::ref_ptr<ParticleVisualization> particleViz;
};

}

#endif
