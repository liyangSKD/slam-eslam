#ifndef __VIZKIT_ESLAMWIDGET__
#define __VIZKIT_ESLAMWIDGET__

#include <Eigen/Geometry>
#include <asguard/BodyState.hpp>
#include <envire/Core.hpp>
#include <base/pose.h>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/AsguardVisualization.hpp>

namespace eslam
{
    class PoseDistribution;
}

namespace vizkit
{
class EnvireVisualization;
class AsguardVisualization;
class ParticleVisualization;
class TrajectoryVisualization;

class EslamWidget : public QVizkitWidget
{
public:
    EslamWidget( QWidget* parent = 0, Qt::WindowFlags f = 0 );
    ~EslamWidget();
    void setPoseDistribution( const eslam::PoseDistribution& dist );
    void setBodyState( const asguard::BodyState& body_state ); 
    void setReferencePose( const base::Pose& pose );
    void setCentroidPose( const base::Pose& pose );
    void setEnvironment( envire::Environment *env );

    int getInspectedParticleIndex() const;
    void setInspectedParticleIndex( int index );

    void setDirty(); 
    bool isDirty() const;

private:
    boost::shared_ptr<EnvireVisualization> envViz;
    boost::shared_ptr<AsguardVisualization> robotViz;
    boost::shared_ptr<ParticleVisualization> particleViz;

    boost::shared_ptr<TrajectoryVisualization> referenceViz;
    boost::shared_ptr<TrajectoryVisualization> centroidViz;

    vizkit::AsguardState asguardState;
};

}

#endif
