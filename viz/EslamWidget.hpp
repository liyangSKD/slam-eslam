#ifndef __VIZKIT_ESLAMWIDGET__
#define __VIZKIT_ESLAMWIDGET__

#include <Eigen/Geometry>
#include <eslam/ContactState.hpp>
#include <envire/Core.hpp>
#include <base/pose.h>

#include <vizkit/Vizkit3DWidget.hpp>
#include <vizkit/AsguardVisualization.hpp>

namespace eslam
{
    class PoseDistribution;
}

namespace envire
{
    class MLSMap;
    class EnvireVisualization;
}

namespace vizkit
{
class AsguardVisualization;
class ParticleVisualization;
class TrajectoryVisualization;
class MapVizEventFilter;

class EslamWidget : public Vizkit3DWidget
{
public:
    EslamWidget( QWidget* parent = 0, Qt::WindowFlags f = 0 );
    ~EslamWidget();
    void setPoseDistribution( const eslam::PoseDistribution& dist );
    void setBodyState( const eslam::BodyContactState& body_state ); 
    void setReferencePose( const base::Pose& pose );
    void setCentroidPose( const base::Pose& pose );
    void setEnvironment( envire::Environment *env );

    int getInspectedParticleIndex() const;
    void setInspectedParticleIndex( int index );

    void viewMap( envire::MLSMap* map );

    void setDirty(); 
    bool isDirty() const;

private:
    boost::shared_ptr<envire::EnvireVisualization> envViz;
    boost::shared_ptr<AsguardVisualization> robotViz;
    boost::shared_ptr<ParticleVisualization> particleViz;

    boost::shared_ptr<TrajectoryVisualization> referenceViz;
    boost::shared_ptr<TrajectoryVisualization> centroidViz;

    boost::shared_ptr<MapVizEventFilter> filter;

};

}

#endif
