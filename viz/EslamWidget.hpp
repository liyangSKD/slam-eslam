#ifndef __VIZKIT_ESLAMWIDGET__
#define __VIZKIT_ESLAMWIDGET__

#include <Eigen/Geometry>
#include <odometry/ContactState.hpp>
#include <eslam/PoseEstimator.hpp>
#include <envire/Core.hpp>
#include <base/Pose.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>

namespace eslam
{
    class PoseDistribution;
}

namespace envire
{
    class MLSMap;
    class EnvireVisualization;
}

namespace vizkit3d
{
class ParticleVisualization;
class TrajectoryVisualization;
class MapVizEventFilter;

class EslamWidget : public Vizkit3DWidget
{
    Q_OBJECT 

public:
    EslamWidget( QWidget* parent = 0, Qt::WindowFlags f = 0 );
    ~EslamWidget();

    void setPoseDistribution( const eslam::PoseDistribution& dist );
    void setBodyState( const odometry::BodyContactState& body_state ); 
    void setReferencePose( const base::Pose& pose );
    void setCentroidPose( const base::Pose& pose );
    void setEnvironment( envire::Environment *env );
    void setPoseParticles( std::vector<eslam::PoseParticleGA> *pe );
    void setRobotViz(const boost::shared_ptr< VizPluginAdapter<base::samples::RigidBodyState> >& _robotViz);

    int getInspectedParticleIndex() const;
    void setInspectedParticleIndex( int index );

    void setDirty(); 
    bool isDirty() const;

public slots:
    void viewMap( int particle_idx );

private:
    boost::shared_ptr<envire::EnvireVisualization> envViz;
    boost::shared_ptr< vizkit3d::VizPluginAdapter<base::samples::RigidBodyState> > robotViz;
    boost::shared_ptr<ParticleVisualization> particleViz;
    boost::shared_ptr<TrajectoryVisualization> referenceViz;
    boost::shared_ptr<TrajectoryVisualization> centroidViz;

    boost::shared_ptr<MapVizEventFilter> filter;

    std::vector<eslam::PoseParticleGA> *pe;
};

}

#endif
