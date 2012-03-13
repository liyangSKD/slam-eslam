#include "EslamWidget.hpp"
#include "ParticleVisualization.hpp"

#include <eslam/PoseParticle.hpp>

#include <vizkit/AsguardVisualization.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <vizkit/TrajectoryVisualization.hpp>

#include <envire/maps/MLSMap.hpp>

#include "MapVizEventFilter.hpp"

using namespace vizkit;
using namespace envire;

EslamWidget::EslamWidget( QWidget* parent, Qt::WindowFlags f )
    : Vizkit3DWidget( parent, f ),
    envViz( new EnvireVisualization() ),
    robotViz( new AsguardVisualization() ),
    particleViz( new ParticleVisualization() ),
    referenceViz( new TrajectoryVisualization() ),
    centroidViz( new TrajectoryVisualization() ),
    filter( new MapVizEventFilter() )
{
    addDataHandler( envViz.get() );
    addDataHandler( robotViz.get() );
    addDataHandler( particleViz.get() );
    addDataHandler( referenceViz.get() );
    addDataHandler( centroidViz.get() );

    referenceViz->setColor( 0, 0, 0, 1.0 );
    centroidViz->setColor( 0.0, 0.0, 1.0, 1.0 );

    // switch to manual dirty handling
    envViz->handleDirty( false );

    // attach the filter
    envViz->setFilter( filter.get() );
}

EslamWidget::~EslamWidget()
{
    removeDataHandler( envViz.get() );
    removeDataHandler( robotViz.get() );
    removeDataHandler( particleViz.get() );
}

int EslamWidget::getInspectedParticleIndex() const
{
    return particleViz->getInspectedParticle();
}

void EslamWidget::setInspectedParticleIndex( int index )
{
    particleViz->inspectParticle( index );
}

void EslamWidget::setPoseDistribution( const eslam::PoseDistribution& dist )
{
    particleViz->updateData( dist );
}

void EslamWidget::setBodyState( const eslam::BodyContactState& body_state ) 
{
    /*
    asguardState.bodyState = body_state;
    robotViz->updateData( asguardState );
    */
}

void EslamWidget::setCentroidPose( const base::Pose& pose )
{
    centroidViz->updateData( base::Vector3d( pose.position ) );
}
    
void EslamWidget::setReferencePose( const base::Pose& pose )
{
    base::samples::RigidBodyState rbs;
    rbs.position = pose.position;
    rbs.orientation = pose.orientation;
    robotViz->updateData( rbs );

    referenceViz->updateData( base::Vector3d( pose.position ) );
}

void EslamWidget::setEnvironment( envire::Environment *env )
{
    envViz->updateData( env );
}

void EslamWidget::setDirty() 
{
    envViz->setDirty();
}

bool EslamWidget::isDirty() const
{
    return envViz->isDirty();
    envViz->setDirty();
}

void EslamWidget::viewMap( envire::MLSMap* map )
{
    filter->viewMap( map );
    envViz->setDirty();
}
