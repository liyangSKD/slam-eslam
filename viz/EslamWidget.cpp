#include "EslamWidget.hpp"
#include "ParticleVisualization.hpp"

#include <eslam/PoseParticle.hpp>

#include <vizkit/AsguardVisualization.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <vizkit/TrajectoryVisualization.hpp>

using namespace vizkit;

EslamWidget::EslamWidget( QWidget* parent, Qt::WindowFlags f )
    : QVizkitWidget( parent, f ),
    envViz( new EnvireVisualization() ),
    robotViz( new AsguardVisualization() ),
    particleViz( new ParticleVisualization() ),
    referenceViz( new TrajectoryVisualization() ),
    centroidViz( new TrajectoryVisualization() )
{
    addDataHandler( envViz.get() );
    addDataHandler( robotViz.get() );
    addDataHandler( particleViz.get() );
    addDataHandler( referenceViz.get() );
    addDataHandler( centroidViz.get() );

    referenceViz->setColor( 1.0, 0, 0, 1.0 );
    centroidViz->setColor( 0.0, 1.0, 0.0, 1.0 );

    // switch to manual dirty handling
    envViz->handleDirty( false );
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

void EslamWidget::setBodyState( const asguard::BodyState& body_state ) 
{
    asguardState.bodyState = body_state;
    robotViz->updateData( asguardState );
}

void EslamWidget::setCentroidPose( const base::Pose& pose )
{
    centroidViz->updateData( Eigen::Vector3d( pose.position ) );
}
    
void EslamWidget::setReferencePose( const base::Pose& pose )
{
    asguardState.rigidBodyState.position = pose.position;
    asguardState.rigidBodyState.orientation = pose.orientation;
    
    robotViz->updateData( asguardState );
    referenceViz->updateData( Eigen::Vector3d( pose.position ) );
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
