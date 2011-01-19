#include "EslamWidget.hpp"
#include "ParticleVisualization.hpp"

#include <eslam/PoseParticle.hpp>

#include <vizkit/AsguardVisualization.hpp>
#include <vizkit/EnvireVisualization.hpp>

using namespace vizkit;

EslamWidget::EslamWidget( QWidget* parent, Qt::WindowFlags f )
    : QVizkitWidget( parent, f ),
    envViz( new EnvireVisualization() ),
    robotViz( new AsguardVisualization() ),
    particleViz( new ParticleVisualization() )
{
    addDataHandler( envViz.get() );
    addDataHandler( robotViz.get() );
    addDataHandler( particleViz.get() );
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

void EslamWidget::setReferencePose( const base::Pose& pose, const asguard::BodyState& body_state )
{
    vizkit::AsguardState state;
    state.rigidBodyState.position = pose.position;
    state.rigidBodyState.orientation = pose.orientation;
    state.bodyState = body_state;
    
    robotViz->updateData( state );
}

void EslamWidget::setEnvironment( envire::Environment *env )
{
    envViz->updateData( env );
}
