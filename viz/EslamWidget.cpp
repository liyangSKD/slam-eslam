#include "EslamWidget.hpp"
#include "ParticleVisualization.hpp"

#include <eslam/PoseParticle.hpp>

#include <enview/AsguardVisualization.hpp>
#include <enview/EnvireVisualization.hpp>

using namespace enview;

EslamWidget::EslamWidget( QWidget* parent, Qt::WindowFlags f )
    : QEnviewWidget( parent, f ),
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

void EslamWidget::setPoseDistribution( const eslam::PoseDistribution& dist )
{
    particleViz->updateData( dist );
}

void EslamWidget::setReferencePose( const base::Pose& pose, const asguard::BodyState& body_state )
{
    enview::AsguardState state;
    state.rigidBodyState.position = pose.position;
    state.rigidBodyState.orientation = pose.orientation;
    state.bodyState = body_state;
    
    robotViz->updateData( state );
}

void EslamWidget::setEnvironment( envire::Environment *env )
{
    envViz->updateData( env );
}
