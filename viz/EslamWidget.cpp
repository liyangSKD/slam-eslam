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
    envViz( new envire::EnvireVisualization() ),
    robotViz( new AsguardVisualization() ),
    particleViz( new ParticleVisualization() ),
    referenceViz( new TrajectoryVisualization() ),
    centroidViz( new TrajectoryVisualization() ),
    filter( new MapVizEventFilter() ),
    pe( NULL )
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

    // connect signals for particle inspection
    connect( particleViz.get(), SIGNAL(inspectParticleUpdate(int)), this, SLOT(viewMap(int)) );
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

void EslamWidget::setPoseParticles( std::vector<eslam::PoseParticleGA> *pe )
{
    this->pe = pe;
}

void EslamWidget::setDirty() 
{
    envViz->setDirty();
}

bool EslamWidget::isDirty() const
{
    return envViz->isDirty();
}

void EslamWidget::viewMap( int particle_idx )
{
    if( pe && particle_idx >= 0 && (size_t)particle_idx < pe->size() )
    {
	envire::MLSMap *map = (*pe)[particle_idx].grid.getMap();
	filter->viewMap( map );
	envViz->setDirty();
    }
}
