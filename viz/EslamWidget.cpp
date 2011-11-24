#include "EslamWidget.hpp"
#include "ParticleVisualization.hpp"

#include <eslam/PoseParticle.hpp>

#include <vizkit/AsguardVisualization.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <vizkit/TrajectoryVisualization.hpp>

#include <envire/maps/MLSMap.hpp>
#include <envire/core/EventHandler.hpp>

using namespace vizkit;
using namespace envire;

namespace vizkit {
/** this class is used to filter the events for a single particle
 * and hide the other particles (maps).
 */
class MapVizEventFilter : public envire::EventFilter
{
    std::set<envire::EnvironmentItem*> items;
    Environment* env;

public:
    bool filter( envire::Event const& event )
    {
	if( items.count( event.a.get() ) )
	{
	    if( event.type == Event::ITEM && event.operation == Event::REMOVE )
		items.erase( event.a.get() );
	    return true;
	}
	else 
	    return false;
    }

    void viewMap( envire::MLSMap* map )
    {
	env = map->getEnvironment();

	std::set<envire::EnvironmentItem*> update, add, remove;

	// collect the nodes that need to be updated 
	update.insert( env->getRootNode() );
	update.insert( map->getFrameNode() );

	for( std::vector<envire::MultiLevelSurfaceGrid::Ptr>::iterator it = map->grids.begin();
		it != map->grids.end(); it++ )
	{
	    update.insert( it->get() );
	    FrameNode *fn = (*it)->getFrameNode();
	    while( fn != env->getRootNode() )
	    {
		update.insert( fn );
		fn = fn->getParent();
	    }
	}

	// calculate add and remove sets
	std::set_difference( items.begin(), items.end(), update.begin(),
		update.end(), std::inserter( remove, remove.end() ) );
	std::set_difference( update.begin(), update.end(), items.begin(),
		items.end(), std::inserter( add, add.end() ) );

	// do a two pass system where the nodes get added first
	// and then the relations are set
	for( std::set<envire::EnvironmentItem*>::iterator it = add.begin(); it != add.end(); it++ )
	{
	    items.insert( *it );
	    env->handle( Event( Event::ITEM, Event::ADD, *it ) );
	}

	for( std::set<envire::EnvironmentItem*>::iterator it = add.begin(); it != add.end(); it++ )
	{
	    itemEvents( *it, Event::ADD );
	}
	    
	// now all the items that are in the erase list need to be removed do a two stage thins
	// again, where the relations are removed first, and then the items
	for( std::set<envire::EnvironmentItem*>::iterator it = remove.begin(); it != remove.end(); it++ )
	{
	    itemEvents( *it, Event::REMOVE );
	}

	for( std::set<envire::EnvironmentItem*>::iterator it = remove.begin(); it != remove.end(); it++ )
	{
	    env->handle( Event( Event::ITEM, Event::REMOVE, *it ) );
	    items.erase( *it );
	}
    }

protected:
    void itemEvents( EnvironmentItem* item, Event::Operation op )
    {
	if( dynamic_cast<envire::CartesianMap*>( item ) )
	{
	    MLSGrid *grid = dynamic_cast<envire::MLSGrid*>( item );
	    env->handle( Event( Event::FRAMENODE, op, grid, grid->getFrameNode() ) );
	}
	else if( dynamic_cast<envire::FrameNode*>( item ) )
	{
	    FrameNode *fn = dynamic_cast<envire::FrameNode*>( item );
	    if( env->getRootNode() == fn )
	    {
		env->handle( Event( Event::ROOT, op, fn ) );
	    }
	    else
	    {
		env->handle( Event( Event::FRAMENODE_TREE, op, fn->getParent(), fn ) );
	    }
	}
	else
	{
	    throw std::runtime_error("filter: don't know what to do with environment item. " + item->getClassName() );
	}
    }
	    
};
}

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

void EslamWidget::setBodyState( const asguard::BodyState& body_state ) 
{
    asguardState.bodyState = body_state;
    robotViz->updateData( asguardState );
}

void EslamWidget::setCentroidPose( const base::Pose& pose )
{
    centroidViz->updateData( base::Vector3d( pose.position ) );
}
    
void EslamWidget::setReferencePose( const base::Pose& pose )
{
    asguardState.rigidBodyState.position = pose.position;
    asguardState.rigidBodyState.orientation = pose.orientation;
    
    robotViz->updateData( asguardState );
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
