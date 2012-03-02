#ifndef VIZKIT_MAPVIZEVENTFILTER__
#define VIZKIT_MAPVIZEVENTFILTER__

#include <envire/Core.hpp>
#include <envire/core/EventHandler.hpp>
#include <envire/maps/MLSGrid.hpp>

namespace vizkit {
/** this class is used to filter the events for a single particle
 * and hide the other particles (maps).
 */
class MapVizEventFilter : public envire::EventFilter
{
    std::set<envire::EnvironmentItem*> items;
    envire::Environment* env;

public:
    bool filter( envire::Event const& event )
    {
	if( items.count( event.a.get() ) )
	{
	    if( event.type == envire::event::ITEM && event.operation == envire::event::REMOVE )
		items.erase( event.a.get() );
	    return true;
	}
	else 
	    return false;
    }

    void viewMap( envire::MLSMap* map )
    {
	if( !handler )
	    throw std::runtime_error( "EventFilter not connected to handler" );

	env = map->getEnvironment();

	std::set<envire::EnvironmentItem*> update, add, remove;

	// collect the nodes that need to be updated 
	update.insert( env->getRootNode() );
	update.insert( map->getFrameNode() );

	for( std::vector<envire::MultiLevelSurfaceGrid::Ptr>::iterator it = map->grids.begin();
		it != map->grids.end(); it++ )
	{
	    update.insert( it->get() );
	    envire::FrameNode *fn = (*it)->getFrameNode();
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
	    handler->handle( envire::Event( envire::event::ITEM, envire::event::ADD, *it ) );
	}

	for( std::set<envire::EnvironmentItem*>::iterator it = add.begin(); it != add.end(); it++ )
	{
	    itemEvents( *it, envire::event::ADD );
	}
	    
	// now all the items that are in the erase list need to be removed do a two stage thins
	// again, where the relations are removed first, and then the items
	for( std::set<envire::EnvironmentItem*>::iterator it = remove.begin(); it != remove.end(); it++ )
	{
	    itemEvents( *it, envire::event::REMOVE );
	}

	for( std::set<envire::EnvironmentItem*>::iterator it = remove.begin(); it != remove.end(); it++ )
	{
	    handler->handle( envire::Event( envire::event::ITEM, envire::event::REMOVE, *it ) );
	    items.erase( *it );
	}
    }

protected:
    void itemEvents( envire::EnvironmentItem* item, envire::event::Operation op )
    {
	if( dynamic_cast<envire::CartesianMap*>( item ) )
	{
	    envire::MLSGrid *grid = dynamic_cast<envire::MLSGrid*>( item );
	    handler->handle( envire::Event( envire::event::FRAMENODE, op, grid, grid->getFrameNode() ) );
	}
	else if( dynamic_cast<envire::FrameNode*>( item ) )
	{
	    envire::FrameNode *fn = dynamic_cast<envire::FrameNode*>( item );
	    if( env->getRootNode() == fn )
	    {
		handler->handle( envire::Event( envire::event::ROOT, op, fn ) );
	    }
	    else
	    {
		handler->handle( envire::Event( envire::event::FRAMENODE_TREE, op, fn->getParent(), fn ) );
	    }
	}
	else
	{
	    throw std::runtime_error("filter: don't know what to do with environment item. " + item->getClassName() );
	}
    }
	    
};
}

#endif
