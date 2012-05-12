#include <vizkit/QtThreadedWidget.hpp>
#include <vizkit/EnvireWidget.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <vizkit/AsguardVisualization.hpp>
#include <odometry/ContactOdometry.hpp>
#include <asguard/Configuration.hpp>

#include <Eigen/Geometry>

using namespace envire;
using namespace vizkit;

struct MapTest
{
    QtThreadedWidget<envire::EnvireWidget> app;
    AsguardVisualization aviz;
    Environment *env;
    MLSGrid* grid; 
    asguard::Configuration asguardConfig;
    asguard::BodyState bodyState;
    odometry::FootContact odometry;
    Eigen::Affine3d body2world;

    MapTest()
	: odometry( odometry::Configuration() )
    {
    }

    void init()
    {
	app.start();
	app.getWidget()->addPlugin( &aviz );
	env = app.getWidget()->getEnvironment();

	grid = new MLSGrid( 100, 100, 0.05, 0.05, -2.5, -2.5 );
	env->setFrameNode( grid, env->getRootNode() );

	body2world = Eigen::Affine3d::Identity();
    }

    void run()
    {
	for(int i=0;i<500 && app.isRunning();i++)
	{
	    step( i );
	    usleep(50*1000);
	}
    }

    void step( int i )
    {
	// odometry udpate
	for(int j=0;j<4;j++)
	    bodyState.wheelPos[j] += 0.1;

	odometry::BodyContactState bcs;
	asguardConfig.setContactState( bodyState, bcs );
	odometry.update( bcs, Eigen::Quaterniond::Identity() );
	body2world = body2world * odometry.getPoseDelta().toTransform();

	aviz.updateData( bodyState );
	aviz.updateTransform( body2world );

	// grid update
	MLSGrid::Position p;
	if( grid->toGrid( Eigen::Vector2d( 0, (i+0.5)/20.0 ), p ) )
	{
	    grid->updateCell( 
		    p,
		    MLSGrid::SurfacePatch( 0.0, 0.1 ) );

	    env->itemModified( grid );
	}
    }

};

int main( int argc, char **argv )
{
    MapTest mt;

    mt.init();
    mt.run();
}
