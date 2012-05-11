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

	aviz.updateData( bodyState );

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

    void fitPlane( asguard::BodyState &body_state )
    {
	// find the twist angle, such that we can fit a plane
	// to the points
	float eps = 1e-3, error = 1e3;
	float step = 0.1;
	size_t contact_idx[4] = {0,0,0,0};

	while( error > eps )
	{
	    Eigen::Vector3d p1, p2, p3, p4;
	    p1 = asguardConfig.getFootPosition( body_state, asguard::wheelIdxEnum(0), contact_idx[0] );
	    p2 = asguardConfig.getFootPosition( body_state, asguard::wheelIdxEnum(1), contact_idx[1] );
	    p3 = asguardConfig.getFootPosition( body_state, asguard::wheelIdxEnum(2), contact_idx[2] );
	    p4 = asguardConfig.getFootPosition( body_state, asguard::wheelIdxEnum(3), contact_idx[3] );

	    Eigen::Matrix3d r;
	    r.row(0) = p2 - p1;
	    r.row(1) = (p3 - p1).cross( r.row(0) );
	    r.row(2) = r.row(0).cross( r.row(1) );
	    Eigen::Affine3d t;
	    t.linear() = r;
	    t.translation() = p1;

	    for(int wheel_idx = 0; wheel_idx < 4; wheel_idx++)
	    {
		for (unsigned int foot_idx = 0; foot_idx < 5; ++foot_idx)
		{
		    Eigen::Vector3d p = 
			asguardConfig.getFootPosition(body_state, asguard::wheelIdxEnum(wheel_idx), foot_idx);

		    if( (t.inverse() * p).z() < 0 )
		    {
			if( wheel_idx == 3 )
			{

			}
			else 
			{
			    contact_idx[wheel_idx] = (contact_idx[wheel_idx]+1) % 4;
			    break;
			}
		    }
		}
	    }
	}
    }
};

int main( int argc, char **argv )
{
    MapTest mt;

    mt.init();
    mt.run();
}
