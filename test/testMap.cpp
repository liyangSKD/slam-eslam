#include <vizkit/QtThreadedWidget.hpp>
#include <vizkit/EnvireWidget.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <vizkit/AsguardVisualization.hpp>
#include <odometry/ContactOdometry.hpp>
#include <asguard/Configuration.hpp>

#include <Eigen/Geometry>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/lexical_cast.hpp>

using namespace envire;
using namespace vizkit;

struct AsguardSim
{
    asguard::Configuration asguardConfig;
    asguard::BodyState bodyState;
    odometry::FootContact odometry;
    Eigen::Affine3d body2world;

    AsguardSim()
	: odometry( odometry::Configuration() )
    {
	body2world = Eigen::Affine3d::Identity();
	for(int j=0;j<4;j++)
	    bodyState.wheelPos[j] = 0.0;
	bodyState.twistAngle = 0;
	body2world.translation().z() = 
	    -asguardConfig.getLowestFootPosition( bodyState ).z();
    }

    void step()
    {
	for( int s=0; s<10; s++ )
	{
	    // odometry udpate
	    for(int j=0;j<4;j++)
		bodyState.wheelPos[j] += 0.01;

	    odometry::BodyContactState bcs;
	    asguardConfig.setContactState( bodyState, bcs );
	    odometry.update( bcs, Eigen::Quaterniond::Identity() );
	    body2world = body2world * odometry.getPoseDelta().toTransform();
	}
    }
};

struct MapTest
{
    Environment *env;
    MLSGrid* grid; 

    AsguardSim sim;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > nrand;

    double sigma_step, sigma_body, sigma_sensor;
    double z_var, z_pos;

    MapTest()
	: nrand( boost::mt19937(time(0)),
	     boost::normal_distribution<>()),
	sigma_step( 0.0 ),
	sigma_body( 0.0 ),
	sigma_sensor( 0.0 )
    {
    }

    void init()
    {
	grid = new MLSGrid( 200, 200, 0.05, 0.05, -5, -5 );
	env->setFrameNode( grid, env->getRootNode() );

	z_pos = sim.body2world.translation().z();
	z_var = 0;
    }

    virtual void run()
    {
	for(int i=0;i<500;i++)
	{
	    step( i );
	}
    }

    virtual void step( int i )
    {
	// run simulation and get real z_delta
	double z_delta = sim.body2world.translation().z();
	sim.step();
	z_delta = sim.body2world.translation().z() - z_delta;

	// handle z position uncertainty
	z_pos += z_delta + nrand() * sigma_step;
	z_var += pow(sigma_step,2);

	// our believe of body2world
	Eigen::Affine3d body2world( sim.body2world );
	body2world.translation().z() = z_pos;

	// generate grid cells
	for( size_t i=0; i<50; i++ )
	{
	    // z height of measurement
	    double z_meas = 
		-sim.body2world.translation().z() 
		+ nrand() * sigma_sensor; 

	    Eigen::Vector3d m_pos( 
		    ((float)i-25.0)*0.02, 
		    1.0, 
		    z_meas );
	    m_pos = body2world * m_pos;
	    MLSGrid::Position p;
	    if( grid->toGrid( (m_pos).head<2>(), p ) )
	    {
		double sigma = sqrt( pow(sigma_sensor,2) + z_var );
		grid->updateCell( 
			p,
			MLSGrid::SurfacePatch( m_pos.z(), sigma ) );
	    }
	}
    }

};

struct VizMapTest : public MapTest
{
    QtThreadedWidget<envire::EnvireWidget> app;
    AsguardVisualization aviz;

    VizMapTest()
    {
	app.start();
	app.getWidget()->addPlugin( &aviz );
	env = app.getWidget()->getEnvironment();
    }

    void updateViz()
    {
	// viz update
	aviz.updateData( sim.bodyState );
	aviz.updateTransform( sim.body2world );

	env->itemModified( grid );
    }

    virtual void run()
    {
	for(int i=0;i<500 && app.isRunning();i++)
	{
	    step( i );
	    usleep(100*1000);
	    updateViz();
	}
    }
};

int main( int argc, char **argv )
{
    VizMapTest mt;
    mt.init();

    if( argc >= 2 )
	mt.sigma_step = boost::lexical_cast<double>( argv[1] );
    if( argc >= 3 )
	mt.sigma_sensor = boost::lexical_cast<double>( argv[2] );
    if( argc >= 4 )
	mt.sigma_body = boost::lexical_cast<double>( argv[3] );

    mt.run();
}
