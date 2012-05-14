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

#include <numeric/stats.hpp>

#include <boost/program_options.hpp>
#include <fstream>

using namespace envire;
using namespace vizkit;
namespace po = boost::program_options;

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

    size_t max_steps;

    MapTest()
	: 
	env(0), grid(0),
	nrand( boost::mt19937(time(0)),
		boost::normal_distribution<>()),
	sigma_step( 0.0 ),
	sigma_body( 0.0 ),
	sigma_sensor( 0.0 ),
	max_steps( 500 )
    {
    }

    virtual ~MapTest()
    {
    }

    void init()
    {
	if( grid )
	    env->detachItem( grid );

	grid = new MLSGrid( 200, 200, 0.05, 0.05, -5, -5 );
	env->setFrameNode( grid, env->getRootNode() );

	sim = AsguardSim();

	z_pos = sim.body2world.translation().z();
	z_var = 0;
    }

    virtual void run()
    {
	for(size_t i=0; i<max_steps; i++)
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
	for(size_t i=0;i<max_steps && app.isRunning();i++)
	{
	    step( i );
	    usleep(100*1000);
	    updateViz();
	}
    }
};

struct StatMapTest : public MapTest
{
    std::vector<base::Stats<double> > height;
    std::vector<double> forward;

    size_t max_runs;
    std::ofstream out;

    StatMapTest( int max_runs, std::string const& file )
	: max_runs( max_runs )
    {
	out.open( file.c_str() );
	height.resize( max_steps );
	forward.resize( max_steps );

	env = new envire::Environment();
    }

    ~StatMapTest()
    {
	delete env;
    }

    virtual void run()
    {
	for( size_t run=0; run<max_runs; run++ )
	{
	    std::cerr << "run " << run << "     \r";
	    for( size_t i=0; i<max_steps; i++ )
	    {
		step(i);
		height[i].update( z_pos );
		forward[i] = sim.body2world.translation().y();
	    }
	    init();
	}

	for( size_t i=0; i<max_steps; i++ )
	{
	    out 
		<< i << " "
		<< forward[i] << " "
		<< height[i].mean() << " "
		<< height[i].stdev() << " "
		<< height[i].min() << " "
		<< height[i].max() << " "
		<< std::endl;
	}
    }
};

int main( int argc, char **argv )
{
    MapTest *mt;
    std::string mode = argv[1];
    if( mode == "viz" )
    {
	mt = new VizMapTest;
    }
    else if( mode == "batch" )
    {
	mt = new StatMapTest( 150, "res.out" );
    }
    else
	throw std::runtime_error("mode needs to be either viz or batch");

    if( argc >= 3 )
	mt->sigma_step = boost::lexical_cast<double>( argv[2] );
    if( argc >= 4 )
	mt->sigma_sensor = boost::lexical_cast<double>( argv[3] );
    if( argc >= 5 )
	mt->sigma_body = boost::lexical_cast<double>( argv[4] );

    mt->init();
    mt->run();

    delete mt;
}
