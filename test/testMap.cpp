#include <vizkit/QtThreadedWidget.hpp>
#include <vizkit/EnvireWidget.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <vizkit/AsguardVisualization.hpp>
#include <odometry/ContactOdometry.hpp>
#include <asguard/Configuration.hpp>
#include <eslam/ContactModel.hpp>

#include <Eigen/Geometry>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <numeric/stats.hpp>

#include <fstream>

using namespace std;
using namespace envire;
using namespace vizkit;

struct Config
{
    double sigma_step, sigma_body, sigma_sensor;

    size_t max_steps;
    size_t max_runs;
    size_t min_contacts;

    string result_file;

    void set( string const& conf_file )
    {
	ifstream cf( conf_file.c_str() );
	string line;
	map<string,string> conf;
	while( getline( cf, line ) )
	{
	    vector<string> words;
	    boost::split(words, line, boost::is_any_of("="), boost::token_compress_on);
	    if( words.size() == 2 )
	    {
		conf[words[0]] = words[1];
	    }
	}
	sigma_step = boost::lexical_cast<double>(conf["sigma_step"]);
	sigma_body = boost::lexical_cast<double>(conf["sigma_body"]);
	sigma_sensor = boost::lexical_cast<double>(conf["sigma_sensor"]);
	max_steps = boost::lexical_cast<size_t>(conf["max_steps"]);
	max_runs = boost::lexical_cast<size_t>(conf["max_runs"]);
	min_contacts = boost::lexical_cast<size_t>(conf["min_contacts"]);
	result_file = conf["result_file"];
    }
};

struct AsguardSim
{
    asguard::Configuration asguardConfig;
    asguard::BodyState bodyState;
    odometry::FootContact odometry;
    Eigen::Affine3d body2world;
    odometry::BodyContactState contactState;

    AsguardSim()
	: odometry( odometry::Configuration() )
    {
	body2world = Eigen::Affine3d::Identity();
	for(int j=0;j<4;j++)
	    bodyState.wheelPos[j] = 0.0;
	bodyState.twistAngle = 0;

	// put the robot so that the feet are in 0 height
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

	    asguardConfig.setContactState( bodyState, contactState );
	    odometry.update( contactState, Eigen::Quaterniond::Identity() );
	    body2world = body2world * odometry.getPoseDelta().toTransform();
	}
	// odometry seems to get z height wrong in the case of
	// a foot transition
	// TODO investigate and fix
	body2world.translation().z() = 
	    -asguardConfig.getLowestFootPosition( bodyState ).z();
    }
};

struct MapTest
{
    Environment *env;
    MLSGrid* grid; 

    AsguardSim sim;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > nrand;
    eslam::ContactModel contactModel;

    Config conf;

    double z_var, z_pos;

    MapTest()
	: 
	env(0), grid(0),
	nrand( boost::mt19937(time(0)),
		boost::normal_distribution<>())
    {
    }

    virtual ~MapTest()
    {
    }

    virtual void init()
    {
	if( grid )
	    env->detachItem( grid );

	grid = new MLSGrid( 200, 200, 0.05, 0.05, -5, 0 );
	env->setFrameNode( grid, env->getRootNode() );

	sim = AsguardSim();

	z_pos = sim.body2world.translation().z();
	z_var = 0;

	contactModel.setMinContacts( conf.min_contacts );
    }

    virtual void run()
    {
	for(size_t i=0; i<conf.max_steps; i++)
	{
	    step( i );
	}
    }

    bool getMap( Eigen::Vector3d const& pos, MLSGrid::SurfacePatch& patch )
    {
	MLSGrid::Position pi;
	if( grid->toGrid( (pos).head<2>(), pi ) )
	{
	    MLSGrid::SurfacePatch *p = grid->get( pi, patch ); 
	    if( p )
	    {
		patch = *p;
		return true;
	    }
	}
	return false;
    }

    virtual void step( int i )
    {
	// run simulation and get real z_delta
	double z_delta = sim.body2world.translation().z();
	sim.step();
	z_delta = sim.body2world.translation().z() - z_delta;

	// handle z position uncertainty
	z_pos += z_delta + nrand() * conf.sigma_step;
	z_var += pow(conf.sigma_step,2);

	// our believe of body2world
	Eigen::Affine3d body2world( sim.body2world );
	body2world.translation().z() = z_pos;

	// measurement of the body on the grid
	contactModel.setContactPoints( 
		sim.contactState, 
		Eigen::Quaterniond(body2world.linear()) );

	bool hasContact = contactModel.evaluatePose( 
		Eigen::Affine3d( Eigen::Translation3d( body2world.translation() ) ), 
		pow( conf.sigma_body, 2 ), 
		boost::bind( &MapTest::getMap, this, _1, _2 ) );

	if( hasContact )
	{
	    z_pos += contactModel.getZDelta();
	}

	// generate grid cells
	for( size_t i=0; i<50; i++ )
	{
	    // z height of measurement
	    double z_meas = 
		-sim.body2world.translation().z() 
		+ nrand() * conf.sigma_sensor; 

	    Eigen::Vector3d m_pos( 
		    ((float)i-25.0)*0.02, 
		    1.0, 
		    z_meas );
	    m_pos = body2world * m_pos;
	    MLSGrid::Position p;
	    if( grid->toGrid( (m_pos).head<2>(), p ) )
	    {
		double sigma = sqrt( pow(conf.sigma_sensor,2) + z_var );
		// for now, only add new cells
		if( grid->beginCell( p.x, p.y ) == grid->endCell() )
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
	for(size_t i=0;i<conf.max_steps && app.isRunning();i++)
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
    std::vector<double> z_variance;
    std::vector<base::Stats<double> > map_z;
    std::vector<double> map_stdev;

    std::ofstream out;

    StatMapTest()
    {
	env = new envire::Environment();
    }

    void init()
    {
	MapTest::init();

	height.resize( conf.max_steps );
	forward.resize( conf.max_steps );
	z_variance.resize( conf.max_steps );
	map_z.resize( conf.max_steps );
	map_stdev.resize( conf.max_steps );
    }

    ~StatMapTest()
    {
	delete env;
    }

    virtual void run()
    {
	for( size_t run=0; run<conf.max_runs; run++ )
	{
	    std::cerr << "run " << run << "     \r";
	    for( size_t i=0; i<conf.max_steps; i++ )
	    {
		step(i);

		// store results
		height[i].update( z_pos - sim.body2world.translation().z() );
		forward[i] = sim.body2world.translation().y();
		z_variance[i] = z_var;

		// get map height
		MLSGrid::Position p;
		if( grid->toGrid( (sim.body2world.translation()).head<2>(), p ) )
		{
		    MLSGrid::iterator it = grid->beginCell( p.x, p.y );
		    if( it != grid->endCell() )
		    {
			map_z[i].update( it->mean );
			map_stdev[i] = it->stdev;
		    }
		}
	    }
	    init();
	}

	out.open( conf.result_file.c_str() );
	for( size_t i=0; i<conf.max_steps; i++ )
	{
	    out 
		<< i << " "
		<< forward[i] << " "
		<< height[i].mean() << " "
		<< height[i].stdev() << " "
		<< sqrt(z_variance[i]) << " "
		<< map_z[i].mean() << " "
		<< map_z[i].stdev() << " "
		<< map_stdev[i] << " "
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
	mt = new StatMapTest();
    }
    else
	throw std::runtime_error("mode needs to be either viz or batch");

    if( argc >=3 )
	mt->conf.set( argv[2] );

    mt->init();
    mt->run();

    delete mt;
}
