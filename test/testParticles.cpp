#include <QtGui/QApplication>
#include <osg/ArgumentParser>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

#include "ApplicationWindow.hpp"
#include "GridNode.h"
#include "ParticleVisualization.h"
#include <envire/Core.hpp>
#include <envire/maps/LaserScan.hpp>
#include "EnvireEventListener.h"
#include "LaserScanVisualization.h"
#include "FrameNodeVisualization.h"
#include "Robot.h"

#include <base/pose.h>
#include <base/timemark.h>

#include <asguard/EmbodiedSlamFilter.hpp>

#include <osgDB/WriteFile>

class Application: public QApplication {
public:
   Application(int &c, char **v): QApplication(c, v) {}
   bool notify(QObject *rec, QEvent *ev) {
       try {
           return QApplication::notify(rec, ev);
       }
       catch (char const *str) {
	   std::cerr << "EXCEPTION: " << str << std::endl;
           return false;
       }
   }
};


int main( int argc, char **argv )
{
    Application a( argc, argv );
    osg::ArgumentParser arguments(&argc, argv);

    // setup mainWindow and get pointer to environment
    vizkit::ApplicationWindow mainWindow;

    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );

    vizkit::GridNode *gn = new enview::GridNode();
    mainWindow.addNode(gn);

    // test setup for pose visualisation
    vizkit::Robot *rv = new enview::Robot();
    mainWindow.addNode(rv->getNode());
    //mainWindow.registerSequenceData(rv);

    // create a dummy node that can be tracked as the home position
    osg::ref_ptr<osg::Node> homeNode = new osg::Node();
    rv->getNode()->addChild( homeNode );
    //mainWindow.getInstance()->getView()->setTrackedNode( homeNode );

    // test setup for particle filter visualisation
    vizkit::ParticleVisualization *pv = new enview::ParticleVisualization();
    mainWindow.addNode(pv->getNode());
    mainWindow.registerSequenceData(pv);

    int max_steps = std::numeric_limits<int>::max();

    if( argc > 2 )
    {
	// load environment from arg1
	envire::Serialization so;
	envire::Environment *env = so.unserialize( argv[1] );
	mainWindow.setEnvironment(env);

	// read logfile from arg2
	//asguard::SimulationLog log( argv[2] );
	asguard::PocosimLog log( argv[2] );
	std::cout << "reading from log file " << argv[2] << std::endl;

	if( argc > 3 )
	{
	    max_steps = boost::lexical_cast<int>( argv[3] );
	}

	asguard::Configuration config;
	eslam::EmbodiedSlamFilter filter(config);
	if( !log.next() )
	    exit(0);

	filter.init( env, log.getPose() );

	int n = 0;
	double time = 0;
	while( log.next() && (max_steps > n )) 
	{
	    // update the real robots pose
	    vizkit::RobotState state;
	    state.rigidBodyState.position = log.getPose().position;
	    state.rigidBodyState.orientation = log.getPose().orientation;
	    state.bodyState = log.getBodyState();
	    
	    rv->updateData( base::Time::fromSeconds(time), state );

	    filter.update( log.getBodyState(), log.getPose().orientation );

	    // inject odometry data into particle filter
	    vizkit::particleVector vec;
	    std::vector<eslam::PoseEstimator::Particle>& xi_k( filter.getParticles() );

	    for(size_t i=0;i<xi_k.size();i++)
	    {
		eslam::PoseEstimator::Particle& p( xi_k[i] );
		vec.push_back( p );
	    }
	    pv->updateData( base::Time::fromSeconds(time), vec );


	    time+=.01;
	    n++;
	}
	std::cout << "read " << n << " lines." << std::endl;
    }

    mainWindow.updateSequenceData();

    //osgDB::writeNodeFile( *mainWindow.getRootNode(), "/tmp/test.dot" );
    
    return a.exec();
    
}

