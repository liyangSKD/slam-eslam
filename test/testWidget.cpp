#include <QtGui/QApplication>
#include <vizkit/QtThreadedWidget.hpp>

#include "../viz/EslamWidget.hpp"

int main( int argc, char **argv )
{
    QtThreadedWidget<vizkit::EslamWidget> app;
    app.start();

    boost::shared_ptr<envire::Environment> env;
    if( argc >= 2 )
    {
	std::cout << "Loading environment : " << argv[1] << std::endl;
	// load environment from arg1
	envire::Serialization so;
	env = boost::shared_ptr<envire::Environment>(so.unserialize( argv[1] ));
	app.getWidget()->setEnvironment(env.get());
    }

    for(int i=0;i<500 && app.isRunning();i++)
    {
	asguard::BodyState bs;
	double r = i/10.0;

	bs.twistAngle = sin(r);
	for(int j=0;j<4;j++)
	    bs.wheelPos[j] = r;

	app.getWidget()->setBodyState( bs );
	app.getWidget()->setReferencePose( base::Pose( Eigen::Vector3d( 0, r * 0.1, 0 ), Eigen::Quaterniond::Identity() ) );
	app.getWidget()->setCentroidPose( base::Pose( Eigen::Vector3d( r * 0.02, r * 0.1, 0 ), Eigen::Quaterniond::Identity() ) );

	usleep(50*1000);
    }
}

