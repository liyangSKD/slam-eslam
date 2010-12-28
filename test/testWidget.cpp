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
	app.widget->setEnvironment(env.get());
    }

    for(int i=0;i<500 && app.isRunning();i++)
    {
	asguard::BodyState bs;
	double r = i/10.0;

	bs.twistAngle = sin(r);
	for(int j=0;j<4;j++)
	    bs.wheelPos[j] = r;

	app.widget->setReferencePose( base::Pose(), bs );

	usleep(50*1000);
    }
}

