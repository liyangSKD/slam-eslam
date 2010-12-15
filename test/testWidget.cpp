#include <QtGui/QApplication>

#include "../viz/EslamWidget.hpp"

int main( int argc, char **argv )
{
    QApplication a( argc, argv );
    osg::ArgumentParser arguments(&argc, argv);

    vizkit::EslamWidget widget;

    boost::shared_ptr<envire::Environment> env;
    if( argc >= 2 )
    {
	std::cout << "Loading environment : " << argv[1] << std::endl;
	// load environment from arg1
	envire::Serialization so;
	env = boost::shared_ptr<envire::Environment>(so.unserialize( argv[1] ));
	widget.setEnvironment(env.get());
    }

    widget.show();
    return a.exec();
}

