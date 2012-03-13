#include <vizkit/Vizkit3DPlugin.hpp>
#include "ParticleVisualization.hpp"
#include "BodyContactStateVisualization.hpp"

namespace vizkit {
    class QtPluginVizkit : public vizkit::VizkitPluginFactory {
    private:
    public:
	
	QtPluginVizkit() {
	}
	
	/**
	* Returns a list of all available visualization plugins.
	* @return list of plugin names
	*/
        virtual QStringList* getAvailablePlugins() const
	{
	    QStringList *pluginNames = new QStringList();
	    pluginNames->push_back("ParticleVisualization");
	    pluginNames->push_back("BodyContactStateVisualization");
	    return pluginNames;
	}
	
        virtual QObject* createPlugin(QString const& pluginName)
        {
	    vizkit::VizPluginBase* plugin = 0;
	    if (pluginName == "ParticleVisualization")
	    {
		plugin = new vizkit::ParticleVisualization();
	    }
	    else if (pluginName == "BodyContactStateVisualization")
	    {
		plugin = new vizkit::BodyContactStateVisualization();
	    }

	    if (plugin) 
	    {
		return plugin;
	    }
	    return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkit, QtPluginVizkit)
}
