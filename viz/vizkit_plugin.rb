Vizkit::UiLoader.register_3d_plugin('BodyContactStateVisualization', 'eslam', 'BodyContactStateVisualization')
Vizkit::UiLoader.register_3d_plugin_for('BodyContactStateVisualization', "/odometry/BodyContactState", :updateContactState )

Vizkit::UiLoader.register_3d_plugin('ParticleVisualization', 'eslam', 'ParticleVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ParticleVisualization', "/eslam/PoseDistribution", :updatePoseDistribution )
