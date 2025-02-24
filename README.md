Representation Manager
======================

This package implements the representation manager, as described in the submitted paper.

Building
--------

`$ rosdep init`

`$ rosdep update`

`$ rosdep install -y --skip-keys map_handler --skip-keys representation_plugin_base --skip-keys vdb2pc --skip-keys OpenCV --skip-keys openvdb --skip-keys reg_of_space_server --skip-keys representation_plugins --from-path src/representation_manager`

`$ colcon build --packages-select representation_manager`

Running
-------

The package offers a ROS 2 node implemeting the Representation Manager as described in the paper.
To run it: `ros2 run representation_manager representation_manager_node`

Interfaces
**********

The interfaces exposed by the node are:
- `/add_plugin` (service, type: `representation_manager/srv/AddPlugin`): it expects the name of the representation plugin to load.
- `/remove_plugin` (service, type: `representation_manager/srv/RemovePlugin`): it expects the name of the representation plugin to stop.