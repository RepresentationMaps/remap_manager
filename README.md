reMap Manager
======================

This package implements the reMap manager. The reMap manager has the following responsibilities:
- manage the activation and deactivation of reMap plugins;
- initializing the objects shared among the plugins.

Two classes are implemented as part of this package:
- `RemapManager` (inheriting from `rclcpp::Node`): implementing the basic functionalities of a reMap manager, including the ROS 2 interface.
- `SemanticRemapManager`: inheriting from `RemapManager`, it implements and manages all the functionalities relative to the semantic representation,
such as the map_handler and the regions register.

This package also provides a ROS 2 node, `remap_manager_node`, initializing and spinning a `SemanticRemapManager` object.

## ROS 2 Interface

### Nodes

#### remap_manager_node

This node initializes and spins a `SemanticRemapManager` object, which implements the ROS 2 interface for activating and deactivating reMap plugins.
This node might also subscribe to various other topics, depending on the activated plugins. All the plugins share this node's interface to subscribe to the required topics.
Check each plugin documentation to verify which additional topics this node might subscribe to.

##### Executing the node

`ros2 run remap_manager remap_manager_node`

##### Launch files

`remap_manager.launch.py`: launches the `remap_manager_node`.

##### Topics

- `/remap/world` ([`sensor_msgs::msg::PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)): the full reMap grid, published as a point cloud.

##### Services

- `/remap_manager/activate_plugin` ([`remap_msgs/srv/AddPlugin`](https://gitlab.pal-robotics.com/interaction/remap_msgs/-/blob/main/srv/AddPlugin.srv?ref_type=heads)): 
interface to activate a certain reMap plugin. It returns a boolean, representing activation success.

- `/remap_manager/remove_plugin` ([`remap_msgs/srv/AddPlugin`](https://gitlab.pal-robotics.com/interaction/remap_msgs/-/blob/main/srv/RemovePlugin.srv?ref_type=heads)):
interface to deactivate a certain reMap plugin. It returns a boolean, representing deactivation success.