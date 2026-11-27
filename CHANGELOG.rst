^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remap_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-11-27)
------------------
* Publish the grid as a compressed vdb_msgs/VdbGrid on /remap/world/vdb
  The grid with its region -> entities legend, for rviz_vdb_plugins: a cheap
  snapshot in the mapping cycle (only when someone subscribes, at most
  vdb_rate_hz, default 2), pruned, serialised (OpenVDB stream, Blosc by
  default: vdb_compression) and published by a worker thread, latched.
  Measured on the table-top sim (316k voxels at 2 cm): 406 kB per grid,
  0.76 MB/s, against 5 MB per PointCloud2 and 39 MB/s on /remap/world.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.3.2 (2026-02-18)
------------------
* compat with jazzy
  - avoid CMake conflict with TBB
* Contributors: Séverin Lemaignan

0.3.1 (2025-10-13)
------------------
* Fix ament_auto warning about headers install destination
* Contributors: Noel Jimenez

0.3.0 (2025-06-18)
------------------
* making linters happy
  Lint whispers gently,
  Trailing commas in the breeze—
  Silence means it's clean.
* fix plugin node warning
  we are providing the node name parameter via launch file.
  However, we only want that parameter to affect the remap manager
  node, not the node created by each plugin. The issue is that,
  once the node name is provided via launch file (that is,
  embedded into the command line parameters), this is inevitably
  propagated downstream to all the node sharing the same context
  as the main one (that is, all the plugin nodes). Here, we are
  proposing a solution to fix this aspect, taking inspiration
  from the yolo_ros_onnx package.
* adding default plugins parameter to default config
* Fixed error in handling empty default plugin name
* adding missing dependency
* checking that default plugins are actually available
* Improving add/remove plugin functions
  Previously, the node was crashing when adding a plugin
  that was not available to the class loader. Additionally, the node
  now prints a warn when trying to load a plugin already loade
* Changing initialization order for plugins
  Previously, plugin initialization was happening before map
  initialization. This did not allow using map-related information
  (for instance, reference frame) while initializing the plugin.
* README
* Contributors: Lorenzo Ferrini, lorenzoferrini

0.2.0 (2025-04-07)
------------------
* enabling default plugins
* update README
* making linters happy
* supporting launch and configuration parameters
* wip
* fix typo
* updated README.md
* computing spatial relationships
* update README
* using node namespace for services
* Contributors: Lorenzo Ferrini

0.1.0 (2025-02-24)
------------------
* making linters happy
* renaming
* removed continous reset of the grid
* polish MAINTENANCE_NOTE
* maintenance and ethical statements
* polish README
* updates and anonymization
* exposing multithreading
* basic representation manager implementation
* Initial commit
* Contributors: Lorenzo Ferrini, lorenzoferrini
