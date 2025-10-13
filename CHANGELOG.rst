^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remap_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
