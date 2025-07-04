// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "remap_manager/semantic_remap_manager.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  bool threaded = false;
  float voxel_size = 0.1;
  bool vertex_centered = false;

  auto node = std::make_shared<remap::manager::SemanticRemapManager>(
    threaded, voxel_size, vertex_centered);

  rclcpp::NodeOptions plugin_node_options;

  std::vector<std::string> node_arguments = plugin_node_options.arguments();
  for (int i = 1; i < argc; ++i) {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args") {
      // A simple way to reject non ros args
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  plugin_node_options.arguments(node_arguments);

  node->setPluginNodeOptions(plugin_node_options);

  std::this_thread::sleep_for(1s);

  node->initialize();

  rclcpp::spin(node);
  return 0;
}
