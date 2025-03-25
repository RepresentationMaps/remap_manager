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

#include "remap_manager/semantic_remap_manager.hpp"

namespace remap
{
namespace manager
{
SemanticRemapManager::SemanticRemapManager(
  const bool & threaded,
  const float & voxel_size,
  const bool & vertex_centered,
  const std::string & fixed_frame)
: RemapManager()
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};

  descriptor.description = "VDB grid voxel size";
  this->declare_parameter("voxel_size", voxel_size, descriptor);

  descriptor.description = "VDB voxel centering";
  this->declare_parameter("vertex_centered", vertex_centered, descriptor);

  descriptor.description = "Semantic map refernce frame";
  this->declare_parameter("fixed_frame", fixed_frame, descriptor);

  threaded_ = threaded;
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  vertex_centered_ = this->get_parameter("vertex_centered").as_bool();
  fixed_frame_ = this->get_parameter("fixed_frame").as_string();

  semantic_map_ = std::make_shared<map_handler::SemanticMapHandler>(
    threaded_, voxel_size_, vertex_centered_, fixed_frame_);

  regions_register_ = std::make_shared<remap::regions_register::RegionsRegister>(threaded_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SemanticRemapManager::run, this));
}

void SemanticRemapManager::addPlugin(
  const std::shared_ptr<remap_msgs::srv::AddPlugin::Request> request,
  std::shared_ptr<remap_msgs::srv::AddPlugin::Response> response)
{
  if (semantic_plugins_.find(request->plugin_name) == semantic_plugins_.end()) {
    RepPluginPtr plugin =
      plugin_loader_.createSharedInstance(std::string("") + (request->plugin_name.c_str()));
    SemanticPluginPtr semantic_plugin = std::dynamic_pointer_cast<remap::plugins::SemanticPlugin>(
      plugin);
    semantic_plugin->setup(shared_from_this(), request->plugin_name, request->threaded);
    semantic_plugin->initializeSimulationStructures();
    semantic_plugin->initialize();
    semantic_plugin->setSemanticMapHandler(semantic_map_);
    semantic_plugin->setRegionsRegister(regions_register_);
    semantic_plugins_[request->plugin_name] = semantic_plugin;
    response->success = true;
  } else {
    response->success = false;
  }
}

void SemanticRemapManager::removePlugin(
  const std::shared_ptr<remap_msgs::srv::RemovePlugin::Request> request,
  std::shared_ptr<remap_msgs::srv::RemovePlugin::Response> response)
{
  if (semantic_plugins_.find(request->plugin_name) != semantic_plugins_.end()) {
    semantic_plugins_.erase(request->plugin_name);
    response->success = true;
  } else {
    response->success = false;
  }
}

void SemanticRemapManager::run()
{
  for (auto & semantic_plugin : semantic_plugins_) {
    semantic_plugin.second->run();
  }

  // Compute spatial relationships
  semantic_map_->processRelationships(regions_register_);

  map_publisher_->publish(*(semantic_map_->getGridPtr()));

  auto instances = regions_register_->getInstances();
}

void SemanticRemapManager::initialize()
{
  map_publisher_ = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>>(
    shared_from_this(),
    "/remap/test_grid",
    "map");
}
}  // namespace manager
}  // namespace remap
