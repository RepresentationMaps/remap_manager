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

#include <openvdb/tools/Prune.h>

#include <vdb_ros/conversions.hpp>

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

  descriptor.description = "Plugins run at startup";
  this->declare_parameter("default_plugins", std::vector<std::string>(), descriptor);

  descriptor.description = "Rate of the compressed grid (vdb_msgs/VdbGrid) on /remap/world/vdb";
  this->declare_parameter("vdb_rate_hz", 2.0, descriptor);
  descriptor.description = "Compression of the VdbGrid: blosc, zip or none";
  this->declare_parameter("vdb_compression", std::string("blosc"), descriptor);

  threaded_ = threaded;
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  vertex_centered_ = this->get_parameter("vertex_centered").as_bool();
  fixed_frame_ = this->get_parameter("fixed_frame").as_string();
  default_plugins_ = this->get_parameter("default_plugins").as_string_array();
  vdb_rate_hz_ = this->get_parameter("vdb_rate_hz").as_double();
  vdb_compression_ = this->get_parameter("vdb_compression").as_string();

  semantic_map_ = std::make_shared<map_handler::SemanticMapHandler>(
    threaded_, voxel_size_, vertex_centered_, fixed_frame_);

  regions_register_ = std::make_shared<remap::regions_register::RegionsRegister>(threaded_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SemanticRemapManager::run, this));
}

void SemanticRemapManager::instatiateNewPlugin(
  const std::string & plugin_name,
  const bool & threaded)
{
  RCLCPP_INFO(get_logger(), "Adding plugin %s", plugin_name.c_str());
  RepPluginPtr plugin =
    plugin_loader_.createSharedInstance(std::string("") + (plugin_name.c_str()));
  SemanticPluginPtr semantic_plugin =
    std::dynamic_pointer_cast<remap::plugins::SemanticPlugin>(plugin);
  semantic_plugin->setPluginNodeOptions(plugin_node_options_);
  semantic_plugin->setup(shared_from_this(), plugin_name, threaded);
  semantic_plugin->initializeSimulationStructures();
  semantic_plugin->setSemanticMapHandler(semantic_map_);
  semantic_plugin->setRegionsRegister(regions_register_);
  semantic_plugin->initialize();
  semantic_plugins_[plugin_name] = semantic_plugin;
  RCLCPP_INFO(get_logger(), "Added plugin %s", plugin_name.c_str());
}

void SemanticRemapManager::addPlugin(
  const std::shared_ptr<remap_msgs::srv::AddPlugin::Request> request,
  std::shared_ptr<remap_msgs::srv::AddPlugin::Response> response)
{
  if (semantic_plugins_.find(request->plugin_name) == semantic_plugins_.end()) {
    if (!plugin_loader_.isClassAvailable(request->plugin_name)) {
      RCLCPP_ERROR(
        get_logger(), "Plugin %s not found in available classes", request->plugin_name.c_str());
      response->success = false;
      return;
    }
    instatiateNewPlugin(request->plugin_name, request->threaded);
    response->success = true;
  } else {
    RCLCPP_WARN(
      get_logger(), "Plugin %s already uploaded", request->plugin_name.c_str());
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

  for (auto & semantic_plugin : semantic_plugins_) {
    semantic_plugin.second->storeRegionsRelationships(
      semantic_map_->getRegionsRelationshipMatrix());
    semantic_plugin.second->storeEntitiesRelationships(
      semantic_map_->getEntitiesRelationshipMatrix());
  }

  map_publisher_->publish(*(semantic_map_->getGridPtr()));
  snapshotVdbGrid();

  auto instances = regions_register_->getInstances();
}

SemanticRemapManager::~SemanticRemapManager()
{
  {
    std::lock_guard<std::mutex> lock(vdb_mutex_);
    vdb_stop_ = true;
  }
  vdb_cv_.notify_all();
  if (vdb_worker_.joinable()) {
    vdb_worker_.join();
  }
}

void SemanticRemapManager::snapshotVdbGrid()
{
  if (!vdb_publisher_ || vdb_rate_hz_ <= 0.0 ||
    vdb_publisher_->get_subscription_count() == 0)
  {
    return;
  }
  const rclcpp::Time now = this->now();
  if (last_vdb_publish_.nanoseconds() > 0 &&
    (now - last_vdb_publish_).seconds() < 1.0 / vdb_rate_hz_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(vdb_mutex_);
  if (vdb_busy_) {
    return;  // the previous grid is still being serialised: skip this one
  }
  // Cheap in the cycle (copy-on-write leaves); pruning and compression run in
  // the worker thread.
  vdb_snapshot_ = semantic_map_->getGridPtr()->deepCopy();
  vdb_legend_.clear();
  for (const auto & [entities, id] : regions_register_->getAreas()) {
    vdb_legend_[id] = entities;
  }
  vdb_busy_ = true;
  last_vdb_publish_ = now;
  vdb_cv_.notify_one();
}

void SemanticRemapManager::vdbWorker()
{
  while (true) {
    openvdb::Int32Grid::Ptr grid;
    std::map<int64_t, std::vector<std::string>> legend;
    {
      std::unique_lock<std::mutex> lock(vdb_mutex_);
      vdb_cv_.wait(lock, [this] {return vdb_stop_ || vdb_busy_;});
      if (vdb_stop_) {
        return;
      }
      grid = vdb_snapshot_;
      legend.swap(vdb_legend_);
    }
    try {
      openvdb::tools::prune(grid->tree());
      grid->setName("remap_world");
      // only the legend entries of values present in the grid
      std::map<int64_t, std::vector<std::string>> used;
      for (auto it = grid->cbeginValueOn(); it; ++it) {
        const auto l = legend.find(*it);
        if (l != legend.end()) {
          used.insert(*l);
        }
        if (used.size() == legend.size()) {
          break;
        }
      }
      vdb_msgs::msg::VdbGrid msg;
      vdb_ros::toMsg(grid, msg, vdb_compression_);
      vdb_ros::setLabels(msg, used);
      msg.header.stamp = this->now();
      msg.header.frame_id = fixed_frame_;
      msg.sequence = vdb_sequence_++;
      vdb_publisher_->publish(msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "VdbGrid publication failed: %s", e.what());
    }
    std::lock_guard<std::mutex> lock(vdb_mutex_);
    vdb_snapshot_.reset();
    vdb_busy_ = false;
  }
}

void SemanticRemapManager::initialize()
{
  // latched: a display started later gets the last grid at once
  vdb_publisher_ = this->create_publisher<vdb_msgs::msg::VdbGrid>(
    "/remap/world/vdb", rclcpp::QoS(1).reliable().transient_local());
  vdb_worker_ = std::thread(&SemanticRemapManager::vdbWorker, this);

  map_publisher_ = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>>(
    shared_from_this(),
    "/remap/world",
    fixed_frame_);

  for (auto & plugin_name : default_plugins_) {
    bool plugin_available = plugin_loader_.isClassAvailable(plugin_name);
    bool plugin_not_loaded = semantic_plugins_.find(plugin_name) == semantic_plugins_.end();
    if (!plugin_name.empty() && plugin_available && plugin_not_loaded) {
      instatiateNewPlugin(plugin_name, threaded_);
    } else if (!plugin_name.empty() && !plugin_available) {
      RCLCPP_ERROR(
        get_logger(), "Default plugin %s not found in available classes", plugin_name.c_str());
    } else if (!plugin_name.empty() && !plugin_not_loaded) {
      RCLCPP_WARN(
        get_logger(), "Default plugin %s already uploaded", plugin_name.c_str());
    }
  }
}
}  // namespace manager
}  // namespace remap
