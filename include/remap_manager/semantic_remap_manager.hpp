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

#ifndef REMAP_MANAGER__SEMANTIC_REMAP_MANAGER_HPP_
#define REMAP_MANAGER__SEMANTIC_REMAP_MANAGER_HPP_

#include <openvdb/openvdb.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <remap_map_handler/semantic_map_handler.hpp>

#include <vdb2pc/vdb2pc_publisher.hpp>

#include "remap_manager/remap_manager.hpp"

typedef std::shared_ptr<remap::plugins::SemanticPlugin> SemanticPluginPtr;

namespace remap
{
namespace manager
{
class SemanticRemapManager : public RemapManager
{
  bool threaded_;
  float voxel_size_;
  bool vertex_centered_;
  std::string fixed_frame_;

  std::shared_ptr<remap::map_handler::SemanticMapHandler> semantic_map_;
  std::shared_ptr<remap::regions_register::RegionsRegister> regions_register_;

  std::map<std::string, SemanticPluginPtr> semantic_plugins_;

  std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> map_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  int period_ms_;

  std::vector<std::string> default_plugins_;

  void instatiateNewPlugin(
    const std::string & plugin_name,
    const bool & threaded);

protected:
  void addPlugin(
    const std::shared_ptr<remap_msgs::srv::AddPlugin::Request> request,
    std::shared_ptr<remap_msgs::srv::AddPlugin::Response> response) override;
  void removePlugin(
    const std::shared_ptr<remap_msgs::srv::RemovePlugin::Request> request,
    std::shared_ptr<remap_msgs::srv::RemovePlugin::Response> response) override;

  virtual void run();

public:
  SemanticRemapManager(
    const bool & threaded = false,
    const float & voxel_size_ = 0.1,
    const bool & vertex_centered = true,
    const std::string & fixed_frame = "map");
  void initialize();
};
}  // namespace manager
}  // namespace remap
#endif  // REMAP_MANAGER__SEMANTIC_REMAP_MANAGER_HPP_
