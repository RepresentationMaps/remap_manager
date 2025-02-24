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

#ifndef REMAP_MANAGER__REMAP_MANAGER_HPP_
#define REMAP_MANAGER__REMAP_MANAGER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>

#include <pluginlib/class_loader.hpp>

#include "remap_msgs/srv/add_plugin.hpp"
#include "remap_msgs/srv/remove_plugin.hpp"

typedef std::shared_ptr<remap::plugins::PluginBase> RepPluginPtr;

namespace remap
{
namespace manager
{
class RemapManager : public rclcpp::Node
{
protected:
  rclcpp::Service<remap_msgs::srv::AddPlugin>::SharedPtr add_plugin_service_;
  rclcpp::Service<remap_msgs::srv::RemovePlugin>::SharedPtr remove_plugin_service_;

  pluginlib::ClassLoader<remap::plugins::PluginBase> plugin_loader_;
  std::map<std::string, RepPluginPtr> plugins_;

  virtual void addPlugin(
    const std::shared_ptr<remap_msgs::srv::AddPlugin::Request> request,
    std::shared_ptr<remap_msgs::srv::AddPlugin::Response> response);
  virtual void removePlugin(
    const std::shared_ptr<remap_msgs::srv::RemovePlugin::Request> request,
    std::shared_ptr<remap_msgs::srv::RemovePlugin::Response> response);

public:
  RemapManager();
};
}  // namespace manager
}  // namespace remap
#endif  // REMAP_MANAGER__REMAP_MANAGER_HPP_
