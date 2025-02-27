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

#include "remap_manager/remap_manager.hpp"

namespace remap
{
namespace manager
{
RemapManager::RemapManager()
: Node("remap_manager"),
  plugin_loader_("remap_plugin_base", "remap::plugins::PluginBase")
{
  add_plugin_service_ = create_service<remap_msgs::srv::AddPlugin>(
    "~/add_plugin",
    std::bind(
      &RemapManager::addPlugin,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  remove_plugin_service_ = create_service<remap_msgs::srv::RemovePlugin>(
    "~/remove_plugin",
    std::bind(
      &RemapManager::removePlugin,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void RemapManager::addPlugin(
  const std::shared_ptr<remap_msgs::srv::AddPlugin::Request> request,
  std::shared_ptr<remap_msgs::srv::AddPlugin::Response> response)
{
  if (plugins_.find(request->plugin_name) == plugins_.end()) {
    RepPluginPtr plugin = plugin_loader_.createSharedInstance(
      std::string("") + (request->plugin_name.c_str()));
    plugin->setup(shared_from_this(), request->plugin_name, request->threaded);
    plugin->initialize();
    plugins_[request->plugin_name] = plugin;
    response->success = true;
  } else {
    response->success = false;
  }
}

void RemapManager::removePlugin(
  const std::shared_ptr<remap_msgs::srv::RemovePlugin::Request> request,
  std::shared_ptr<remap_msgs::srv::RemovePlugin::Response> response)
{
  if (plugins_.find(request->plugin_name) != plugins_.end()) {
    plugins_.erase(request->plugin_name);
    response->success = true;
  } else {
    response->success = false;
  }
}
}          // namespace manager
}  // namespace remap
