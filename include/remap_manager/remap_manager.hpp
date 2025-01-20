#include <vector>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include "remap_plugin_base/plugin_base.hpp"
#include "remap_plugins/semantic_plugin.hpp"

#include <pluginlib/class_loader.hpp>

#include "remap_manager/srv/add_plugin.hpp"
#include "remap_manager/srv/remove_plugin.hpp"

typedef std::shared_ptr<remap::plugins::PluginBase> RepPluginPtr;

namespace remap {
	namespace manager {
		class RemapManager : public rclcpp::Node{
			protected:
				rclcpp::Service<remap_manager::srv::AddPlugin>::SharedPtr add_plugin_service_;
				rclcpp::Service<remap_manager::srv::RemovePlugin>::SharedPtr remove_plugin_service_;

				pluginlib::ClassLoader<remap::plugins::PluginBase> plugin_loader_;
				std::map<std::string, RepPluginPtr> plugins_;

				virtual void addPlugin(
					const std::shared_ptr<remap_manager::srv::AddPlugin::Request> request,
					std::shared_ptr<remap_manager::srv::AddPlugin::Response> response);
				virtual void removePlugin(
					const std::shared_ptr<remap_manager::srv::RemovePlugin::Request> request,
					std::shared_ptr<remap_manager::srv::RemovePlugin::Response> response);
			public:
				RemapManager();
		};
	}  // namespace manager
}  // namespace remap