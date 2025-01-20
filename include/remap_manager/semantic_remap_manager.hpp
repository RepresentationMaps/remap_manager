#include "remap_manager/remap_manager.hpp"
#include <vdb2pc/vdb2pc_publisher.hpp>

typedef std::shared_ptr<remap::plugins::SemanticPlugin> SemanticPluginPtr;

namespace remap{
	namespace manager{
		class SemanticRemapManager : public RemapManager{
			bool threaded_;
			float voxel_size_;
			bool vertex_centered_;

			std::shared_ptr<remap::map_handler::SemanticMapHandler> semantic_map_;
			std::shared_ptr<remap::plugins::RegionsRegister> regions_register_;

			std::map<std::string, SemanticPluginPtr> semantic_plugins_;

			std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> map_publisher_;

			rclcpp::TimerBase::SharedPtr timer_;
			int period_ms_;

			protected:
				void addPlugin(
					const std::shared_ptr<remap_manager::srv::AddPlugin::Request> request,
					std::shared_ptr<remap_manager::srv::AddPlugin::Response> response) override;
				void removePlugin(
					const std::shared_ptr<remap_manager::srv::RemovePlugin::Request> request,
					std::shared_ptr<remap_manager::srv::RemovePlugin::Response> response) override;

				virtual void run();
			public:
				SemanticRemapManager(
					const bool & threaded, const float & voxel_size_, const bool & vertex_centered);
				void initialize();
		};
	}  // namespace manager
}  // namespace remap