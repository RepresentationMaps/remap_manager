#include "representation_manager/semantic_representation_manager.hpp"

namespace representation_manager{
	SemanticRepresentationManager::SemanticRepresentationManager(
		const bool & threaded, const float & voxel_size, const bool & vertex_centered):
		RepresentationManager(),
		threaded_(threaded), voxel_size_(voxel_size), vertex_centered_(vertex_centered)
	{
		semantic_map_ = std::make_shared<map_handler::SemanticMapHandler>(
			threaded_, voxel_size_, vertex_centered_);
		
		regions_register_ = std::make_shared<representation_plugins::RegionsRegister>(threaded_);
		
		timer_ = create_wall_timer(
			std::chrono::milliseconds(50),
			std::bind(&SemanticRepresentationManager::run, this));
	}

	void SemanticRepresentationManager::addPlugin(
		const std::shared_ptr<representation_manager::srv::AddPlugin::Request> request,
		std::shared_ptr<representation_manager::srv::AddPlugin::Response> response)
	{
		if(semantic_plugins_.find(request->plugin_name) == semantic_plugins_.end()){
			RepPluginPtr plugin = plugin_loader_.createSharedInstance(std::string("")+(request->plugin_name.c_str()));
			SemanticPluginPtr semantic_plugin = std::dynamic_pointer_cast<representation_plugins::SemanticPlugin>(plugin);
			semantic_plugin->setup(shared_from_this(), request->plugin_name, request->threaded);
			semantic_plugin->initializeSimulationStructures();
			semantic_plugin->initialize();
			semantic_plugin->setSemanticMapHandler(semantic_map_);
			semantic_plugin->setRegionsRegister(regions_register_);
			semantic_plugins_[request->plugin_name] = semantic_plugin;
			response->success = true;
		}
		else{
			response->success = false;
		}
	}

	void SemanticRepresentationManager::removePlugin(
		const std::shared_ptr<representation_manager::srv::RemovePlugin::Request> request,
		std::shared_ptr<representation_manager::srv::RemovePlugin::Response> response)
	{
		if(semantic_plugins_.find(request->plugin_name) != semantic_plugins_.end()){
			semantic_plugins_.erase(request->plugin_name);
			response->success = true;
		}
		else{
			response->success = false;
		}
	}

	void SemanticRepresentationManager::run(){
		for (auto & semantic_plugin: semantic_plugins_){
			semantic_plugin.second->run();
		}

		map_publisher_->publish(*(semantic_map_->getGridPtr()));

		auto instances = regions_register_->getInstances();
		for (const auto & instance : instances){
			// semantic_map_->removeRegion(instance, *regions_register_);
		}
		// regions_register_->clear();
	}

	void SemanticRepresentationManager::initialize(){
		map_publisher_ = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>>(
			shared_from_this(),
			"/remap/test_grid",
			"map");
	}
}