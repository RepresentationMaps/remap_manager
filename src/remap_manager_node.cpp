#include <rclcpp/rclcpp.hpp>

#include "remap_manager/semantic_remap_manager.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	// Testing
	bool threaded = false;
	float voxel_size = 0.1;
	bool vertex_centered = false;

	auto node = std::make_shared<remap::manager::SemanticRemapManager>(
			threaded, voxel_size, vertex_centered);

	node->initialize();

	rclcpp::spin(node);
	return 0;
}
