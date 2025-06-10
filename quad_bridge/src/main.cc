#include <quad_bridge/quad_bridge_node.h>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "quad_bridge_node");

  // Create and initialize the bridge node
  quad_bridge::QuadBridgeNode bridge_node;

  if (!bridge_node.Initialize()) {
    ROS_ERROR("Failed to initialize QuadBridgeNode");
    return -1;
  }

  ROS_INFO("QuadBridgeNode started successfully");

  // Run the node
  bridge_node.Run();

  return 0;
}
