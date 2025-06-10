#pragma once

#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>

// Use our mock unitree types instead of the real SDK
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/OccupancyGrid_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace jsc {
class JointStateConverter;
}  // namespace jsc

namespace quad_bridge {

/**
 * @brief Bridge node that converts ROS messages to Unitree SDK2 format
 */
class QuadBridgeNode {
 public:
  /**
   * @brief Constructor
   */
  QuadBridgeNode();

  /**
   * @brief Destructor
   */
  ~QuadBridgeNode();

  /**
   * @brief Initialize the node
   * @return true if initialization successful
   */
  bool Initialize();

  /**
   * @brief Run the node
   */
  void Run();

 private:
  // ROS node handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ROS subscribers
  ros::Subscriber joint_state_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber terrain_map_sub_;
  ros::Subscriber fl_contact_sub_;
  ros::Subscriber rl_contact_sub_;
  ros::Subscriber fr_contact_sub_;
  ros::Subscriber rr_contact_sub_;

  // Unitree SDK2 publishers
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_pub_;
  unitree::robot::ChannelPublisherPtr<nav_msgs::msg::dds_::OccupancyGrid_> occ_pub_;

  // Data storage
  unitree_go::msg::dds_::LowState_ low_state_;
  nav_msgs::msg::dds_::OccupancyGrid_ occ_map_;

  // Joint state converter
  std::unique_ptr<jsc::JointStateConverter> joint_state_converter_;

  // Parameters
  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string terrain_map_topic_;
  std::string fl_contact_topic_;
  std::string rl_contact_topic_;
  std::string fr_contact_topic_;
  std::string rr_contact_topic_;
  std::string unitree_low_state_topic_;
  std::string unitree_occ_topic_;
  int32_t unitree_domain_id_;
  std::string unitree_network_interface_;

  // Callback functions
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void TerrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
  void FLContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RLContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void FRContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RRContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);

  // Helper functions
  bool LoadParameters();
  bool InitializeUnitreeSDK();
  void PublishLowState();
  void PublishOccupancyMap();
};

}  // namespace quad_bridge
