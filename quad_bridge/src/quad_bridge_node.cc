#include "quad_bridge/quad_bridge_node.h"

#include <ros/ros.h>

#include <chrono>
#include <cstring>

#include "quad_bridge/joint_state_converter.h"

namespace quad_bridge {

QuadBridgeNode::QuadBridgeNode() : private_nh_("~") {
  // Initialize low state message
  memset(&low_state_, 0, sizeof(low_state_));
}

QuadBridgeNode::~QuadBridgeNode() {
  // Cleanup if needed
}

bool QuadBridgeNode::Initialize() {
  // Load parameters from ROS parameter server
  if (!LoadParameters()) {
    ROS_ERROR("Failed to load parameters");
    return false;
  }

  // Initialize Unitree SDK2
  if (!InitializeUnitreeSDK()) {
    ROS_ERROR("Failed to initialize Unitree SDK2");
    return false;
  }

  // Initialize joint state converter
  {
    using namespace jsc;
    constexpr float kPi = 3.1415926;
    joint_state_converter_ = std::make_unique<JointStateConverter>();
    joint_state_converter_->setJointConfig(LegJoint::FL_thigh, JointConfig(4, -1.0, +kPi / 2));
    joint_state_converter_->setJointConfig(LegJoint::FL_calf, JointConfig(5, 1.0, -2.8));
    joint_state_converter_->setJointConfig(LegJoint::RL_thigh, JointConfig(10, -1.0, +kPi / 2));
    joint_state_converter_->setJointConfig(LegJoint::RL_calf, JointConfig(11, 1.0, -2.8));
    joint_state_converter_->setJointConfig(LegJoint::FR_thigh, JointConfig(1, -1.0, +kPi / 2));
    joint_state_converter_->setJointConfig(LegJoint::FR_calf, JointConfig(2, 1.0, -2.8));
    joint_state_converter_->setJointConfig(LegJoint::RR_thigh, JointConfig(7, -1.0, +kPi / 2));
    joint_state_converter_->setJointConfig(LegJoint::RR_calf, JointConfig(8, 1.0, -2.8));
    joint_state_converter_->setJointConfig(LegJoint::FL_hip, JointConfig(3));
    joint_state_converter_->setJointConfig(LegJoint::RL_hip, JointConfig(9));
    joint_state_converter_->setJointConfig(LegJoint::FR_hip, JointConfig(0));
    joint_state_converter_->setJointConfig(LegJoint::RR_hip, JointConfig(6));
  }

  // Create ROS subscribers based on parameters
  joint_state_sub_ =
      nh_.subscribe(joint_state_topic_, 1, &QuadBridgeNode::JointStateCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic_, 1, &QuadBridgeNode::ImuCallback, this);
  terrain_map_sub_ =
      nh_.subscribe(terrain_map_topic_, 1, &QuadBridgeNode::TerrainMapCallback, this);
  fl_contact_sub_ = nh_.subscribe(fl_contact_topic_, 1, &QuadBridgeNode::FLContactCallback, this);
  rl_contact_sub_ = nh_.subscribe(rl_contact_topic_, 1, &QuadBridgeNode::RLContactCallback, this);
  fr_contact_sub_ = nh_.subscribe(fr_contact_topic_, 1, &QuadBridgeNode::FRContactCallback, this);
  rr_contact_sub_ = nh_.subscribe(rr_contact_topic_, 1, &QuadBridgeNode::RRContactCallback, this);

  ROS_INFO("QuadBridgeNode initialized successfully");
  ROS_INFO("Subscribing to:");
  ROS_INFO("  Joint states: %s", joint_state_topic_.c_str());
  ROS_INFO("  IMU: %s", imu_topic_.c_str());
  ROS_INFO("  Terrain map: %s", terrain_map_topic_.c_str());
  ROS_INFO("  FL contact: %s", fl_contact_topic_.c_str());
  ROS_INFO("  RL contact: %s", rl_contact_topic_.c_str());
  ROS_INFO("  FR contact: %s", fr_contact_topic_.c_str());
  ROS_INFO("  RR contact: %s", rr_contact_topic_.c_str());
  ROS_INFO("Publishing to Unitree low-level state channel: %s", unitree_low_state_topic_.c_str());
  ROS_INFO("Publishing to Unitree occupancy map channel: %s", unitree_occ_topic_.c_str());

  return true;
}

void QuadBridgeNode::Run() {
  ros::Rate rate(100);  // 100 Hz

  while (ros::ok()) {
    ros::spinOnce();

    // Publish low state at regular intervals
    PublishLowState();

    rate.sleep();
  }
}

bool QuadBridgeNode::LoadParameters() {
  // Load input topic names from parameters
  private_nh_.param<std::string>("joint_state_topic", joint_state_topic_, "");
  private_nh_.param<std::string>("imu_topic", imu_topic_, "");
  private_nh_.param<std::string>("terrain_map_topic", terrain_map_topic_, "");
  private_nh_.param<std::string>("fl_contact_topic", fl_contact_topic_, "");
  private_nh_.param<std::string>("rl_contact_topic", rl_contact_topic_, "");
  private_nh_.param<std::string>("fr_contact_topic", fr_contact_topic_, "");
  private_nh_.param<std::string>("rr_contact_topic", rr_contact_topic_, "");

  // Load Unitree SDK2 parameters
  private_nh_.param<int>("unitree_domain_id", unitree_domain_id_, 1);
  private_nh_.param<std::string>("unitree_occ_topic", unitree_occ_topic_, "");
  private_nh_.param<std::string>("unitree_low_state_topic", unitree_low_state_topic_, "");
  private_nh_.param<std::string>("unitree_network_interface", unitree_network_interface_, "");

  return true;
}

bool QuadBridgeNode::InitializeUnitreeSDK() {
  try {
    // Initialize Unitree SDK2 channel factory
    unitree::robot::ChannelFactory::Instance()->Init(unitree_domain_id_,
                                                     unitree_network_interface_);

    // Create low state publisher
    low_state_pub_ =
        std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_>>(
            unitree_low_state_topic_);
    occ_pub_ =
        std::make_shared<unitree::robot::ChannelPublisher<nav_msgs::msg::dds_::OccupancyGrid_>>(
            unitree_occ_topic_);

    // Initialize the channel
    low_state_pub_->InitChannel();
    occ_pub_->InitChannel();

    ROS_INFO("Unitree SDK2 initialized with domain_id: %d, interface: %s", unitree_domain_id_,
             unitree_network_interface_.c_str());

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to initialize Unitree SDK2: %s", e.what());
    return false;
  }
}

void QuadBridgeNode::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  auto jd = joint_state_converter_->fromJointState(*msg);

  for (int i = 0; i < static_cast<int>(jd.size()); ++i) {
    low_state_.motor_state()[i].q() = jd[i].q;
    low_state_.motor_state()[i].dq() = jd[i].dq;
    low_state_.motor_state()[i].ddq() = jd[i].ddq;
    low_state_.motor_state()[i].tau_est() = jd[i].tau;
  }
}

void QuadBridgeNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // Map IMU data to low state message
  low_state_.imu_state().quaternion()[0] = msg->orientation.w;
  low_state_.imu_state().quaternion()[1] = msg->orientation.x;
  low_state_.imu_state().quaternion()[2] = msg->orientation.y;
  low_state_.imu_state().quaternion()[3] = msg->orientation.z;

  double w = low_state_.imu_state().quaternion()[0];
  double x = low_state_.imu_state().quaternion()[1];
  double y = low_state_.imu_state().quaternion()[2];
  double z = low_state_.imu_state().quaternion()[3];

  low_state_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  low_state_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
  low_state_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  low_state_.imu_state().gyroscope()[0] = msg->angular_velocity.x;
  low_state_.imu_state().gyroscope()[1] = msg->angular_velocity.y;
  low_state_.imu_state().gyroscope()[2] = msg->angular_velocity.z;

  low_state_.imu_state().accelerometer()[0] = msg->linear_acceleration.x;
  low_state_.imu_state().accelerometer()[1] = msg->linear_acceleration.y;
  low_state_.imu_state().accelerometer()[2] = msg->linear_acceleration.z;
}

void QuadBridgeNode::TerrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);
}

void QuadBridgeNode::FLContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
  // Map contact state for Front Left foot
  low_state_.foot_force()[0] = msg->states.empty() ? 0 : 1;
}

void QuadBridgeNode::RLContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
  // Map contact state for Rear Left foot
  low_state_.foot_force()[1] = msg->states.empty() ? 0 : 1;
}

void QuadBridgeNode::FRContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
  // Map contact state for Front Right foot
  low_state_.foot_force()[2] = msg->states.empty() ? 0 : 1;
}

void QuadBridgeNode::RRContactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
  // Map contact state for Rear Right foot
  low_state_.foot_force()[3] = msg->states.empty() ? 0 : 1;
}

void QuadBridgeNode::PublishLowState() {
  if (low_state_pub_) {
    low_state_pub_->Write(low_state_);
  }
}

void QuadBridgeNode::PublishOccupancyMap() {
  if (occ_pub_) {
    occ_pub_->Write(occ_map_);
  }
}

}  // namespace quad_bridge
