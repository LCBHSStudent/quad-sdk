#pragma once

#include <quad_msgs/LegCommandArray.h>
#include <sensor_msgs/JointState.h>

#include "quad_bridge/jsc_constants.h"
#include "quad_bridge/jsc_types.h"

namespace jsc {

/**
 * @brief Joint state converter for transforming between different quadruped robot systems
 *
 * This class provides conversion between quad-sdk joint representations and other
 * robot systems with support for:
 * - Joint index mapping
 * - Sign inversion for coordinate system differences
 * - Offset adjustments
 */
class JointStateConverter {
 public:
  /**
   * @brief Constructor with joint configuration mapping
   * @param joint_configs Configuration for each joint (indexed by LegJoint enum)
   */
  explicit JointStateConverter(const std::array<JointConfig, kJointCount>& joint_configs);

  /**
   * @brief Default constructor using identity mapping
   */
  JointStateConverter();

  /**
   * @brief Set the joint configuration for a given joint
   * @param joint The joint to set the configuration for
   * @param config The configuration to set
   */
  void setJointConfig(LegJoint joint, JointConfig const& config);

  /**
   * @brief Get the joint configuration for a given joint
   * @param joint The joint to get the configuration for
   * @return The configuration for the given joint
   */
  JointConfig getJointConfig(LegJoint joint) const;

  // Convert from target system array to intermediate state
  IntermediateJointState fromTargetSystem(const JointDataArray& target_joints) const;

  // Convert from intermediate state to target system array
  JointDataArray toTargetSystem(const IntermediateJointState& intermediate) const;

  // Convert from quad_msgs::JointState to target system array
  JointDataArray fromJointState(const sensor_msgs::JointState& joint_state) const;

  // Convert from quad_msgs::LegCommandArray to target system array
  JointDataArray fromLegCommandArray(const quad_msgs::LegCommandArray& leg_command_array) const;

  // Convert from target system array to quad_msgs::JointState
  sensor_msgs::JointState toJointState(const JointDataArray& target_joints) const;

  // Convert from target system array to quad_msgs::LegCommandArray
  quad_msgs::LegCommandArray toLegCommandArray(const JointDataArray& target_joints) const;

 private:
  std::array<JointConfig, kJointCount> joint_configs_;

  // Helper method to get quad-sdk joint index from LegJoint enum
  static int getQuadSdkIndex(LegJoint joint);
};

}  // namespace jsc
