#include "quad_bridge/joint_state_converter.h"

namespace jsc {

// Constructor with joint configuration mapping
JointStateConverter::JointStateConverter(const std::array<JointConfig, kJointCount>& joint_configs)
    : joint_configs_(joint_configs) {}

// Default constructor using identity mapping
JointStateConverter::JointStateConverter() {
  // Initialize with identity mapping (index in target system)
  for (int i = 0; i < kJointCount; ++i) {
    joint_configs_[i] = JointConfig(i, 1.0, 0.0);
  }
}

int JointStateConverter::getQuadSdkIndex(LegJoint joint) {
  return kJointToIndex[static_cast<int>(joint)];
}

void JointStateConverter::setJointConfig(LegJoint joint, JointConfig const& config) {
  joint_configs_[static_cast<int>(joint)] = config;
}

JointConfig JointStateConverter::getJointConfig(LegJoint joint) const {
  return joint_configs_[static_cast<int>(joint)];
}

// Convert from target system array to intermediate state
IntermediateJointState JointStateConverter::fromTargetSystem(
    const JointDataArray& target_joints) const {
  IntermediateJointState intermediate;

  // Apply inverse transformation: (target_value - offset) / sign
  for (int i = 0; i < kJointCount; ++i) {
    LegJoint joint = kIndexToJoint[i];

    const JointConfig& config = joint_configs_[i];
    auto& joint_state = intermediate[joint];

    joint_state.q = (target_joints[config.target_index].q - config.offset) * config.sign;
    joint_state.dq = target_joints[config.target_index].dq * config.sign;
    joint_state.tau = target_joints[config.target_index].tau * config.sign;
    joint_state.kp = target_joints[config.target_index].kp;
    joint_state.kd = target_joints[config.target_index].kd;
  }

  return intermediate;
}

// Convert from intermediate state to target system array
JointDataArray JointStateConverter::toTargetSystem(
    const IntermediateJointState& intermediate) const {
  JointDataArray target_joints{};

  // Apply transformation: sign * intermediate_value + offset
  for (int i = 0; i < kJointCount; ++i) {
    LegJoint joint = kIndexToJoint[i];
    const JointConfig& config = joint_configs_[i];

    const auto& joint_state = intermediate[joint];
    target_joints[config.target_index].q = joint_state.q * config.sign + config.offset;
    target_joints[config.target_index].dq = joint_state.dq * config.sign;
    target_joints[config.target_index].tau = joint_state.tau * config.sign;
    target_joints[config.target_index].kp = joint_state.kp;
    target_joints[config.target_index].kd = joint_state.kd;
  }

  return target_joints;
}

// Convert from quad_msgs::LegCommandArray to target system array
JointDataArray JointStateConverter::fromLegCommandArray(
    const quad_msgs::LegCommandArray& leg_commands) const {
  IntermediateJointState intermediate;

  // Convert LegCommandArray to intermediate state
  for (int i = 0; i < kLegCount; ++i) {
    for (int j = 0; j < kJointPerLeg; ++j) {
      auto [leg, joint] = kJointToLegAndJointIdx[i * kJointPerLeg + j];
      auto leg_idx = static_cast<int>(leg);
      auto joint_idx = static_cast<int>(joint);

      const auto& motor_cmd = leg_commands.leg_commands[leg_idx].motor_commands[joint_idx];
      auto& joint_state = intermediate[kLegAndJointIdxToJoint[leg_idx][joint_idx]];

      joint_state.q = motor_cmd.pos_setpoint;
      joint_state.dq = motor_cmd.vel_setpoint;
      joint_state.tau = motor_cmd.torque_ff;
      joint_state.kp = motor_cmd.kp;
      joint_state.kd = motor_cmd.kd;
    }
  }

  return toTargetSystem(intermediate);
}

// Convert from quad_msgs::JointState to target system array
JointDataArray JointStateConverter::fromJointState(
    const sensor_msgs::JointState& joint_state) const {
  IntermediateJointState intermediate;

  // Map joint state data to intermediate representation
  for (int i = 0; i < static_cast<int>(joint_state.position.size()) && i < kJointCount; ++i) {
    LegJoint joint = kIndexToJoint[i];
    auto joint_idx = getQuadSdkIndex(joint);
    auto& joint_value = intermediate[joint];

    joint_value.q = joint_state.position[joint_idx];
    if (joint_idx < static_cast<int>(joint_state.velocity.size())) {
      joint_value.dq = joint_state.velocity[joint_idx];
    }
    if (joint_idx < static_cast<int>(joint_state.effort.size())) {
      joint_value.tau = joint_state.effort[joint_idx];
    }
  }

  return toTargetSystem(intermediate);
}

// Convert from target system array to quad_msgs::JointState
sensor_msgs::JointState JointStateConverter::toJointState(
    const JointDataArray& target_joints) const {
  IntermediateJointState intermediate = fromTargetSystem(target_joints);
  sensor_msgs::JointState joint_state;

  joint_state.position.resize(kJointCount);
  joint_state.velocity.resize(kJointCount);
  joint_state.effort.resize(kJointCount);

  for (int i = 0; i < kJointCount; ++i) {
    LegJoint joint = kIndexToJoint[i];
    int quad_idx = getQuadSdkIndex(joint);
    const auto& joint_value = intermediate[joint];

    joint_state.name[quad_idx] = std::to_string(static_cast<int>(joint));
    joint_state.position[quad_idx] = joint_value.q;
    joint_state.velocity[quad_idx] = joint_value.dq;
    joint_state.effort[quad_idx] = joint_value.tau;
  }

  return joint_state;
}

// Convert from target system array to quad_msgs::LegCommandArray
quad_msgs::LegCommandArray JointStateConverter::toLegCommandArray(
    const JointDataArray& target_joints) const {
  IntermediateJointState intermediate = fromTargetSystem(target_joints);
  quad_msgs::LegCommandArray leg_commands;

  // Initialize the leg commands array
  leg_commands.leg_commands.resize(kLegCount);
  for (int i = 0; i < kLegCount; ++i) {
    leg_commands.leg_commands[i].motor_commands.resize(kJointPerLeg);
  }

  // Convert intermediate state to LegCommandArray
  for (int i = 0; i < kLegCount; ++i) {
    for (int j = 0; j < kJointPerLeg; ++j) {
      auto [leg, joint] = kJointToLegAndJointIdx[i * kJointPerLeg + j];
      auto leg_idx = static_cast<int>(leg);
      auto joint_idx = static_cast<int>(joint);

      const auto& joint_state = intermediate[kLegAndJointIdxToJoint[leg_idx][joint_idx]];
      auto& motor_cmd = leg_commands.leg_commands[leg_idx].motor_commands[joint_idx];

      motor_cmd.pos_setpoint = joint_state.q;
      motor_cmd.vel_setpoint = joint_state.dq;
      motor_cmd.torque_ff = joint_state.tau;
      motor_cmd.kp = joint_state.kp;
      motor_cmd.kd = joint_state.kd;
    }
  }

  return leg_commands;
}

}  // namespace jsc
