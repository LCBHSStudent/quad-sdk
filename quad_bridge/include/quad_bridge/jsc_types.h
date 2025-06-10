#pragma once

#include <array>
#include <cstddef>

#include "quad_bridge/jsc_constants.h"

namespace jsc {

struct JointStateValue {
  float q = 0.0;    // position (rad)
  float dq = 0.0;   // velocity (rad/s)
  float ddq = 0.0;  // acceleration (rad/s^2)
  float tau = 0.0;  // torque (N·m)
  float kp = 0.0;   // position gain
  float kd = 0.0;   // velocity gain

  JointStateValue() = default;
};

// Type aliases for common data structures
using JointDataArray = std::array<JointStateValue, kJointCount>;

/**
 * @brief Joint configuration
 * @param target_index Target system joint index
 * @param sign Sign multiplier (+1.0 or -1.0)
 * @param offset Offset to add (from quad-sdk to target system)
 */
struct JointConfig {
  int target_index;  // Target system joint index
  double sign;       // Sign multiplier (+1.0 or -1.0)
  double offset;     // Offset to add/subtract

  JointConfig() : target_index(0), sign(1.0), offset(0.0) {}
  JointConfig(int idx, double s = 1.0, double off = 0.0)
      : target_index(idx), sign(s), offset(off) {}
};

/**
 * @brief Intermediate joint state representation
 *
 * Allows access to joint values using LegJoint enum
 */
class IntermediateJointState {
 public:
  IntermediateJointState() = default;

  // Access by LegJoint enum
  JointStateValue& operator[](LegJoint joint);
  const JointStateValue& operator[](LegJoint joint) const;

  // Access by leg and joint type
  JointStateValue& operator()(Leg leg, Joint joint);
  const JointStateValue& operator()(Leg leg, Joint joint) const;

  IntermediateJointState& operator=(const JointDataArray& quad_joints);

  // Direct array access
  JointDataArray& data();
  const JointDataArray& data() const;

 private:
  JointDataArray values_;
};

}  // namespace jsc
