#include "quad_bridge/jsc_types.h"

namespace jsc {

JointStateValue& IntermediateJointState::operator[](LegJoint joint) {
  return values_[static_cast<int>(joint)];
}

const JointStateValue& IntermediateJointState::operator[](LegJoint joint) const {
  return values_[static_cast<int>(joint)];
}

JointStateValue& IntermediateJointState::operator()(Leg leg, Joint joint) {
  return values_[static_cast<int>(leg) * kJointPerLeg + static_cast<int>(joint)];
}

const JointStateValue& IntermediateJointState::operator()(Leg leg, Joint joint) const {
  return values_[static_cast<int>(leg) * kJointPerLeg + static_cast<int>(joint)];
}

IntermediateJointState& IntermediateJointState::operator=(const JointDataArray& quad_joints) {
  values_ = quad_joints;
  return *this;
}

JointDataArray& IntermediateJointState::data() { return values_; }

const JointDataArray& IntermediateJointState::data() const { return values_; }

}  // namespace jsc
