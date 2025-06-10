#pragma once

#include <array>
#include <cstdint>

namespace jsc {

enum class LegJoint : int {
  FL_thigh = 0,
  FL_calf,
  RL_thigh,
  RL_calf,
  FR_thigh,
  FR_calf,
  RR_thigh,
  RR_calf,
  FL_hip,
  RL_hip,
  FR_hip,
  RR_hip,
};

enum class Leg : int {
  FL = 0,
  RL,
  FR,
  RR,
};

enum class Joint : int {
  hip = 0,
  thigh,
  calf,
};

constexpr int kLegCount = 4;
constexpr int kJointCount = 12;
constexpr int kJointPerLeg = 3;

inline static std::array<LegJoint, kJointCount> const kIndexToJoint = {
    LegJoint::FL_hip,   LegJoint::FL_thigh, LegJoint::FL_calf,  LegJoint::RL_hip,
    LegJoint::RL_thigh, LegJoint::RL_calf,  LegJoint::FR_hip,   LegJoint::FR_thigh,
    LegJoint::FR_calf,  LegJoint::RR_hip,   LegJoint::RR_thigh, LegJoint::RR_calf,
};

inline static std::array<int, kJointCount> const kJointToIndex = {
    1, 2, 4, 5, 7, 8, 10, 11, 0, 3, 6, 9,
};

inline static std::array<Leg, kLegCount> const kIndexToLeg = {Leg::FL, Leg::RL, Leg::FR, Leg::RR};

inline static std::array<int, kLegCount> const kLegToIndex = {0, 1, 2, 3};

inline static std::array<std::array<LegJoint, kJointPerLeg>, kLegCount> const
    kLegAndJointIdxToJoint = {
        std::array<LegJoint, kJointPerLeg>{LegJoint::FL_hip, LegJoint::FL_thigh, LegJoint::FL_calf},
        std::array<LegJoint, kJointPerLeg>{LegJoint::RL_hip, LegJoint::RL_thigh, LegJoint::RL_calf},
        std::array<LegJoint, kJointPerLeg>{LegJoint::FR_hip, LegJoint::FR_thigh, LegJoint::FR_calf},
        std::array<LegJoint, kJointPerLeg>{LegJoint::RR_hip, LegJoint::RR_thigh, LegJoint::RR_calf},
};

inline static std::array<std::pair<Leg, Joint>, kJointCount> const kJointToLegAndJointIdx = {
    std::pair<Leg, Joint>{Leg::FL, Joint::thigh}, std::pair<Leg, Joint>{Leg::FL, Joint::calf},
    std::pair<Leg, Joint>{Leg::RL, Joint::thigh}, std::pair<Leg, Joint>{Leg::RL, Joint::calf},
    std::pair<Leg, Joint>{Leg::RR, Joint::thigh}, std::pair<Leg, Joint>{Leg::RR, Joint::calf},
    std::pair<Leg, Joint>{Leg::FL, Joint::hip},   std::pair<Leg, Joint>{Leg::RL, Joint::hip},
    std::pair<Leg, Joint>{Leg::FR, Joint::hip},   std::pair<Leg, Joint>{Leg::RR, Joint::hip},
};

}  // namespace jsc
