#pragma once

// The lab's dummy: a robot made of boxes, one per body segment, each attached rigidly to its bone (like VRChat's robot
// avatar), so a bone's full rotation -- twist and roll included -- is visible. A box is defined in its bone's local frame
// from V's reference pose: its long axis along the bone (or a fixed shape for pelvis, head, hands, feet) and its side /
// front axes taken from V's model axes (x right, y forward) in that pose, so in the T-pose every box faces forward.
#include "Fbt/FbtSkeleton.hpp"
#include <vector>

namespace lab {

struct Box {
    int bone = -1;
    wrpuppets::fbt::Vec3 center;                      // in the bone's local frame
    wrpuppets::fbt::Vec3 axisX, axisY, axisZ;          // unit, in the bone's local frame
    wrpuppets::fbt::Vec3 size;                         // along axisX, axisY, axisZ, metres
    float color[3]{};
};

// The robot for this skeleton; boxes whose bones are missing are left out.
std::vector<Box> BuildRobot(const wrpuppets::fbt::Skeleton& skeleton);

// A box in model space for a pose: centre, the three axes scaled by the size.
void Place(const Box& box, const wrpuppets::fbt::Pose& pose, wrpuppets::fbt::Vec3* center, wrpuppets::fbt::Vec3 axes[3]);

}  // namespace lab
