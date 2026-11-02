#include "Robot.hpp"
#include <cstdio>

using namespace wrpuppets::fbt;

namespace lab {
namespace {
const float kBody[3] = {0.78f, 0.79f, 0.82f};
const float kLeft[3] = {0.55f, 0.66f, 0.92f};
const float kRight[3] = {0.92f, 0.58f, 0.55f};
const float kDark[3] = {0.12f, 0.16f, 0.20f};
const float kThumb[3] = {0.95f, 0.85f, 0.25f};   // the thumb in yellow, so the hand's rotation reads at a glance

// A model-space direction of the reference pose, in bone b's local frame.
Vec3 ToLocal(const Skeleton& sk, int b, Vec3 modelDir) { return Rotate(Conjugate(sk.BoneAt(b).refModelRot), modelDir); }
Vec3 ToLocalPoint(const Skeleton& sk, int b, Vec3 modelPoint) {
    return Rotate(Conjugate(sk.BoneAt(b).refModelRot), modelPoint - sk.BoneAt(b).refModelPos);
}

// A box from bone `from`'s joint to bone `to`'s joint (to is a descendant), `width` across and `depth` front to back.
// Its side axis is V's model x made perpendicular to the bone (model z for bones along x), its front axis the third.
bool SegmentTo(const Skeleton& sk, int from, Vec3 b, float width, float depth, const float* color, Box* out);
bool Segment(const Skeleton& sk, int from, int to, float width, float depth, const float* color, Box* out) {
    if (from < 0 || to < 0) return false;
    return SegmentTo(sk, from, sk.BoneAt(to).refModelPos, width, depth, color, out);
}
// The same, to a model-space point of the reference pose (a fingertip with no bone of its own).
bool SegmentTo(const Skeleton& sk, int from, Vec3 b, float width, float depth, const float* color, Box* out) {
    if (from < 0) return false;
    const Vec3 a = sk.BoneAt(from).refModelPos;
    const Vec3 along = Normalize(b - a);
    if (Length(b - a) < 0.005f) return false;
    // The side reference: model x, unless the bone runs along x (the arms in the T-pose): then model z (up).
    Vec3 side = std::fabs(along.x) > 0.8f ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    side = Normalize(side - along * Dot(side, along));
    const Vec3 front = Normalize(Cross(side, along));   // the box is symmetric: its sign does not matter
    Box box;
    box.bone = from;
    box.center = ToLocalPoint(sk, from, (a + b) * 0.5f);
    box.axisX = ToLocal(sk, from, side);
    box.axisY = ToLocal(sk, from, along);
    box.axisZ = ToLocal(sk, from, front);
    box.size = {width, Length(b - a), depth};
    for (int k = 0; k < 3; ++k) box.color[k] = color[k];
    *out = box;
    return true;
}

// A box from a to b (model space, reference pose) with its width along `across` (made perpendicular to a -> b), attached
// to bone `bone`.
bool Oriented(const Skeleton& sk, int bone, Vec3 a, Vec3 b, Vec3 across, float width, float depth, const float* color, Box* out) {
    if (bone < 0 || Length(b - a) < 0.005f) return false;
    const Vec3 along = Normalize(b - a);
    const Vec3 side = Normalize(across - along * Dot(across, along));
    Box box;
    box.bone = bone;
    box.center = ToLocalPoint(sk, bone, (a + b) * 0.5f);
    box.axisX = ToLocal(sk, bone, side);
    box.axisY = ToLocal(sk, bone, along);
    box.axisZ = ToLocal(sk, bone, Normalize(Cross(side, along)));
    box.size = {width, Length(b - a), depth};
    for (int k = 0; k < 3; ++k) box.color[k] = color[k];
    *out = box;
    return true;
}

// A box with V's model axes at the reference pose, centred at `modelCenter`, attached to bone b.
bool Fixed(const Skeleton& sk, int b, Vec3 modelCenter, Vec3 size, const float* color, Box* out) {
    if (b < 0) return false;
    Box box;
    box.bone = b;
    box.center = ToLocalPoint(sk, b, modelCenter);
    box.axisX = ToLocal(sk, b, {1, 0, 0});
    box.axisY = ToLocal(sk, b, {0, 1, 0});
    box.axisZ = ToLocal(sk, b, {0, 0, 1});
    box.size = size;
    for (int k = 0; k < 3; ++k) box.color[k] = color[k];
    *out = box;
    return true;
}
}  // namespace

std::vector<Box> BuildRobot(const Skeleton& sk) {
    std::vector<Box> boxes;
    Box box;
    auto add = [&](bool ok) { if (ok) boxes.push_back(box); };
    const int hips = sk.Role("hips"), head = sk.Role("head"), neck = sk.Role("neck"), neck1 = sk.Role("neck1");
    auto pos = [&](int b) { return b >= 0 ? sk.BoneAt(b).refModelPos : Vec3{}; };

    // Pelvis and torso: the spine bones in a row, wide and flat, then the neck.
    if (hips >= 0) add(Fixed(sk, hips, pos(hips) + Vec3{0, 0, -0.03f}, {0.30f, 0.18f, 0.16f}, kBody, &box));
    int spine[8], spines = 0;
    for (int c = 0; c < 8; ++c) {
        char role[16];
        sprintf_s(role, "spine%d", c);
        if (const int b = sk.Role(role); b >= 0) spine[spines++] = b;
    }
    for (int c = 0; c < spines; ++c) {
        const int to = c + 1 < spines ? spine[c + 1] : neck;
        const float width = 0.28f + 0.03f * c;
        add(Segment(sk, spine[c], to, width, 0.18f, kBody, &box));
    }
    if (neck >= 0 && neck1 >= 0) add(Segment(sk, neck, neck1, 0.09f, 0.09f, kBody, &box));
    if (neck1 >= 0 && head >= 0) add(Segment(sk, neck1, head, 0.08f, 0.08f, kBody, &box));
    // Head: a box above the head joint, and a dark visor on its front.
    if (head >= 0) {
        add(Fixed(sk, head, pos(head) + Vec3{0, 0.01f, 0.10f}, {0.17f, 0.21f, 0.23f}, kBody, &box));
        add(Fixed(sk, head, pos(head) + Vec3{0, 0.115f, 0.12f}, {0.14f, 0.012f, 0.06f}, kDark, &box));
    }

    // Arms and legs, per side.
    for (int side = 0; side < 2; ++side) {
        const bool left = side == 0;
        const float* tint = left ? kLeft : kRight;
        const char* s = left ? "left" : "right";
        char role[32];
        auto roleOf = [&](const char* part) { sprintf_s(role, "%s_%s", s, part); return sk.Role(role); };
        const int upper = roleOf("upperarm"), fore = roleOf("forearm"), hand = roleOf("hand");
        const int upLeg = roleOf("upleg"), leg = roleOf("leg"), foot = roleOf("foot");
        if (upper >= 0) add(Segment(sk, sk.BoneAt(upper).parent, upper, 0.07f, 0.07f, tint, &box));   // clavicle
        add(Segment(sk, upper, fore, 0.09f, 0.09f, tint, &box));
        add(Segment(sk, fore, hand, 0.075f, 0.075f, tint, &box));
        // Palm: from the wrist to the middle of the knuckle row, as wide as the row (index -> pinky) plus a margin, thin
        // across the palm.
        if (hand >= 0) {
            const int index = sk.Find(left ? "LeftHandIndex1" : "RightHandIndex1"), pinky = sk.Find(left ? "LeftHandPinky1" : "RightHandPinky1");
            const int end = sk.Role(left ? "left_hand_end" : "right_hand_end");
            if (index >= 0 && pinky >= 0) {
                const Vec3 knuckles = (pos(index) + pos(pinky)) * 0.5f;
                add(Oriented(sk, hand, pos(hand), knuckles, pos(pinky) - pos(index), Length(pos(pinky) - pos(index)) + 0.02f, 0.03f, tint, &box));
            } else {
                add(Segment(sk, hand, end >= 0 ? end : hand, 0.09f, 0.035f, tint, &box));
            }
        }
        // Fingers: a box per segment, the last one out to an extrapolated fingertip; the thumb yellow.
        const char* side_ = left ? "Left" : "Right";
        struct { const char* name; int first, last; } fingers[] = {
            {"Thumb", 0, 2}, {"Index", 0, 3}, {"Middle", 0, 3}, {"Ring", 0, 3}, {"Pinky", 0, 3}};
        for (const auto& f : fingers) {
            const bool thumb = f.name[0] == 'T';
            int joints[5], count = 0;
            char name[48];
            sprintf_s(name, "%sInHand%s", side_, f.name);
            if (const int b = sk.Find(name); b >= 0 && thumb) joints[count++] = b;   // the thumb's metacarpal shows
            for (int i = 1; i <= f.last; ++i) {
                sprintf_s(name, "%sHand%s%d", side_, f.name, i);
                if (const int b = sk.Find(name); b >= 0) joints[count++] = b;
            }
            const float w = thumb ? 0.022f : 0.017f, d = thumb ? 0.020f : 0.015f;
            const float* col = thumb ? kThumb : tint;
            for (int i = 0; i + 1 < count; ++i) add(Segment(sk, joints[i], joints[i + 1], w, d, col, &box));
            if (count >= 2) {
                const Vec3 last = pos(joints[count - 1]), before = pos(joints[count - 2]);
                add(SegmentTo(sk, joints[count - 1], last + (last - before) * 0.8f, w * 0.9f, d * 0.9f, col, &box));
            }
        }
        add(Segment(sk, upLeg, leg, 0.14f, 0.14f, tint, &box));
        add(Segment(sk, leg, foot, 0.11f, 0.11f, tint, &box));
        // Foot: from the heel to the toe on the floor, and a dark toe cap.
        if (foot >= 0) {
            const int heel = sk.Find(left ? "LeftHeel" : "RightHeel"), toe = sk.Find(left ? "LeftToeBase" : "RightToeBase");
            const Vec3 ankle = pos(foot);
            const Vec3 back = heel >= 0 ? pos(heel) : ankle - Vec3{0, 0.05f, 0};
            const Vec3 front = toe >= 0 ? pos(toe) : ankle + Vec3{0, 0.15f, 0};
            const float length = std::fabs(front.y - back.y) + 0.06f;
            const Vec3 center{(back.x + front.x) * 0.5f, (back.y + front.y) * 0.5f + 0.02f, ankle.z * 0.5f};
            add(Fixed(sk, foot, center, {0.10f, length, ankle.z}, tint, &box));
            add(Fixed(sk, foot, {center.x, front.y + 0.04f, ankle.z * 0.3f}, {0.104f, 0.04f, ankle.z * 0.6f}, kDark, &box));
        }
    }
    return boxes;
}

void Place(const Box& box, const Pose& pose, Vec3* center, Vec3 axes[3]) {
    const Quat r = pose.modelRot[box.bone];
    *center = pose.modelPos[box.bone] + Rotate(r, box.center);
    axes[0] = Rotate(r, box.axisX) * box.size.x;
    axes[1] = Rotate(r, box.axisY) * box.size.y;
    axes[2] = Rotate(r, box.axisZ) * box.size.z;
}

}  // namespace lab
