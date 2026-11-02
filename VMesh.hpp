#pragma once

// V'S BODY MESH for the lab: the game's player body parts (torso + legs, arms, underwear), exported by WolvenKit to glTF
// binary (game files: each user exports their own, see README.md), skinned on the CPU to the lab's pose.
//
// WolvenKit's glTF is Y-up: a point (x, y, z) of V's model space is (x, z, -y) there (C), and every joint frame is the
// game bone's turned 90 deg about its own x (K: glTF joint = C * game bone * K; checked on the hips, spine and neck,
// which sit the same in the mesh's bind pose -- an A-pose -- and in V's reference T-pose; bone lengths match to 1 mm).
// So a vertex v of the mesh lands in V's model space at
//   sum over its joints of  weight * (game bone's model transform now) * K * inverseBind * v
// with the bones matched by name to the rig dump (a joint missing from it falls back to its nearest ancestor).
#include "BoxRenderer.hpp"
#include "Fbt/FbtSkeleton.hpp"
#include <string>
#include <vector>

namespace lab {

class VMesh {
public:
    struct Part {
        std::string name;
        unsigned firstIndex = 0, indexCount = 0;
        float color[3]{};
    };
    // Appends one .glb (all its skinned primitives) as a part with the given colour; false + *error when it fails.
    bool Load(const char* path, const wrpuppets::fbt::Skeleton& skeleton, const float color[3], std::string* error);
    void Clear();
    bool Empty() const { return m_indices.empty(); }
    // Skins every vertex to the pose (model transforms current).
    void Skin(const wrpuppets::fbt::Pose& pose);
    const std::vector<BoxRenderer::MeshVertex>& Vertices() const { return m_skinned; }
    const std::vector<unsigned>& Indices() const { return m_indices; }
    const std::vector<Part>& Parts() const { return m_parts; }
    int UnmatchedJoints() const { return m_unmatched; }
    // Check: for every vertex owned by one joint alone, its distance to that joint skinned to the pose against the same in
    // the mesh's bind pose (a rigid part keeps it). Returns the worst difference, metres; *count the vertices checked.
    float RigidCheck(const wrpuppets::fbt::Pose& pose, int* count) const;
    // The mean skinned position of the vertices whose largest weight is on `bone` (orientation checks); false if none.
    bool Centroid(int bone, wrpuppets::fbt::Vec3* out) const;

private:
    struct Mat34 { float m[3][4]; };
    struct Joint { int bone = -1; Mat34 bind; };   // bind = K * inverseBind
    struct Influence { unsigned short joint[8]; float weight[8]; };
    std::vector<Joint> m_joints;
    std::vector<float> m_pos, m_normal;   // 3 per vertex, glTF space
    std::vector<Influence> m_influence;
    std::vector<unsigned> m_indices;
    std::vector<Part> m_parts;
    std::vector<BoxRenderer::MeshVertex> m_skinned;
    std::vector<Mat34> m_jointNow;
    int m_unmatched = 0;
};

}  // namespace lab
