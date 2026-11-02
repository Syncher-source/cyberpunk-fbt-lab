#define CGLTF_IMPLEMENTATION
#include "cgltf.h"
#include "VMesh.hpp"
#include <cmath>
#include <cstring>

using namespace wrpuppets::fbt;

namespace lab {
namespace {
// K: the glTF joint frame from the game bone's (90 deg about x; rows).
constexpr float kK[3][3] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
}  // namespace

void VMesh::Clear() { *this = {}; }

bool VMesh::Load(const char* path, const Skeleton& skeleton, const float color[3], std::string* error) {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&options, path, &data) != cgltf_result_success) { if (error) *error = std::string("cannot read ") + path; return false; }
    if (cgltf_load_buffers(&options, data, path) != cgltf_result_success) {
        cgltf_free(data);
        if (error) *error = std::string("cannot load the buffers of ") + path;
        return false;
    }
    Part part;
    part.name = path;
    if (const char* s = std::strrchr(path, '\\')) part.name = s + 1;
    part.firstIndex = static_cast<unsigned>(m_indices.size());
    std::memcpy(part.color, color, sizeof(part.color));
    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (!node.mesh || !node.skin) continue;
        const cgltf_skin& skin = *node.skin;
        // This skin's joints, appended to the shared list: the bone (by name, else the nearest ancestor in the rig) and
        // K * inverseBind.
        const size_t jointBase = m_joints.size();
        for (cgltf_size j = 0; j < skin.joints_count; ++j) {
            Joint joint;
            for (const cgltf_node* walk = skin.joints[j]; walk && joint.bone < 0; walk = walk->parent)
                if (walk->name) joint.bone = skeleton.Find(walk->name);
            if (!skin.joints[j]->name || skeleton.Find(skin.joints[j]->name) < 0) ++m_unmatched;
            float ibm[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};   // column-major
            if (skin.inverse_bind_matrices) cgltf_accessor_read_float(skin.inverse_bind_matrices, j, ibm, 16);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c) {
                    float v = 0;
                    for (int k = 0; k < 3; ++k) v += kK[r][k] * ibm[c * 4 + k];
                    joint.bind.m[r][c] = v;
                }
            m_joints.push_back(joint);
        }
        for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p) {
            const cgltf_primitive& prim = node.mesh->primitives[p];
            if (prim.type != cgltf_primitive_type_triangles || !prim.indices) continue;
            const cgltf_accessor *position = nullptr, *normal = nullptr, *joints[2]{}, *weights[2]{};
            for (cgltf_size a = 0; a < prim.attributes_count; ++a) {
                const cgltf_attribute& at = prim.attributes[a];
                if (at.type == cgltf_attribute_type_position) position = at.data;
                else if (at.type == cgltf_attribute_type_normal) normal = at.data;
                else if (at.type == cgltf_attribute_type_joints && at.index < 2) joints[at.index] = at.data;
                else if (at.type == cgltf_attribute_type_weights && at.index < 2) weights[at.index] = at.data;
            }
            if (!position || !joints[0] || !weights[0]) continue;
            const unsigned base = static_cast<unsigned>(m_pos.size() / 3);
            for (cgltf_size v = 0; v < position->count; ++v) {
                float x[3]{}, nn[3] = {0, 1, 0};
                cgltf_accessor_read_float(position, v, x, 3);
                if (normal) cgltf_accessor_read_float(normal, v, nn, 3);
                m_pos.insert(m_pos.end(), x, x + 3);
                m_normal.insert(m_normal.end(), nn, nn + 3);
                Influence inf{};
                float total = 0;
                for (int set = 0; set < 2; ++set) {
                    if (!joints[set] || !weights[set]) continue;
                    cgltf_uint jj[4]{};
                    float ww[4]{};
                    cgltf_accessor_read_uint(joints[set], v, jj, 4);
                    cgltf_accessor_read_float(weights[set], v, ww, 4);
                    for (int k = 0; k < 4; ++k) {
                        inf.joint[set * 4 + k] = static_cast<unsigned short>(jointBase + jj[k]);
                        inf.weight[set * 4 + k] = ww[k];
                        total += ww[k];
                    }
                }
                if (total > 0)
                    for (float& w : inf.weight) w /= total;
                m_influence.push_back(inf);
            }
            for (cgltf_size i = 0; i < prim.indices->count; ++i)
                m_indices.push_back(base + static_cast<unsigned>(cgltf_accessor_read_index(prim.indices, i)));
        }
    }
    cgltf_free(data);
    part.indexCount = static_cast<unsigned>(m_indices.size()) - part.firstIndex;
    if (part.indexCount == 0) { if (error) *error = std::string("no skinned triangles in ") + path; return false; }
    m_parts.push_back(part);
    m_skinned.resize(m_pos.size() / 3);
    return true;
}

void VMesh::Skin(const Pose& pose) {
    // Each joint now: the bone's model transform (game space) * K * inverseBind.
    m_jointNow.resize(m_joints.size());
    for (size_t j = 0; j < m_joints.size(); ++j) {
        const Joint& jt = m_joints[j];
        Mat34& out = m_jointNow[j];
        if (jt.bone < 0) { std::memset(&out, 0, sizeof(out)); continue; }
        const Quat q = pose.modelRot[jt.bone];
        const Vec3 t = pose.modelPos[jt.bone];
        const float x = q.x, y = q.y, z = q.z, w = q.w;
        const float R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                               {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                               {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
        const float tt[3] = {t.x, t.y, t.z};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) {
                float v = c == 3 ? tt[r] : 0;
                for (int k = 0; k < 3; ++k) v += R[r][k] * jt.bind.m[k][c];
                out.m[r][c] = v;
            }
    }
    const size_t count = m_influence.size();
    for (size_t v = 0; v < count; ++v) {
        const float* p = &m_pos[v * 3];
        const float* n = &m_normal[v * 3];
        float sp[3]{}, sn[3]{};
        const Influence& inf = m_influence[v];
        for (int k = 0; k < 8; ++k) {
            const float w = inf.weight[k];
            if (w <= 0) continue;
            const Mat34& M = m_jointNow[inf.joint[k]];
            for (int r = 0; r < 3; ++r) {
                sp[r] += w * (M.m[r][0] * p[0] + M.m[r][1] * p[1] + M.m[r][2] * p[2] + M.m[r][3]);
                sn[r] += w * (M.m[r][0] * n[0] + M.m[r][1] * n[1] + M.m[r][2] * n[2]);
            }
        }
        const float len = std::sqrt(sn[0] * sn[0] + sn[1] * sn[1] + sn[2] * sn[2]);
        BoxRenderer::MeshVertex& o = m_skinned[v];
        for (int r = 0; r < 3; ++r) {
            o.pos[r] = sp[r];
            o.normal[r] = len > 1e-8f ? sn[r] / len : 0;
        }
    }
}

float VMesh::RigidCheck(const Pose& pose, int* count) const {
    float worst = 0;
    int checked = 0;
    for (size_t v = 0; v < m_influence.size(); ++v) {
        const Influence& inf = m_influence[v];
        int only = -1;
        for (int k = 0; k < 8; ++k)
            if (inf.weight[k] > 0.999f) only = inf.joint[k];
        if (only < 0 || m_joints[only].bone < 0) continue;
        // The joint's bind position in glTF space: -R^T t of the inverse bind (bind = K * IBM, K a rotation: same length).
        const Mat34& B = m_joints[only].bind;
        float jb[3];
        for (int c = 0; c < 3; ++c) jb[c] = -(B.m[0][c] * B.m[0][3] + B.m[1][c] * B.m[1][3] + B.m[2][c] * B.m[2][3]);
        const float* p = &m_pos[v * 3];
        const float dBind = std::sqrt((p[0] - jb[0]) * (p[0] - jb[0]) + (p[1] - jb[1]) * (p[1] - jb[1]) + (p[2] - jb[2]) * (p[2] - jb[2]));
        const Vec3 j = pose.modelPos[m_joints[only].bone];
        const BoxRenderer::MeshVertex& o = m_skinned[v];
        const float dNow = Length(Vec3{o.pos[0], o.pos[1], o.pos[2]} - j);
        worst = std::fmax(worst, std::fabs(dNow - dBind));
        ++checked;
    }
    if (count) *count = checked;
    return worst;
}

bool VMesh::Centroid(int bone, Vec3* out) const {
    Vec3 sum{};
    int n = 0;
    for (size_t v = 0; v < m_influence.size(); ++v) {
        const Influence& inf = m_influence[v];
        int best = 0;
        for (int k = 1; k < 8; ++k)
            if (inf.weight[k] > inf.weight[best]) best = k;
        if (m_joints[inf.joint[best]].bone != bone) continue;
        const auto& o = m_skinned[v];
        sum = sum + Vec3{o.pos[0], o.pos[1], o.pos[2]};
        ++n;
    }
    if (!n) return false;
    *out = sum * (1.0f / n);
    return true;
}

}  // namespace lab
