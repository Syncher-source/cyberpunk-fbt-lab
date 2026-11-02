#pragma once

// FLIGHT RECORDER for the VR lab: after a calibration, every frame's raw device poses (room space: STAGE in V's model
// axes) with their tracked flags and time, and the calibration in the header -- enough to re-run the same calibration
// and IK offline (fbt_ik_test --replay) and tell an input jump (a device read wrong) from a solver jump. A recording
// plus the rig file is a complete bug report. Text:
//   cal <scale> <player eye height> <V eye height> <origin xyz> <yaw inverse xyzw> <head rot inverse xyzw>
//   arm <left> <right> <1 if measured with the arms out>          (v2; the arm lengths in V's scale)
//   dev <index> <valid> <offset xyz> <rot inverse xyzw>             per calibrated device
// then one frame per line:
//   f <seconds> then per device (head, waist, left foot, right foot, left hand, right hand): valid px py pz qx qy qz qw
//   then (v2, optional) the hand-tracking wrist joints, left and right, the same way
#include "Fbt/FbtCalibration.hpp"
#include <cstdio>
#include <string>
#include <vector>

namespace lab {

struct RecFrame {
    double t = 0;
    wrpuppets::fbt::RoomPose head, dev[wrpuppets::fbt::kCalCount];   // the hands: grip poses
    wrpuppets::fbt::RoomPose wrist[2];                           // the hand-tracking wrist joints (wrist = true when valid)
};

struct Recording {
    wrpuppets::fbt::Calibration calibration;
    bool armsOut = false;   // the arm lengths were measured with the arms out to the sides (else hanging: not a scale)
    bool armsDownFile = false;   // a v2 file: captured with the arms down (the hanging length recorded; v1: the T-pose)
    std::vector<RecFrame> frames;
};

inline bool SaveRecording(const char* path, const Recording& r) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "w") != 0 || !f) return false;
    const auto& c = r.calibration;
    fprintf(f, "# fbt recording v2: calibration, then one frame per line (room space, V's model axes)\n");
    fprintf(f, "cal %.7f %.6f %.6f  %.6f %.6f %.6f  %.7f %.7f %.7f %.7f  %.7f %.7f %.7f %.7f\n", c.scale, c.playerEyeHeight,
            c.avatarEyeHeight, c.origin.x, c.origin.y, c.origin.z, c.yawInverse.x, c.yawInverse.y, c.yawInverse.z, c.yawInverse.w,
            c.headRotInverse.x, c.headRotInverse.y, c.headRotInverse.z, c.headRotInverse.w);
    fprintf(f, "arm %.5f %.5f %d\n", c.armLength[0], c.armLength[1], r.armsOut ? 1 : 0);
    for (int i = 0; i < wrpuppets::fbt::kCalCount; ++i)
        fprintf(f, "dev %d %d  %.6f %.6f %.6f  %.7f %.7f %.7f %.7f\n", i, c.valid[i] ? 1 : 0, c.offset[i].x, c.offset[i].y, c.offset[i].z,
                c.rotInverse[i].x, c.rotInverse[i].y, c.rotInverse[i].z, c.rotInverse[i].w);
    auto pose = [&](const wrpuppets::fbt::RoomPose& p) {
        fprintf(f, "  %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f", p.valid ? 1 : 0, p.pos.x, p.pos.y, p.pos.z, p.rot.x, p.rot.y, p.rot.z, p.rot.w);
    };
    for (const auto& fr : r.frames) {
        fprintf(f, "f %.5f", fr.t);
        pose(fr.head);
        for (const auto& d : fr.dev) pose(d);
        for (const auto& w : fr.wrist) pose(w);
        fprintf(f, "\n");
    }
    fclose(f);
    return true;
}

inline bool LoadRecording(const char* path, Recording* out) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") != 0 || !f) return false;
    Recording r;
    auto& c = r.calibration;
    char line[2048];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'c') {
            sscanf_s(line, "cal %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &c.scale, &c.playerEyeHeight, &c.avatarEyeHeight, &c.origin.x,
                     &c.origin.y, &c.origin.z, &c.yawInverse.x, &c.yawInverse.y, &c.yawInverse.z, &c.yawInverse.w, &c.headRotInverse.x,
                     &c.headRotInverse.y, &c.headRotInverse.z, &c.headRotInverse.w);
            c.done = true;
        } else if (line[0] == 'a') {
            int out = 0;
            sscanf_s(line, "arm %f %f %d", &c.armLength[0], &c.armLength[1], &out);
            r.armsOut = out != 0;
            r.armsDownFile = true;
        } else if (line[0] == 'd') {
            int i = 0, valid = 0;
            wrpuppets::fbt::Vec3 o;
            wrpuppets::fbt::Quat q;
            if (sscanf_s(line, "dev %d %d %f %f %f %f %f %f %f", &i, &valid, &o.x, &o.y, &o.z, &q.x, &q.y, &q.z, &q.w) == 9 && i >= 0 &&
                i < wrpuppets::fbt::kCalCount) {
                c.valid[i] = valid != 0;
                c.offset[i] = o;
                c.rotInverse[i] = q;
            }
        } else if (line[0] == 'f') {
            RecFrame fr;
            const char* p = line + 1;
            char* end = nullptr;
            fr.t = strtod(p, &end);
            p = end;
            auto pose = [&](wrpuppets::fbt::RoomPose& d) {
                float v[8]{};
                for (float& x : v) {
                    x = strtof(p, &end);
                    if (end == p) return;   // an older recording: no more fields
                    p = end;
                }
                d.valid = v[0] != 0;
                d.pos = {v[1], v[2], v[3]};
                d.rot = {v[4], v[5], v[6], v[7]};
            };
            pose(fr.head);
            for (auto& d : fr.dev) pose(d);
            for (auto& w : fr.wrist) { pose(w); w.wrist = w.valid; }
            r.frames.push_back(fr);
        }
    }
    fclose(f);
    *out = r;
    return c.done;
}

}  // namespace lab
