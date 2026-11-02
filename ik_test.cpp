// FBT IK TEST: the stack (the port's src/Fbt) on V's rig, no window, three modes.
//   fbt_ik_test.exe <rig file>                          the stress test: invariants (bone lengths, goals reached, hinges,
//                                                       knees forward...) over goal sweeps, 20,000 random poses and a
//                                                       synthetic calibration; a PASS/FAIL report
//   fbt_ik_test.exe --replay <recording> <rig file> [--nofilter]
//                                                       a VR-lab recording through the same calibration, filter and
//                                                       stack: statistics, then the worst frames (biggest bone step
//                                                       from the previous frame), each classed INPUT (a device jumped),
//                                                       REACH (a limb at its length) or SOLVER (a jump on smooth input)
//   fbt_ik_test.exe --mesh <rig file> <glb>...          V's exported body skinned to the reference pose: rigid parts and
//                                                       orientation checks
// Replay knobs (environment variables, for tuning without a rebuild): FBT_DUMP_FRAMES=from-to prints per-frame
// detail; the rest override StackOptions / FilterOptions / Calibrator fields, see kReplayKnobs below.
#include "Fbt/FbtCalibration.hpp"
#include "Fbt/FbtFilter.hpp"
#include "Fbt/FbtIk.hpp"
#include "Recording.hpp"
#include "VMesh.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

using namespace wrpuppets::fbt;

namespace {
constexpr float kDeg = 3.14159265f / 180.0f;

float AngleBetween(Quat a, Quat b) {
    const float d = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * std::acos(std::fmin(1.0f, d)) / kDeg;
}

struct Rig {
    Skeleton sk;
    IkStack stack;
    int hips, upLeg[2], leg[2], foot[2];
    Vec3 thighSide[2], shinSide[2], footFwd[2], footSide[2];   // in the bones' own frames
    float a[2], b[2];
    int spine[16], spineCount = 0;   // Spine .. Head
    TwoBone arm[2];                  // as the stack builds them (MakeTwoBone)
};

struct Worst {
    float length = 0, reach = 0, footRot = 0, hinge = 0;
    int kneeBackward = 0, cases = 0, fallback = 0;
    float spineLength = 0, head = 0, headRot = 0;
    float armLength = 0, wrist = 0, handRot = 0, elbowHinge = 0, elbowRoll = 0, wristBend = 0;
    int wristClamped = 0;
    std::string where;
};

const StackDebug* g_debug = nullptr;
float g_wristLimit = StackOptions{}.wristLimitDeg;
void Check(const Rig& r, const Pose& p, const Goal g[kGoalCount], const StackOptions& o, Worst& w, const char* label) {
    ++w.cases;
    if (g_debug)
        for (int s = 0; s < 2; ++s) w.elbowRoll = std::fmax(w.elbowRoll, std::fabs(g_debug->elbowRollDeg[s]));
    // Arms: lengths, the wrist on its goal when reachable, the hand on its rotation, upper arm and forearm on the hinge.
    for (int s = 0; s < 2; ++s) {
        const TwoBone& c = r.arm[s];
        const Goal& hg = g[s == 0 ? kGoalLeftHand : kGoalRightHand];
        const Vec3 sh = p.modelPos[c.upper], el = p.modelPos[c.lower], wr = p.modelPos[c.end];
        const float le = std::fmax(std::fabs(Length(el - sh) - c.a), std::fabs(Length(wr - el) - c.b));
        if (le > w.armLength) w.armLength = le;
        const float d = Length(hg.pos - sh);
        if (d < (c.a + c.b) * (1.0f - o.armSoftness) - 1e-3f && d > std::fabs(c.a - c.b) + 1e-2f) {
            const float e = Length(wr - hg.pos);
            if (e > w.wrist) { w.wrist = e; w.where = label; }
        }
        // The hand on the goal's rotation unless the wrist limit clamped it; never bent past the limit.
        const bool clamped = g_debug && g_debug->wristClamped[s];
        if (clamped) ++w.wristClamped;
        const float hr = AngleBetween(p.modelRot[c.end], hg.rot);
        if (!clamped && hr > w.handRot) w.handRot = hr;
        if (g_debug && !clamped) w.wristBend = std::fmax(w.wristBend, g_debug->wristBendDeg[s]);
        const Vec3 line = Normalize(wr - sh), off = (el - sh) - line * Dot(el - sh, line);
        if (Length(off) > 0.005f) {
            const Vec3 hinge = Normalize(Cross(line, Normalize(off)));
            // The upper arm only: the twist solver turns the forearm about its own axis on purpose.
            const float h = std::acos(std::fmin(1.0f, Dot(Rotate(p.modelRot[c.upper], c.upperHinge), hinge))) / kDeg;
            if (h > w.elbowHinge) w.elbowHinge = h;
        }
    }
    // Spine: lengths kept; the head on its goal when reachable, and on its rotation.
    {
        float total = 0;
        for (int i = 0; i + 1 < r.spineCount; ++i) {
            const float l = Length(r.sk.BoneAt(r.spine[i + 1]).refLocalPos);
            total += l;
            const float e = std::fabs(Length(p.modelPos[r.spine[i + 1]] - p.modelPos[r.spine[i]]) - l);
            if (e > w.spineLength) w.spineLength = e;
        }
        const Goal& hg = g[kGoalHead];
        const int headBone = r.spine[r.spineCount - 1];
        if (Length(hg.pos - p.modelPos[r.spine[0]]) < total - 0.01f) {
            const float e = Length(p.modelPos[headBone] - hg.pos);
            if (e > w.head) { w.head = e; w.where = label; }
        }
        const float hr = AngleBetween(p.modelRot[headBone], hg.rot);
        if (hr > w.headRot) w.headRot = hr;
    }
    for (int s = 0; s < 2; ++s) {
        const Goal& fg = g[s == 0 ? kGoalLeftFoot : kGoalRightFoot];
        const Vec3 hip = p.modelPos[r.upLeg[s]], knee = p.modelPos[r.leg[s]], ankle = p.modelPos[r.foot[s]];
        const float lenErr = std::fmax(std::fabs(Length(knee - hip) - r.a[s]), std::fabs(Length(ankle - knee) - r.b[s]));
        if (lenErr > w.length) w.length = lenErr;
        const float full = r.a[s] + r.b[s], d = Length(fg.pos - hip);
        if (d < full * (1.0f - o.legSoftness) - 1e-3f && d > std::fabs(r.a[s] - r.b[s]) + 1e-2f) {
            const float e = Length(ankle - fg.pos);
            if (e > w.reach) { w.reach = e; w.where = label; }
        }
        const float fr = AngleBetween(p.modelRot[r.foot[s]], fg.rot);
        if (fr > w.footRot) w.footRot = fr;
        // The knee is bent towards the foot's forward side of the leg line.
        const Vec3 line = Normalize(ankle - hip);
        const Vec3 off = (knee - hip) - line * Dot(knee - hip, line);
        if (Length(off) > 0.005f) {
            const Vec3 hinge = Normalize(Cross(line, Normalize(off)));
            // Thigh and shin lateral axes on the hinge.
            const float h1 = std::acos(std::fmin(1.0f, Dot(Rotate(p.modelRot[r.upLeg[s]], r.thighSide[s]), hinge))) / kDeg;
            const float h2 = std::acos(std::fmin(1.0f, Dot(Rotate(p.modelRot[r.leg[s]], r.shinSide[s]), hinge))) / kDeg;
            const float h = std::fmax(h1, h2);
            if (h > w.hinge) w.hinge = h;
            // Only where the foot drives the plane: below that the hips' forward takes over by design.
            const Vec3 side = Rotate(fg.rot, r.footSide[s]);
            const float across = Length(side - line * Dot(side, line));
            const Vec3 fwd = Rotate(fg.rot, r.footFwd[s]);
            if (across < 0.35f) ++w.fallback;
            // The knee towards the toes is the rule for a leg within 45 deg of vertical (standing, walking, crouching);
            // a leg thrown sideways or backwards bends its knee towards the floor or up, whatever the foot's pitch.
            else if (line.z < -0.7f && Dot(off, fwd) < -1e-4f && std::fabs(Dot(Normalize(fwd), line)) < 0.95f) {
                if (w.kneeBackward < 3 && std::getenv("FBT_DEBUG_KNEE")) {
                    const Vec3 hipsFwd = Rotate(g[kGoalHips].rot, Rotate(Conjugate(r.sk.BoneAt(r.hips).refModelRot), {0, 1, 0}));
                    std::printf("   knee back %s side %d: line (%+.2f %+.2f %+.2f) knee dir (%+.2f %+.2f %+.2f) foot fwd (%+.2f %+.2f %+.2f) "
                                "foot side (%+.2f %+.2f %+.2f) across %.2f hips fwd (%+.2f %+.2f %+.2f)\n", label, s, line.x, line.y, line.z,
                                Normalize(off).x, Normalize(off).y, Normalize(off).z, fwd.x, fwd.y, fwd.z, side.x, side.y, side.z, across,
                                hipsFwd.x, hipsFwd.y, hipsFwd.z);
                }
                ++w.kneeBackward;
            }
        }
    }
}

void Report(const char* name, const Worst& w) {
    const bool ok = w.length < 1e-4f && w.reach < 1e-3f && w.footRot < 0.1f && w.hinge < 0.5f && w.kneeBackward == 0 &&
                    w.spineLength < 1e-4f && w.head < 1e-3f && w.headRot < 0.1f && w.armLength < 1e-4f && w.wrist < 1e-3f &&
                    w.handRot < 0.1f && w.elbowHinge < 0.5f && w.elbowRoll <= 45.01f && w.wristBend <= g_wristLimit + 0.01f;
    std::printf("%-34s %s  cases %6d  len %.2e m  reach %.2e m  footRot %.3f deg  hinge %.3f deg  kneeBack %d  fallback %d%s%s\n",
                name, ok ? "PASS" : "FAIL", w.cases, w.length, w.reach, w.footRot, w.hinge, w.kneeBackward, w.fallback,
                w.where.empty() ? "" : "  worst at ", w.where.c_str());
    std::printf("%-34s       spine len %.2e m  head %.2e m  headRot %.3f deg | arm len %.2e m  wrist %.2e m  handRot %.3f deg  "
                "upper-arm hinge %.3f deg  elbow roll %.1f deg  wrist clamped %d\n", "", w.spineLength, w.head, w.headRot, w.armLength,
                w.wrist, w.handRot, w.elbowHinge, w.elbowRoll, w.wristClamped);
}

// A sweep: the biggest change of the thigh/shin rotations between consecutive steps; a flip shows as a jump far
// bigger than the step.
struct Sweep {
    Pose prev;
    bool have = false;
    float maxJump = 0;
    std::string where;
    void Step(const Rig& r, const Pose& p, const char* label) {
        auto jump = [&](int bone) {
            const float j = AngleBetween(prev.modelRot[bone], p.modelRot[bone]);
            if (j > maxJump) { maxJump = j; where = std::string(label) + " (" + r.sk.BoneAt(bone).name + ")"; }
        };
        if (have) {
            for (int s = 0; s < 2; ++s) { jump(r.upLeg[s]); jump(r.leg[s]); }
            for (int i = 0; i < r.spineCount; ++i) jump(r.spine[i]);
            for (int s = 0; s < 2; ++s) { jump(r.arm[s].upper); jump(r.arm[s].lower); }
        }
        prev = p;
        have = true;
    }
};
void ReportSweep(const char* name, const Sweep& s, float limitDeg) {
    std::printf("%-34s %s  max step change %.2f deg (limit %.1f)%s%s\n", name, s.maxJump <= limitDeg ? "PASS" : "FAIL", s.maxJump,
                limitDeg, s.where.empty() ? "" : "  at ", s.where.c_str());
}

void ResetGoals(const Rig& r, Goal g[kGoalCount]) {
    const char* roles[kGoalCount] = {"head", "left_hand", "right_hand", "hips", "left_foot", "right_foot"};
    for (int i = 0; i < kGoalCount; ++i) {
        const int b = r.sk.Role(roles[i]);
        g[i].active = b >= 0;
        if (b >= 0) { g[i].pos = r.sk.BoneAt(b).refModelPos; g[i].rot = r.sk.BoneAt(b).refModelRot; }
    }
}
}  // namespace

namespace {
bool EnvFlag(const char* name) { return std::getenv(name) != nullptr; }
// The replay's tuning knobs: an environment variable overriding one field (a float, or a flag turning a bool off or on).
template <class T> struct Knob { const char* name; T* field; bool flag; bool flagValue; };
void ApplyReplayKnobs(StackOptions& o, FilterOptions& fo, Calibrator& cal) {
    const Knob<float> floats[] = {
        {"FBT_ELBOW_DOWN", &o.elbowDown, false, false},        {"FBT_ELBOW_PALM_UP", &o.elbowDownPalmUp, false, false},
        {"FBT_ELBOW_FADE_START", &o.elbowFadeStart, false, false}, {"FBT_ELBOW_FADE_END", &o.elbowFadeEnd, false, false},
        {"FBT_ELBOW_REST_OUT", &o.elbowRestOut, false, false}, {"FBT_ELBOW_REST_BACK", &o.elbowRestBack, false, false},
        {"FBT_WRIST_LIMIT", &o.wristLimitDeg, false, false},   {"FBT_HIPS_STIFF", &o.hipsStiffness, false, false},
        {"FBT_HEAD_PULL", &o.headPull, false, false},          {"FBT_LEG_SHARE", &o.legReachShare, false, false},
        {"FBT_ROT_MIN_CUTOFF", &fo.rotMinCutoff, false, false}, {"FBT_ROT_BETA", &fo.rotBeta, false, false},
        {"FBT_POS_MIN_CUTOFF", &fo.posMinCutoff, false, false}, {"FBT_REJECT_DEG", &fo.rejectStepDeg, false, false},
        {"FBT_WRIST_BACK", &cal.handWristBack, false, false},
    };
    for (const auto& k : floats)
        if (const char* e = std::getenv(k.name)) *k.field = static_cast<float>(std::atof(e));
    const Knob<bool> flags[] = {
        {"FBT_ELBOW_FIXED", &o.elbowFromThumb, true, false},     {"FBT_ELBOW_THUMB", &o.elbowFromThumb, true, true},
        {"FBT_ELBOW_POSITION", &o.elbowFromPosition, true, true}, {"FBT_NO_ELBOW_POSITION", &o.elbowFromPosition, true, false},
        {"FBT_NO_ZONES", &o.elbowZones, true, false},            {"FBT_TWIST", &o.twist, true, true},
        {"FBT_NO_TWIST", &o.twist, true, false},                 {"FBT_TWIST_PERIODIC", &o.twistRollPeriodic, true, true},
        {"FBT_NO_GIVE_WAY", &o.hipsGiveWay, true, false},        {"FBT_HANDS_CAPTURED", &cal.handsFixedMount, true, false},
    };
    for (const auto& k : flags)
        if (std::getenv(k.name)) *k.field = k.flagValue;
    if (const char* e = std::getenv("FBT_MAX_HOLD")) fo.maxHoldFrames = std::atoi(e);
}
}  // namespace

int Replay(const char* recPath, const char* rigPath, bool filtered) {
    Skeleton sk;
    std::string err;
    if (!sk.Load(rigPath, &err)) { std::printf("%s\n", err.c_str()); return 2; }
    IkStack stack;
    Calibrator cal;
    if (!stack.Init(sk) || !cal.Init(sk)) { std::printf("bones missing\n"); return 2; }
    lab::Recording rec;
    if (!lab::LoadRecording(recPath, &rec) || rec.frames.size() < 2) { std::printf("cannot read %s\n", recPath); return 2; }
    const char* devName[kCalCount + 1] = {"waist", "left foot", "right foot", "left hand", "right hand", "head"};
    // The bones whose frame-to-frame rotation change is measured: legs, spine, arms.
    std::vector<int> watched;
    for (const char* role : {"hips", "left_upleg", "left_leg", "left_foot", "right_upleg", "right_leg", "right_foot", "spine0", "spine1",
                             "spine2", "spine3", "neck", "neck1", "head", "left_upperarm", "left_forearm", "left_hand",
                             "right_upperarm", "right_forearm", "right_hand"})
        if (const int b = sk.Role(role); b >= 0) watched.push_back(b);
    struct { float legMax = 0, headMax = 0, shiftMax = 0; double shiftSum = 0; int legOver = 0, legSoft = 0, headOver1 = 0, headOver3 = 0,
             shiftOver2 = 0, frames = 0; } reachStat;
    struct FrameInfo {
        int index;
        double t;
        float boneJump;
        int bone;
        float devJumpCm, devJumpDeg;
        int dev;
        bool lost;
        float legReach[2], armReach[2];
    };
    std::vector<FrameInfo> infos;
    int lostCount[kCalCount + 1]{};
    float maxDevCm[kCalCount + 1]{}, maxDevDeg[kCalCount + 1]{};
    StackOptions o;
    FilterOptions fo;
    fo.enabled = filtered;
    ApplyReplayKnobs(o, fo, cal);
    StackDebug dbg;
    Pose prev;
    // The devices go through the same filter as in the lab (not the headset); the device steps reported below are the
    // filtered ones, so the classification tells what is left after it.
    PoseFilter filters[kCalCount];
    std::vector<lab::RecFrame> frames = rec.frames;
    // The hands from the grip poses (the lab's default); FBT_HANDS_WRIST: from the recorded hand-tracking wrist joints.
    const bool fromWrist = EnvFlag("FBT_HANDS_WRIST");
    int wristFrames = 0;
    for (size_t i = 0; i < frames.size() && fromWrist; ++i)
        for (int h = 0; h < 2; ++h)
            if (rec.frames[i].wrist[h].valid) {
                rec.frames[i].dev[h == 0 ? kCalLeftHand : kCalRightHand] = rec.frames[i].wrist[h];
                wristFrames += h == 0;
            }
    std::printf("hands: %s (%d frames with the left wrist joint)\n", fromWrist ? "wrist joints where tracked" : "grip poses", wristFrames);
    // The arms: the recorded arms-out lengths (v2 with the arms step); else, for a recording captured in the T-pose
    // (v1), the first frame's hands are out to the sides and measure them; else V's arms as they are.
    if (!rec.armsOut) {
        rec.calibration.armLength[0] = rec.calibration.armLength[1] = 0;   // a hanging length is not a scale
        if (!rec.armsDownFile && !rec.frames.empty()) cal.MeasureArmsOut(rec.calibration, rec.frames[0].head, rec.frames[0].dev);
    }
    if (!rec.armsDownFile && !rec.frames.empty()) {   // a T-pose capture: both arm measurements on frame 0, for comparison
        Calibration span = rec.calibration, each = rec.calibration;
        const bool a = cal.MeasureArmsOut(span, rec.frames[0].head, rec.frames[0].dev);
        const bool l = cal.MeasureArmOut(each, rec.frames[0].head, rec.frames[0].dev, 0);
        const bool r = cal.MeasureArmOut(each, rec.frames[0].head, rec.frames[0].dev, 1);
        std::printf("arms on the T-pose frame: span %s %.3f m; each from the neck's shoulder: left %s %.3f, right %s %.3f m\n",
                    a ? "ok" : "--", span.armLength[0], l ? "ok" : "--", each.armLength[0], r ? "ok" : "--", each.armLength[1]);
    }
    o.armScale[0] = cal.ArmScale(rec.calibration, 0);
    o.armScale[1] = cal.ArmScale(rec.calibration, 1);
    if (EnvFlag("FBT_NO_CLAVICLE")) o.clavicle = false;
    if (const char* e = std::getenv("FBT_ARM_SCALE")) o.armScale[0] = o.armScale[1] = static_cast<float>(std::atof(e));   // as if measured
    std::printf("player's arms (V's scale): %.3f %.3f m -> V's arms x %.3f %.3f\n", rec.calibration.armLength[0], rec.calibration.armLength[1], o.armScale[0], o.armScale[1]);
    PoseFilter headFilter;
    const FilterOptions headOptions = HeadGoalFilter();
    for (size_t i = 0; i < frames.size(); ++i) {
        const float dt = i > 0 ? static_cast<float>(frames[i].t - rec.frames[i - 1].t) : 0.0f;
        for (int d = 0; d < kCalCount; ++d) frames[i].dev[d] = filters[d].Filter(rec.frames[i].dev[d], dt, fo);
        if (filtered) frames[i].head = headFilter.Filter(rec.frames[i].head, dt, headOptions);
    }
    rec.frames = frames;
    // Arms: the wrist's bend and twist, the elbow's flexion, how often each passes a human range; the elbow's height
    // over the wrist with the hand in front (and palm up); V's elbow bend where the player's arm is near straight; the
    // clavicle's swing.
    struct ArmStat {
        float maxBend = 0, maxTwist = 0, maxFlex = 0;
        int bendOver = 0, twistOver = 0;
        int front = 0; double frontRise = 0; float frontRiseMax = -1e9f;         // hand > 0.3 m in front of the shoulder
        int frontUp = 0; double frontUpRise = 0; float frontUpRiseMax = -1e9f;   // ...and palm up
        int straight = 0; double straightFlex = 0; int fullStraight = 0; double fullStraightFlex = 0;
        float clavMax = 0; double clavSum = 0; int frames = 0;
    } arm[2];
    int armBone[2][4];
    Vec3 handAlong[2];
    float refBend[2]{};
    for (int s = 0; s < 2; ++s) {
        const bool left = s == 0;
        armBone[s][0] = sk.Role(left ? "left_upperarm" : "right_upperarm");
        armBone[s][1] = sk.Role(left ? "left_forearm" : "right_forearm");
        armBone[s][2] = sk.Role(left ? "left_hand" : "right_hand");
        armBone[s][3] = sk.Role(left ? "left_hand_end" : "right_hand_end");
        const Bone &h = sk.BoneAt(armBone[s][2]), &e = sk.BoneAt(armBone[s][3]), &f = sk.BoneAt(armBone[s][1]);
        handAlong[s] = Rotate(Conjugate(h.refModelRot), Normalize(e.refModelPos - h.refModelPos));
        refBend[s] = std::acos(std::fmin(1.0f, Dot(Normalize(e.refModelPos - h.refModelPos), Normalize(h.refModelPos - f.refModelPos)))) / kDeg;
    }
    for (size_t i = 0; i < rec.frames.size(); ++i) {
        const lab::RecFrame& fr = rec.frames[i];
        Goal goals[kGoalCount];
        cal.Goals(rec.calibration, fr.head, fr.dev, goals);
        Pose p = sk.ReferencePose();
        stack.Solve(p, goals, o, &dbg);
        {   // Out of reach: the legs (hip joint -> ankle goal over the leg's length) and the head (its miss after the spine).
            for (int s = 0; s < 2; ++s)
                if (goals[s == 0 ? kGoalLeftFoot : kGoalRightFoot].active) {
                    reachStat.legMax = std::fmax(reachStat.legMax, dbg.legReach[s]);
                    reachStat.legOver += dbg.legReach[s] > 1.0f;
                    reachStat.legSoft += dbg.legReach[s] > 0.97f;
                }
            {
                const float shift = Length(dbg.hipsShift);
                reachStat.shiftMax = std::fmax(reachStat.shiftMax, shift);
                reachStat.shiftSum += shift;
                reachStat.shiftOver2 += shift > 0.02f;
                ++reachStat.frames;
            }
            if (goals[kGoalHead].active) {
                reachStat.headMax = std::fmax(reachStat.headMax, dbg.spineError);
                reachStat.headOver1 += dbg.spineError > 0.01f;
                reachStat.headOver3 += dbg.spineError > 0.03f;
            }
        }
        if (const char* range = std::getenv("FBT_DUMP_FRAMES")) {   // "from-to": per-frame detail
            int from = 0, to = -1;
            if (sscanf_s(range, "%d-%d", &from, &to) == 2 && static_cast<int>(i) >= from && static_cast<int>(i) <= to) {
                const int neck1 = sk.Role("neck1"), spine0 = sk.Role("spine0");
                const float headStep = i > 0 ? Length(fr.head.pos - rec.frames[i - 1].head.pos) * 100 : 0;
                std::printf("  frame %d: head step %.1f cm, head goal %.3f m from Spine, spine reach %.3f, passes %d, error %.1f mm, "
                            "Neck1 %+.3f %+.3f %+.3f %+.3f\n", static_cast<int>(i), headStep, Length(goals[kGoalHead].pos - p.modelPos[spine0]),
                            dbg.spineReach, dbg.spineIterations, dbg.spineError * 1000, p.localRot[neck1].x, p.localRot[neck1].y,
                            p.localRot[neck1].z, p.localRot[neck1].w);
                for (int s = 0; s < 2; ++s) {
                    const int el = sk.Role(s == 0 ? "left_forearm" : "right_forearm"), wr = sk.Role(s == 0 ? "left_hand" : "right_hand");
                    const int sh = sk.Role(s == 0 ? "left_upperarm" : "right_upperarm");
                    // The palm's normal: V's T-pose palm faces down (-z), carried by the hand goal; +1 = palm up.
                    const Quat& hr = goals[s == 0 ? kGoalLeftHand : kGoalRightHand].rot;
                    const Vec3 palm = Rotate(hr, Rotate(Conjugate(sk.BoneAt(wr).refModelRot), Vec3{0, 0, -1}));
                    const Vec3 hs = p.modelPos[wr] - p.modelPos[sh];
                    const float flex = 180.0f - std::acos(std::fmax(-1.0f, std::fmin(1.0f, Dot(Normalize(p.modelPos[sh] - p.modelPos[el]), Normalize(p.modelPos[wr] - p.modelPos[el]))))) / kDeg;
                    std::printf("      %s arm: palm up %+.2f  reach %.2f  ext %.2f clav %4.1f  elbow dir (%+.2f %+.2f %+.2f) flex %5.1f  elbow - hand height %+5.1f cm  hand from shoulder (%+.2f %+.2f %+.2f)"
                                "  twist %+6.1f  roll %+5.1f  bend %5.1f%s\n", s == 0 ? "left " : "right", palm.z, dbg.armReach[s], dbg.armExtension[s], dbg.clavicleDeg[s], dbg.elbowDir[s].x,
                                dbg.elbowDir[s].y, dbg.elbowDir[s].z, flex, (p.modelPos[el].z - p.modelPos[wr].z) * 100,
                                hs.x, hs.y, hs.z, dbg.twistDeg[s], dbg.elbowRollDeg[s], dbg.wristBendDeg[s],
                                dbg.wristClamped[s] ? " clamped" : "");
                }
            }
        }
        if (i > 0) {
            const lab::RecFrame& pf = rec.frames[i - 1];
            FrameInfo fi{static_cast<int>(i), fr.t, 0, -1, 0, 0, -1, false, {dbg.legReach[0], dbg.legReach[1]}, {dbg.armReach[0], dbg.armReach[1]}};
            for (int b : watched) {
                const float j = AngleBetween(prev.modelRot[b], p.modelRot[b]);
                if (j > fi.boneJump) { fi.boneJump = j; fi.bone = b; }
            }
            for (int d = 0; d <= kCalCount; ++d) {
                const RoomPose& a = d < kCalCount ? pf.dev[d] : pf.head;
                const RoomPose& b = d < kCalCount ? fr.dev[d] : fr.head;
                if (a.valid != b.valid) { fi.lost = true; ++lostCount[d]; fi.dev = d; continue; }
                if (!a.valid) continue;
                const float cm = Length(b.pos - a.pos) * 100.0f, deg = AngleBetween(a.rot, b.rot);
                maxDevCm[d] = std::fmax(maxDevCm[d], cm);
                maxDevDeg[d] = std::fmax(maxDevDeg[d], deg);
                if (cm + deg * 0.25f > fi.devJumpCm + fi.devJumpDeg * 0.25f && !fi.lost) { fi.devJumpCm = cm; fi.devJumpDeg = deg; fi.dev = d; }
            }
            infos.push_back(fi);
        }
        for (int s = 0; s < 2; ++s) {
            if (!fr.dev[s == 0 ? kCalLeftHand : kCalRightHand].valid) continue;
            const Vec3 sh = p.modelPos[armBone[s][0]], el = p.modelPos[armBone[s][1]], wr = p.modelPos[armBone[s][2]];
            const Vec3 hand = Rotate(p.modelRot[armBone[s][2]], handAlong[s]);
            const float bend = std::fabs(std::acos(std::fmin(1.0f, Dot(hand, Normalize(wr - el)))) / kDeg - refBend[s]);
            const float flex = 180.0f - std::acos(std::fmax(-1.0f, std::fmin(1.0f, Dot(Normalize(sh - el), Normalize(wr - el))))) / kDeg;
            const float twist = std::fabs(dbg.twistDeg[s]);
            arm[s].maxBend = std::fmax(arm[s].maxBend, bend);
            arm[s].maxTwist = std::fmax(arm[s].maxTwist, twist);
            arm[s].maxFlex = std::fmax(arm[s].maxFlex, flex);
            arm[s].bendOver += bend > 80.0f;
            arm[s].twistOver += twist > 90.0f;
            ++arm[s].frames;
            arm[s].clavSum += dbg.clavicleDeg[s];
            arm[s].clavMax = std::fmax(arm[s].clavMax, dbg.clavicleDeg[s]);
            if (dbg.armExtension[s] > 0.97f) { ++arm[s].straight; arm[s].straightFlex += flex; }   // the player's arm nearly straight
            if (dbg.armExtension[s] > 0.995f) { ++arm[s].fullStraight; arm[s].fullStraightFlex += flex; }
            if (wr.y - sh.y > 0.3f) {   // the elbow's height over the wrist's, hands in front
                ++arm[s].front;
                arm[s].frontRise += el.z - wr.z;
                arm[s].frontRiseMax = std::fmax(arm[s].frontRiseMax, el.z - wr.z);
                const Vec3 palm = Rotate(p.modelRot[armBone[s][2]], Rotate(Conjugate(sk.BoneAt(armBone[s][2]).refModelRot), Vec3{0, 0, -1}));
                if (palm.z > 0.5f) {
                    ++arm[s].frontUp;
                    arm[s].frontUpRise += el.z - wr.z;
                    arm[s].frontUpRiseMax = std::fmax(arm[s].frontUpRiseMax, el.z - wr.z);
                }
            }
        }
        prev = p;
    }
    const double duration = rec.frames.back().t - rec.frames.front().t;
    std::printf("recording %s: %d frames, %.1f s (%.0f fps); calibration scale %.4f; filter %s\n", recPath,
                static_cast<int>(rec.frames.size()), duration, duration > 0 ? rec.frames.size() / duration : 0, rec.calibration.scale,
                filtered ? "ON" : "off");
    if (filtered)
        std::printf("  rejected one-frame jumps: waist %d, left foot %d, right foot %d, left hand %d, right hand %d\n", filters[0].Rejected(),
                    filters[1].Rejected(), filters[2].Rejected(), filters[3].Rejected(), filters[4].Rejected());
    if (filtered && headFilter.Rejected()) std::printf("  head goal held %d frames (impossible steps: a placeholder pose)\n", headFilter.Rejected());
    std::printf("  lost frames held: waist %d, left foot %d, right foot %d, left hand %d, right hand %d\n", filters[0].LostFrames(),
                filters[1].LostFrames(), filters[2].LostFrames(), filters[3].LostFrames(), filters[4].LostFrames());
    for (int d = 0; d <= kCalCount; ++d)
        std::printf("  %-10s lost/found %3d times   biggest frame step %6.1f cm %6.1f deg\n", devName[d], lostCount[d], maxDevCm[d], maxDevDeg[d]);
    for (int s = 0; s < 2; ++s)
        std::printf("  %-10s wrist bend max %5.1f deg (> 80: %d frames)  wrist twist max %5.1f deg (> 90: %d frames)  elbow flexion max %5.1f deg\n",
                    s == 0 ? "left arm" : "right arm", arm[s].maxBend, arm[s].bendOver, arm[s].maxTwist, arm[s].twistOver, arm[s].maxFlex);
    for (int s = 0; s < 2; ++s)
        if (arm[s].front)
            std::printf("  %-10s hand > 0.3 m in front: %d frames, elbow above the wrist by %+.1f cm on average, %+.1f cm at most\n",
                        s == 0 ? "left arm" : "right arm", arm[s].front, arm[s].frontRise / arm[s].front * 100, arm[s].frontRiseMax * 100);
    for (int s = 0; s < 2; ++s)
        std::printf("  %-10s player's arm > 97 %% extended: %d frames, V's elbow bent %.1f deg on average; > 99.5 %%: %d frames, %.1f deg; clavicle %.1f deg on average, %.1f at most\n",
                    s == 0 ? "left arm" : "right arm", arm[s].straight, arm[s].straight ? arm[s].straightFlex / arm[s].straight : 0.0,
                    arm[s].fullStraight, arm[s].fullStraight ? arm[s].fullStraightFlex / arm[s].fullStraight : 0.0,
                    arm[s].frames ? arm[s].clavSum / arm[s].frames : 0.0, arm[s].clavMax);
    for (int s = 0; s < 2; ++s)
        if (arm[s].frontUp)
            std::printf("  %-10s   of which palm up: %d frames, elbow above the wrist by %+.1f cm on average, %+.1f cm at most\n",
                        s == 0 ? "left arm" : "right arm", arm[s].frontUp, arm[s].frontUpRise / arm[s].frontUp * 100, arm[s].frontUpRiseMax * 100);
    {
        int over90 = 0, over30 = 0;
        for (const auto& fi : infos) { over90 += fi.boneJump > 90.0f; over30 += fi.boneJump > 30.0f; }
        std::printf("  out of reach: leg samples > 0.97 (soft) %d, > 1 %d, max %.3f; head missed > 1 cm %d, > 3 cm %d frames, max %.1f cm\n",
                    reachStat.legSoft, reachStat.legOver, reachStat.legMax, reachStat.headOver1, reachStat.headOver3, reachStat.headMax * 100);
        std::printf("  hips gave way: %.1f cm on average, %.1f cm at most, > 2 cm in %d frames\n",
                    reachStat.frames ? reachStat.shiftSum / reachStat.frames * 100 : 0.0, reachStat.shiftMax * 100, reachStat.shiftOver2);
        std::printf("  frames with a bone step > 30 deg: %d, > 90 deg (flips): %d\n", over30, over90);
    }
    std::sort(infos.begin(), infos.end(), [](const FrameInfo& a, const FrameInfo& b) { return a.boneJump > b.boneJump; });
    std::printf("worst frames (biggest bone rotation change from the previous frame):\n");
    for (size_t k = 0; k < infos.size() && k < 20; ++k) {
        const FrameInfo& fi = infos[k];
        // Input: a device lost/found or stepping more than 5 cm or 15 deg in one frame. Reach: a limb at >= 0.98 of its
        // length (straightening is where 1 mm moves a joint by degrees). Otherwise the solver jumped on smooth input.
        const bool input = fi.lost || fi.devJumpCm > 5.0f || fi.devJumpDeg > 15.0f;
        // The reach of the limb the bone belongs to (legs, arms by name); the spine has none.
        const std::string& bn = fi.bone >= 0 ? sk.BoneAt(fi.bone).name : std::string();
        const bool left = bn.rfind("Left", 0) == 0, right = bn.rfind("Right", 0) == 0;
        const bool legBone = bn.find("Leg") != std::string::npos || bn.find("Foot") != std::string::npos;
        const float reach = !(left || right) ? 0.0f : legBone ? fi.legReach[left ? 0 : 1] : fi.armReach[left ? 0 : 1];
        const char* verdict = input ? "INPUT " : reach >= 0.98f ? "REACH " : "SOLVER";
        std::printf("  #%6d %7.2f s  %6.1f deg %-16s %s  device: %-10s %s%5.1f cm %5.1f deg  reach legs %.2f %.2f arms %.2f %.2f\n", fi.index,
                    fi.t, fi.boneJump, fi.bone >= 0 ? sk.BoneAt(fi.bone).name.c_str() : "-", verdict, fi.dev >= 0 ? devName[fi.dev] : "-",
                    fi.lost ? "LOST " : "", fi.devJumpCm, fi.devJumpDeg, fi.legReach[0], fi.legReach[1], fi.armReach[0], fi.armReach[1]);
    }
    return 0;
}

// --mesh <rig> <glb>...: V's body skinned to the reference pose, each rigid part's distance to its joint kept.
int MeshCheck(int argc, char** argv) {
    Skeleton sk;
    std::string err;
    if (!sk.Load(argv[2], &err)) { std::printf("%s\n", err.c_str()); return 2; }
    lab::VMesh mesh;
    const float c[3] = {1, 1, 1};
    for (int i = 3; i < argc; ++i)
        if (!mesh.Load(argv[i], sk, c, &err)) std::printf("%s\n", err.c_str());
    Pose p = sk.ReferencePose();
    sk.Fk(p);
    mesh.Skin(p);
    int n = 0;
    const float worst = mesh.RigidCheck(p, &n);
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    for (const auto& v : mesh.Vertices())
        for (int k = 0; k < 3; ++k) { lo[k] = std::fmin(lo[k], v.pos[k]); hi[k] = std::fmax(hi[k], v.pos[k]); }
    std::printf("mesh: %d vertices, %d joints not in the rig; T-pose bounds x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f\n",
                static_cast<int>(mesh.Vertices().size()), mesh.UnmatchedJoints(), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
    std::printf("%-34s %s  %d single-joint vertices, distance to their joint kept within %.2e m\n", "mesh rigid parts (T-pose)",
                worst < 1e-3f ? "PASS" : "FAIL", n, worst);
    // Orientation (a turn about a bone's own axis keeps every distance, and a bone's vertices centre on it): flesh off to
    // one side of its bone. The trunk's sits in front of or behind the spine bones, never to the side (a turn about the
    // spine's vertical axis would move it sideways); the foot's (heel, arch) forward of the ankle and below it.
    bool ok = worst < 1e-3f;
    for (const char* name : {"Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck"}) {
        const int bone = sk.Find(name);
        Vec3 c;
        if (bone < 0 || !mesh.Centroid(bone, &c)) continue;
        const Vec3 d = c - p.modelPos[bone];
        const bool good = std::fabs(d.x) < 0.01f && std::fabs(d.y) > 2 * std::fabs(d.x);
        ok = ok && good;
        std::printf("%-34s %s  its flesh from the joint (%+.3f %+.3f %+.3f): front/back, not sideways\n", (std::string(name) + " (T-pose)").c_str(),
                    good ? "PASS" : "FAIL", d.x, d.y, d.z);
    }
    for (const char* name : {"LeftFoot", "RightFoot"}) {
        const int bone = sk.Find(name);
        Vec3 c;
        if (bone < 0 || !mesh.Centroid(bone, &c)) continue;
        const Vec3 d = c - p.modelPos[bone];
        const bool good = d.y > 0.01f && d.z < 0 && std::fabs(d.x) < 0.03f;
        ok = ok && good;
        std::printf("%-34s %s  its flesh from the ankle (%+.3f %+.3f %+.3f): forward and below\n", (std::string(name) + " (T-pose)").c_str(),
                    good ? "PASS" : "FAIL", d.x, d.y, d.z);
    }
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc >= 4 && std::string(argv[1]) == "--mesh") return MeshCheck(argc, argv);
    if (argc >= 4 && std::string(argv[1]) == "--replay")
        return Replay(argv[2], argv[3], !(argc >= 5 && std::string(argv[4]) == "--nofilter"));
    if (argc < 2) {
        std::printf("usage: fbt_ik_test <rig file>\n       fbt_ik_test --replay <recording> <rig file> [--nofilter]\n       fbt_ik_test --mesh <rig file> <glb>...\n");
        return 2;
    }
    Rig r;
    std::string err;
    if (!r.sk.Load(argv[1], &err)) { std::printf("%s\n", err.c_str()); return 2; }
    if (!r.stack.Init(r.sk)) { std::printf("IK bones missing\n"); return 2; }
    r.hips = r.sk.Role("hips");
    const char* sides[2] = {"left", "right"};
    for (int s = 0; s < 2; ++s) {
        r.upLeg[s] = r.sk.Role((std::string(sides[s]) + "_upleg").c_str());
        r.leg[s] = r.sk.Role((std::string(sides[s]) + "_leg").c_str());
        r.foot[s] = r.sk.Role((std::string(sides[s]) + "_foot").c_str());
        r.thighSide[s] = Rotate(Conjugate(r.sk.BoneAt(r.upLeg[s]).refModelRot), {1, 0, 0});
        r.shinSide[s] = Rotate(Conjugate(r.sk.BoneAt(r.leg[s]).refModelRot), {1, 0, 0});
        r.footFwd[s] = Rotate(Conjugate(r.sk.BoneAt(r.foot[s]).refModelRot), {0, 1, 0});
        r.footSide[s] = Rotate(Conjugate(r.sk.BoneAt(r.foot[s]).refModelRot), {1, 0, 0});
        r.a[s] = Length(r.sk.BoneAt(r.leg[s]).refLocalPos);
        r.b[s] = Length(r.sk.BoneAt(r.foot[s]).refLocalPos);
    }
    for (const char* role : {"spine0", "spine1", "spine2", "spine3", "spine4", "spine5", "spine6", "spine7", "neck", "neck1", "head"})
        if (const int b = r.sk.Role(role); b >= 0) r.spine[r.spineCount++] = b;
    MakeTwoBone(r.sk, r.sk.Role("left_upperarm"), r.sk.Role("left_forearm"), r.sk.Role("left_hand"), {0, -1, 0}, &r.arm[0]);
    MakeTwoBone(r.sk, r.sk.Role("right_upperarm"), r.sk.Role("right_forearm"), r.sk.Role("right_hand"), {0, -1, 0}, &r.arm[1]);
    std::printf("rig %s: %d bones; thigh %.3f m, shin %.3f m; spine chain %d joints; upper arm %.3f m, forearm %.3f m\n", argv[1],
                r.sk.Count(), r.a[0], r.b[0], r.spineCount, r.arm[0].a, r.arm[0].b);
    StackOptions o;
    Goal g[kGoalCount];
    char label[128];

    StackDebug dbg;
    auto solve = [&](Pose& p) { p = r.sk.ReferencePose(); r.stack.Solve(p, g, o, &dbg); };
    Pose p;
    g_debug = &dbg;

    // Foot rotations at a slightly crouched stance (hips 8 cm down so the legs bend): yaw, pitch, roll sweeps.
    struct { const char* name; Vec3 axis; float from, to, limit; } footSweeps[] = {
        {"left foot yaw -90..90", {0, 0, 1}, -90, 90, 3.0f},
        {"left foot pitch -60..60", {1, 0, 0}, -60, 60, 3.0f},
        {"left foot roll -40..40", {0, 1, 0}, -40, 40, 3.0f},
        {"left foot roll 0..90 (to fallback)", {0, 1, 0}, 0, 90, 3.0f},
        {"left foot roll 0..-90 (to fallback)", {0, 1, 0}, -90, 0, 3.0f},
    };
    for (const auto& fs : footSweeps) {
        Worst w;
        Sweep sw;
        for (float a = fs.from; a <= fs.to; a += 0.5f) {
            ResetGoals(r, g);
            g[kGoalHips].pos.z -= 0.08f;
            g[kGoalLeftFoot].rot = Normalize(AxisAngle(fs.axis, a * kDeg) * g[kGoalLeftFoot].rot);
            solve(p);
            sprintf_s(label, "%.1f deg", a);
            Check(r, p, g, o, w, label);
            sw.Step(r, p, label);
        }
        Report(fs.name, w);
        ReportSweep("   continuity", sw, fs.limit);
    }
    // Crouch: hips straight down 0.96 -> 0.35 m, 2 mm steps.
    {
        Worst w;
        Sweep sw;
        ResetGoals(r, g);
        const float z0 = g[kGoalHips].pos.z;
        for (float dz = 0; dz <= z0 - 0.35f; dz += 0.002f) {
            ResetGoals(r, g);
            g[kGoalHips].pos.z = z0 - dz;
            solve(p);
            sprintf_s(label, "hips z %.3f", z0 - dz);
            Check(r, p, g, o, w, label);
            sw.Step(r, p, label);
        }
        Report("crouch 0.96 -> 0.35 m", w);
        ReportSweep("   continuity", sw, 3.0f);
    }
    // Steps: left foot forward/back and up, hips 5 cm down.
    {
        Worst w;
        Sweep sw;
        for (float up = 0; up <= 0.40f; up += 0.10f) {
            sw.have = false;
            for (float fwd = -0.45f; fwd <= 0.60f; fwd += 0.005f) {
                ResetGoals(r, g);
                g[kGoalHips].pos.z -= 0.05f;
                g[kGoalLeftFoot].pos.y += fwd;
                g[kGoalLeftFoot].pos.z += up;
                solve(p);
                sprintf_s(label, "fwd %.3f up %.2f", fwd, up);
                Check(r, p, g, o, w, label);
                sw.Step(r, p, label);
            }
        }
        Report("left foot steps", w);
        ReportSweep("   continuity", sw, 3.0f);
    }
    // Hips yaw with the feet planted, crouched 10 cm.
    {
        Worst w;
        Sweep sw;
        for (float a = -90; a <= 90; a += 0.5f) {
            ResetGoals(r, g);
            g[kGoalHips].pos.z -= 0.10f;
            g[kGoalHips].rot = Normalize(AxisAngle({0, 0, 1}, a * kDeg) * g[kGoalHips].rot);
            solve(p);
            sprintf_s(label, "%.1f deg", a);
            Check(r, p, g, o, w, label);
            sw.Step(r, p, label);
        }
        Report("hips yaw -90..90 (feet planted)", w);
        ReportSweep("   continuity", sw, 3.0f);
    }
    // Head sweeps (the spine limb): lean forward/back and sideways on an arc, look around, head down with the hips fixed,
    // hips moving under a fixed head.
    {
        struct { const char* name; int kind; float from, to, step; } headSweeps[] = {
            {"head lean forward/back -60..60", 0, -60, 60, 0.5f},
            {"head lean sideways -40..40", 1, -40, 40, 0.5f},
            {"head yaw -80..80 (look around)", 2, -80, 80, 0.5f},
            {"head down 0..0.45 m (hips fixed)", 3, 0, 0.45f, 0.002f},
            {"hips -0.3..0.2 m under a fixed head", 4, -0.3f, 0.2f, 0.002f},
            {"slouch: lean 0..45 + 5 cm closer", 5, 0, 45, 0.25f},
            {"slouch: lean 0..30 + 10 cm closer", 6, 0, 30, 0.25f},
        };
        for (const auto& hs : headSweeps) {
            Worst w;
            Sweep sw;
            for (float v = hs.from; v <= hs.to + 1e-6f; v += hs.step) {
                ResetGoals(r, g);
                const Vec3 pivot = r.sk.BoneAt(r.spine[0]).refModelPos;
                Goal& h = g[kGoalHead];
                if (hs.kind == 0 || hs.kind == 1) {   // the head on an arc around the spine root, turned with it
                    const Quat q = AxisAngle(hs.kind == 0 ? Vec3{1, 0, 0} : Vec3{0, 1, 0}, (hs.kind == 0 ? -v : v) * kDeg);
                    h.pos = pivot + Rotate(q, h.pos - pivot);
                    h.rot = Normalize(q * h.rot);
                } else if (hs.kind == 2) {
                    h.rot = Normalize(AxisAngle({0, 0, 1}, v * kDeg) * h.rot);
                } else if (hs.kind == 3) {
                    h.pos.z -= v;
                } else if (hs.kind == 4) {
                    g[kGoalHips].pos.z += v * 0.5f;
                    g[kGoalHips].pos.y += v * 0.3f;
                } else {
                    // Slouch: the head leans forward on the arc and comes closer to the spine root as it does, up to
                    // 5 (or 10) cm at the end of the lean -- a hunch, not a telescope.
                    const float shorten = (hs.kind == 5 ? 0.05f : 0.10f) * (v / hs.to);
                    const Quat q = AxisAngle({1, 0, 0}, -v * kDeg);
                    const Vec3 arm = Rotate(q, h.pos - pivot);
                    h.pos = pivot + arm * ((Length(arm) - shorten) / Length(arm));
                    h.rot = Normalize(q * h.rot);
                }
                solve(p);
                sprintf_s(label, "%.3f", v);
                Check(r, p, g, o, w, label);
                sw.Step(r, p, label);
            }
            Report(hs.name, w);
            ReportSweep("   continuity", sw, 3.0f);
        }
    }
    // Arm sweeps (right arm): the hand placed relative to the right shoulder (model space), rotations about the arm.
    {
        const Vec3 shoulder = r.sk.BoneAt(r.arm[1].upper).refModelPos;
        const Quat handRef = r.sk.BoneAt(r.arm[1].end).refModelRot;
        const float reach = (r.arm[1].a + r.arm[1].b) * 0.8f;
        struct { const char* name; int kind; float from, to, step; } armSweeps[] = {
            {"right forearm roll -90..90 (front)", 0, -90, 90, 0.5f},
            {"right reach arc T -> front -> up", 1, 0, 180, 0.5f},
            {"right hand across the chest", 2, 0, 1, 0.0025f},
            {"right hand down and behind", 3, 0, 1, 0.0025f},
        };
        for (const auto& as : armSweeps) {
            Worst w;
            Sweep sw;
            for (float v = as.from; v <= as.to + 1e-6f; v += as.step) {
                ResetGoals(r, g);
                Goal& h = g[kGoalRightHand];
                if (as.kind == 0) {        // hand forward at chest height, palm turning about the forearm's line
                    h.pos = shoulder + Vec3{0.05f, reach, -0.10f};
                    const Quat toFront = AxisAngle({0, 0, 1}, 90 * kDeg);   // the T-pose hand (+x) turned to point forward
                    const Vec3 axis = Normalize(h.pos - shoulder);
                    h.rot = Normalize(AxisAngle(axis, v * kDeg) * toFront * handRef);
                } else if (as.kind == 1) { // from the side (T), through the front, to overhead
                    const float a = v * kDeg;
                    const Vec3 dir = a <= 90 * kDeg ? Vec3{std::cos(a), std::sin(a), 0} : Vec3{0, std::cos(a - 90 * kDeg), std::sin(a - 90 * kDeg)};
                    h.pos = shoulder + dir * reach;
                    h.rot = Normalize(FromTo({1, 0, 0}, dir) * handRef);
                } else if (as.kind == 2) { // in front of the chest towards the other shoulder
                    const Vec3 a = shoulder + Vec3{0.05f, reach, -0.10f}, b = shoulder + Vec3{-0.40f, 0.25f, -0.05f};
                    h.pos = a + (b - a) * v;
                    h.rot = Normalize(FromTo({1, 0, 0}, Normalize(h.pos - shoulder)) * handRef);
                } else {                   // down by the side, then behind the back
                    const Vec3 a = shoulder + Vec3{0.08f, 0, -reach}, b = shoulder + Vec3{-0.05f, -0.30f, -0.40f};
                    h.pos = a + (b - a) * v;
                    h.rot = Normalize(FromTo({1, 0, 0}, Normalize(h.pos - shoulder)) * handRef);
                }
                solve(p);
                sprintf_s(label, "%.3f", v);
                Check(r, p, g, o, w, label);
                sw.Step(r, p, label);
            }
            Report(as.name, w);
            ReportSweep("   continuity", sw, 3.0f);
        }
    }
    // Random goals in a plausible range.
    {
        Worst w;
        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> u(-1, 1);
        for (int i = 0; i < 20000; ++i) {
            ResetGoals(r, g);
            g[kGoalHips].pos = g[kGoalHips].pos + Vec3{u(rng) * 0.15f, u(rng) * 0.15f, -std::fabs(u(rng)) * 0.45f};
            g[kGoalHips].rot = Normalize(AxisAngle({0, 0, 1}, u(rng) * 60 * kDeg) * AxisAngle({1, 0, 0}, u(rng) * 30 * kDeg) * g[kGoalHips].rot);
            g[kGoalHead].pos = g[kGoalHead].pos + Vec3{u(rng) * 0.25f, u(rng) * 0.35f, -std::fabs(u(rng)) * 0.40f};
            g[kGoalHead].rot = Normalize(AxisAngle({0, 0, 1}, u(rng) * 70 * kDeg) * AxisAngle({1, 0, 0}, u(rng) * 40 * kDeg) * g[kGoalHead].rot);
            for (int s = 0; s < 2; ++s) {
                Goal& hg = g[s == 0 ? kGoalLeftHand : kGoalRightHand];
                const Vec3 sh = r.sk.BoneAt(r.arm[s].upper).refModelPos;
                const Vec3 dir = Normalize(Vec3{u(rng), u(rng), u(rng)});
                hg.pos = sh + dir * ((r.arm[s].a + r.arm[s].b) * (0.3f + 0.65f * std::fabs(u(rng))));
                hg.rot = Normalize(AxisAngle(Normalize(Vec3{u(rng), u(rng), u(rng)}), u(rng) * 180 * kDeg) * hg.rot);
            }
            for (int f : {kGoalLeftFoot, kGoalRightFoot}) {
                g[f].pos = g[f].pos + Vec3{u(rng) * 0.25f, u(rng) * 0.40f, std::fabs(u(rng)) * 0.35f};
                g[f].rot = Normalize(AxisAngle({0, 0, 1}, u(rng) * 50 * kDeg) * AxisAngle({1, 0, 0}, u(rng) * 40 * kDeg) *
                                     AxisAngle({0, 1, 0}, u(rng) * 25 * kDeg) * g[f].rot);
            }
            solve(p);
            sprintf_s(label, "random #%d", i);
            Check(r, p, g, o, w, label);
        }
        Report("random 20000", w);
    }
    // Calibration: a synthetic player standing exactly in V's T-pose, 5 % smaller, in a room turned 37 deg and shifted.
    // After the capture the goals must land on V's reference bones; after the player walks 1 m and turns 50 deg they
    // must be the same body moved rigidly.
    {
        Calibrator cal;
        if (!cal.Init(r.sk)) { std::printf("calibration: bones missing\n"); return 1; }
        const char* roles[kCalCount] = {"hips", "left_foot", "right_foot", "left_hand", "right_hand"};
        const GoalId goalOf[kCalCount] = {kGoalHips, kGoalLeftFoot, kGoalRightFoot, kGoalLeftHand, kGoalRightHand};
        const float playerScale = 1.0f / 1.05f;
        auto place = [&](Quat roomYaw, Vec3 roomShift, RoomPose& head, RoomPose dev[kCalCount]) {
            // Device rotations: arbitrary mounts (the device frame need not match the bone's).
            const Quat mount = Normalize(AxisAngle({0.3f, 1, 0.2f}, 1.1f));
            for (int i = 0; i < kCalCount; ++i) {
                const Bone& b = r.sk.BoneAt(r.sk.Role(roles[i]));
                dev[i].valid = true;
                dev[i].pos = Rotate(roomYaw, b.refModelPos * playerScale + Vec3{0.03f, 0.02f, 0.05f}) + roomShift;
                dev[i].rot = Normalize(roomYaw * b.refModelRot * mount);
                if (cal.handsFixedMount && (i == kCalLeftHand || i == kCalRightHand)) {
                    // The controllers in a T-pose grip (palm down, fist axis forward: the calibration's fixed mount), the
                    // wrist behind the grip along its +Y.
                    const bool left = i == kCalLeftHand;
                    const Quat grip = left ? FromBasis({0, 0, -1}, {0, 1, 0}, {1, 0, 0}) : FromBasis({0, 0, 1}, {0, 1, 0}, {-1, 0, 0});
                    dev[i].rot = Normalize(roomYaw * grip);
                    dev[i].pos = Rotate(roomYaw, b.refModelPos * playerScale - Rotate(grip, Vec3{0, 0, cal.handWristBack})) + roomShift;
                }
            }
            const Vec3 eye = r.sk.EyePoint();
            head.valid = true;
            head.pos = Rotate(roomYaw, eye * playerScale) + roomShift;
            head.rot = roomYaw;
        };
        RoomPose head, dev[kCalCount];
        place(AxisAngle({0, 0, 1}, 37 * kDeg), {1.2f, -0.5f, 0}, head, dev);
        Calibration c;
        const char* why = nullptr;
        if (!cal.Capture(head, dev, &c, &why)) { std::printf("calibration capture FAILED: %s\n", why); return 1; }
        Goal goals[kGoalCount];
        cal.Goals(c, head, dev, goals);
        float pe = 0, re2 = 0;
        for (int i = 0; i < kCalCount; ++i) {
            const Bone& b = r.sk.BoneAt(r.sk.Role(roles[i]));
            pe = std::fmax(pe, Length(goals[goalOf[i]].pos - b.refModelPos));
            re2 = std::fmax(re2, AngleBetween(goals[goalOf[i]].rot, b.refModelRot));
        }
        const Bone& hb = r.sk.BoneAt(r.sk.Role("head"));
        pe = std::fmax(pe, Length(goals[kGoalHead].pos - hb.refModelPos));
        re2 = std::fmax(re2, AngleBetween(goals[kGoalHead].rot, hb.refModelRot));
        std::printf("%-34s %s  scale %.4f  goals on V's reference bones: pos %.2e m  rot %.3f deg\n", "calibration capture (head-anchored)",
                    pe < 1e-4f && re2 < 0.1f ? "PASS" : "FAIL", c.scale, pe, re2);
        {   // The feet-midpoint origin for comparison: the trackers' mounts move it, and the head goal with it.
            Calibrator feet = cal;
            feet.headAnchored = false;
            Calibration cf;
            feet.Capture(head, dev, &cf, &why);
            Goal gf[kGoalCount];
            feet.Goals(cf, head, dev, gf);
            std::printf("%-34s INFO  head goal off V's head by %.3f m (foot mounts 3 cm side, 2 cm forward)\n",
                        "calibration capture (feet origin)", Length(gf[kGoalHead].pos - hb.refModelPos));
        }
        // The player walks 1 m and turns 50 deg: same body, moved rigidly.
        RoomPose head2, dev2[kCalCount];
        const Quat turn = AxisAngle({0, 0, 1}, 50 * kDeg);
        place(Normalize(turn * AxisAngle({0, 0, 1}, 37 * kDeg)), Vec3{1.2f, -0.5f, 0} + Vec3{0.6f, 0.8f, 0}, head2, dev2);
        Goal moved[kGoalCount];
        cal.Goals(c, head2, dev2, moved);
        float shape = 0, relRot = 0;
        for (int a = 0; a < kGoalCount; ++a)
            for (int b = 0; b < kGoalCount; ++b) {
                shape = std::fmax(shape, std::fabs(Length(moved[a].pos - moved[b].pos) - Length(goals[a].pos - goals[b].pos)));
                relRot = std::fmax(relRot, AngleBetween(Conjugate(moved[a].rot) * moved[b].rot, Conjugate(goals[a].rot) * goals[b].rot));
            }
        std::printf("%-34s %s  shape kept: distances %.2e m  relative rotations %.3f deg\n", "calibration: walk 1 m, turn 50",
                    shape < 1e-4f && relRot < 0.1f ? "PASS" : "FAIL", shape, relRot);
    }
    return 0;
}
