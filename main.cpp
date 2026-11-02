// FBT IK LAB: V's skeleton outside the game, to develop, check and report on the full-body IK (the port's src/Fbt)
// without launching the game. Loads the rig file the plugin writes when V's rig loads (fbt_rig_male.txt /
// fbt_rig_female.txt), draws V's body (the game's mesh, when exported next to it -- see README.md) and/or a robot of
// boxes (one per segment, rigid on its bone, so rotations show) and a stick figure, over a floor grid, orbit camera.
//   fbt_ik_lab.exe [rig file]            (default: fbt_rig_male.txt in the working folder)
// Without VR the six goals are moved by hand (gizmos). "Start VR" runs it in the headset (OpenXR; SteamVR's floor as
// the floor) with the real trackers, controllers and headset, a mirror, and a flight recorder whose files replay
// offline (fbt_ik_test --replay): a recording is a bug report. The window stays as a spectator view.
// Mouse: right drag = orbit, middle drag = pan, wheel = zoom; left click near a goal = select it, then drag its gizmo
// (W = move, E = rotate, R = reset it).
#include "BoxRenderer.hpp"
#include "Recording.hpp"
#include "Robot.hpp"
#include "VMesh.hpp"
#include "VrSession.hpp"
#include "Fbt/FbtCalibration.hpp"
#include "Fbt/FbtFilter.hpp"
#include "Fbt/FbtIk.hpp"
#include "Fbt/FbtSkeleton.hpp"
#include <d3d11.h>
#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>
#include <ImGuizmo.h>
#include <windows.h>
#include <shellapi.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

using namespace wrpuppets::fbt;

namespace {
ID3D11Device* g_device = nullptr;
ID3D11DeviceContext* g_context = nullptr;
IDXGISwapChain* g_swapChain = nullptr;
ID3D11RenderTargetView* g_rtv = nullptr;
UINT g_resizeW = 0, g_resizeH = 0;

void CreateRenderTarget() {
    ID3D11Texture2D* back = nullptr;
    g_swapChain->GetBuffer(0, IID_PPV_ARGS(&back));
    g_device->CreateRenderTargetView(back, nullptr, &g_rtv);
    back->Release();
}
void CleanupRenderTarget() {
    if (g_rtv) { g_rtv->Release(); g_rtv = nullptr; }
}
bool CreateDevice(HWND hwnd) {
    DXGI_SWAP_CHAIN_DESC sd{};
    sd.BufferCount = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hwnd;
    sd.SampleDesc.Count = 1;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0};
    D3D_FEATURE_LEVEL level{};
    if (FAILED(D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 1, D3D11_SDK_VERSION, &sd,
                                             &g_swapChain, &g_device, &level, &g_context)))
        return false;
    CreateRenderTarget();
    return true;
}
void CleanupDevice() {
    CleanupRenderTarget();
    if (g_swapChain) g_swapChain->Release();
    if (g_context) g_context->Release();
    if (g_device) g_device->Release();
}
LRESULT WINAPI WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wParam, lParam)) return true;
    switch (msg) {
    case WM_SIZE:
        if (wParam != SIZE_MINIMIZED) { g_resizeW = LOWORD(lParam); g_resizeH = HIWORD(lParam); }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) return 0;
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ---- camera: orbit around a target, model space z up; column-major 4x4 (OpenGL style) ----
struct Camera {
    Vec3 target{0, 0, 0.9f};
    float yaw = 0.6f, pitch = 0.25f, distance = 3.2f, fovY = 0.9f;
    Vec3 Eye() const {
        return target + Vec3{std::cos(pitch) * std::sin(yaw), std::cos(pitch) * std::cos(yaw), std::sin(pitch)} * distance;
    }
};
void LookAt(Vec3 eye, Vec3 target, float m[16]) {
    const Vec3 f = Normalize(target - eye), s = Normalize(Cross(f, {0, 0, 1})), u = Cross(s, f);
    const float r[16] = {s.x, u.x, -f.x, 0, s.y, u.y, -f.y, 0, s.z, u.z, -f.z, 0, -Dot(s, eye), -Dot(u, eye), Dot(f, eye), 1};
    std::copy(r, r + 16, m);
}
// Right-handed perspective, D3D clip depth 0..1.
void Perspective(float fovY, float aspect, float zNear, float zFar, float m[16]) {
    const float f = 1.0f / std::tan(fovY * 0.5f);
    const float r[16] = {f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, zFar / (zNear - zFar), -1, 0, 0, zNear * zFar / (zNear - zFar), 0};
    std::copy(r, r + 16, m);
}
void Multiply(const float a[16], const float b[16], float out[16]) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            float v = 0;
            for (int k = 0; k < 4; ++k) v += a[k * 4 + r] * b[c * 4 + k];
            out[c * 4 + r] = v;
        }
}
struct Projector {
    float view[16], proj[16];
    ImVec2 size;
    // A model-space point on the screen; false when behind the camera.
    bool operator()(Vec3 p, ImVec2* out) const {
        float v[4], c[4];
        const float in[4] = {p.x, p.y, p.z, 1};
        for (int r = 0; r < 4; ++r) v[r] = view[r] * in[0] + view[4 + r] * in[1] + view[8 + r] * in[2] + view[12 + r] * in[3];
        for (int r = 0; r < 4; ++r) c[r] = proj[r] * v[0] + proj[4 + r] * v[1] + proj[8 + r] * v[2] + proj[12 + r] * v[3];
        if (c[3] < 0.01f) return false;
        *out = ImVec2((c[0] / c[3] * 0.5f + 0.5f) * size.x, (0.5f - c[1] / c[3] * 0.5f) * size.y);
        return true;
    }
};

void Line3(ImDrawList* dl, const Projector& pr, Vec3 a, Vec3 b, ImU32 color, float thickness) {
    ImVec2 pa, pb;
    if (pr(a, &pa) && pr(b, &pb)) dl->AddLine(pa, pb, color, thickness);
}

// ---- goals: the poses the trackers, headset and hands will give, one per driven bone (in wrpuppets::fbt::GoalId order) ----
struct LabGoal {
    const char* label;
    const char* role;
    Vec3 pos;
    Quat rot;
};
void ToMatrix(Vec3 p, Quat q, float m[16]) {
    const Vec3 x = Rotate(q, {1, 0, 0}), y = Rotate(q, {0, 1, 0}), z = Rotate(q, {0, 0, 1});
    const float r[16] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, z.x, z.y, z.z, 0, p.x, p.y, p.z, 1};
    std::copy(r, r + 16, m);
}
// Position and rotation back from a column-major rigid matrix (columns re-orthonormalised).
void FromMatrix(const float m[16], Vec3* p, Quat* q) {
    *p = {m[12], m[13], m[14]};
    const Vec3 x = Normalize(Vec3{m[0], m[1], m[2]}), y0 = Vec3{m[4], m[5], m[6]};
    const Vec3 z = Normalize(Cross(x, y0)), y = Cross(z, x);
    const float r00 = x.x, r11 = y.y, r22 = z.z, trace = r00 + r11 + r22;
    Quat o;
    if (trace > 0) {
        const float s = std::sqrt(trace + 1.0f) * 2;
        o = {(y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s, 0.25f * s};
    } else if (r00 > r11 && r00 > r22) {
        const float s = std::sqrt(1.0f + r00 - r11 - r22) * 2;
        o = {0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s, (y.z - z.y) / s};
    } else if (r11 > r22) {
        const float s = std::sqrt(1.0f + r11 - r00 - r22) * 2;
        o = {(y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s, (z.x - x.z) / s};
    } else {
        const float s = std::sqrt(1.0f + r22 - r00 - r11) * 2;
        o = {(z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s, (x.y - y.x) / s};
    }
    *q = Normalize(o);
}

// The bones that make up the body proper: every bone on the way from Root to the head, the hands, the feet and the toes.
std::vector<bool> MainBones(const Skeleton& sk) {
    std::vector<bool> main(sk.Count(), false);
    const char* ends[] = {"head", "left_hand", "right_hand", "left_foot", "right_foot"};
    auto mark = [&](int i) { for (; i >= 0; i = sk.BoneAt(i).parent) main[i] = true; };
    for (const char* e : ends) mark(sk.Role(e));
    for (const char* toe : {"LeftToeBase", "RightToeBase"}) mark(sk.Find(toe));
    return main;
}
}  // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int) {
    WNDCLASSEXW wc{sizeof(wc), CS_CLASSDC, WndProc, 0, 0, instance, nullptr, nullptr, nullptr, nullptr, L"FbtIkLab", nullptr};
    RegisterClassExW(&wc);
    HWND hwnd = CreateWindowW(wc.lpszClassName, L"FBT IK lab", WS_OVERLAPPEDWINDOW, 100, 100, 1600, 1000, nullptr, nullptr,
                              wc.hInstance, nullptr);
    if (!CreateDevice(hwnd)) { CleanupDevice(); UnregisterClassW(wc.lpszClassName, wc.hInstance); return 1; }
    lab::BoxRenderer boxes;
    if (!boxes.Init(g_device)) { MessageBoxW(hwnd, L"box renderer failed", L"FBT IK lab", MB_OK); return 1; }
    {
        RECT rc{};
        GetClientRect(hwnd, &rc);
        boxes.Resize(g_device, std::max<LONG>(1, rc.right - rc.left), std::max<LONG>(1, rc.bottom - rc.top));
    }
    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device, g_context);

    // The rig file: the first command-line argument, else fbt_rig_male.txt in the working folder.
    char rigPath[1024] = "fbt_rig_male.txt";
    {
        int argc = 0;
        LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
        if (argv && argc > 1) WideCharToMultiByte(CP_UTF8, 0, argv[1], -1, rigPath, sizeof(rigPath), nullptr, nullptr);
        if (argv) LocalFree(argv);
    }
    Skeleton skeleton;
    std::string status;
    Pose pose;
    IkStack stack;
    Calibrator calibrator;
    Calibration calibration;
    bool calibrating = false, triggersArmed = false, live = false;
    bool armsStep = false;   // calibrated, waiting for the arms-out instant (the second pull measures the arms)
    bool handsFromWrist = false;   // the hands from the hand-tracking wrist joints instead of the grip poses (diagnostic)
    std::string calibStatus = "not calibrated";
    // The devices' filter (not the headset): the goals use the filtered poses, the recording keeps the raw ones.
    FilterOptions filterOptions;
    PoseFilter filters[kCalCount], headFilter;   // the head goal: only impossible steps held (the view is never filtered)
    double lastFrameTime = 0;
    // The flight recorder: starts at each capture, records while live, saved on recalibration, on Stop VR or on demand,
    // into <rig folder>\..\recordings\fbt_<date>_<time>.txt.
    lab::Recording recording;
    bool recordingOn = false;
    double recordingStart = 0;
    std::string recordingStatus = "no recording";
    auto saveRecording = [&] {
        if (!recordingOn || recording.frames.empty()) { recordingOn = false; return; }
        std::string dir = rigPath;
        const size_t slash = dir.find_last_of("\\/");
        dir = (slash == std::string::npos ? std::string(".") : dir.substr(0, slash)) + "\\..\\recordings";
        CreateDirectoryA(dir.c_str(), nullptr);
        SYSTEMTIME st;
        GetLocalTime(&st);
        char name[64];
        sprintf_s(name, "\\fbt_%04d%02d%02d_%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        const std::string path = dir + name;
        recordingStatus = lab::SaveRecording(path.c_str(), recording)
                              ? "saved " + std::to_string(recording.frames.size()) + " frames to " + path
                              : "could not write " + path;
        recordingOn = false;
        recording.frames.clear();
    };
    StackOptions options;
    StackDebug debug;
    std::vector<bool> mainBones;
    std::vector<lab::Box> robot;
    // V's body mesh: the game's player body, exported by WolvenKit into <rig folder>\..\mesh (game files: each user
    // exports their own, see README.md); without it the robot stands in.
    lab::VMesh vmesh;
    std::string meshStatus = "no mesh";
    bool showBody = true;
    auto loadMesh = [&] {
        vmesh.Clear();
        std::string dir = rigPath;
        const size_t slash = dir.find_last_of("\\/");
        dir = (slash == std::string::npos ? std::string(".") : dir.substr(0, slash)) + "\\..\\mesh\\base\\characters\\common\\player_base_bodies\\";
        const bool female = std::string(rigPath).find("female") != std::string::npos;
        const float skin[3] = {0.80f, 0.64f, 0.54f}, cloth[3] = {0.16f, 0.17f, 0.20f};
        struct File { const char* path; const float* color; };
        const File male[] = {{"player_man_average\\t0_000_pma_base__full.glb", skin},
                             {"player_man_average\\arms_hq\\a0_000_pma_base_hq__l.glb", skin},
                             {"player_man_average\\arms_hq\\a0_000_pma_base_hq__r.glb", skin},
                             {"player_man_average\\i0_000_pma_base_full_censored.glb", cloth}};
        const File woman[] = {{"player_female_average\\t0_000_pwa_base__full.glb", skin},
                              {"player_female_average\\arms_hq\\a0_000_pwa_base_hq__l.glb", skin},
                              {"player_female_average\\arms_hq\\a0_000_pwa_base_hq__r.glb", skin},
                              {"player_female_average\\i0_000_pwa_base_full_censored.glb", cloth}};
        std::string error;
        int loaded = 0;
        for (const File& f : female ? woman : male) {
            const std::string path = dir + f.path;
            if (vmesh.Load(path.c_str(), skeleton, f.color, &error)) ++loaded;
        }
        if (!loaded) { meshStatus = "mesh: " + error; return; }
        boxes.SetMesh(g_device, vmesh.Indices().data(), static_cast<UINT>(vmesh.Indices().size()), static_cast<UINT>(vmesh.Vertices().size()));
        meshStatus = "mesh: " + std::to_string(loaded) + " parts, " + std::to_string(vmesh.Vertices().size()) + " vertices, " +
                     std::to_string(vmesh.Indices().size() / 3) + " triangles" +
                     (vmesh.UnmatchedJoints() ? ", " + std::to_string(vmesh.UnmatchedJoints()) + " joints not in the rig" : std::string());
        if (!error.empty() && loaded < 4) meshStatus += " (" + error + ")";
    };
    LabGoal goals[] = {{"head", "head"},          {"left hand", "left_hand"}, {"right hand", "right_hand"},
                    {"hips", "hips"},          {"left foot", "left_foot"}, {"right foot", "right_foot"}};   // GoalId order
    constexpr int kGoals = sizeof(goals) / sizeof(goals[0]);
    static_assert(kGoals == kGoalCount, "one lab goal per GoalId, in its order");
    int selected = -1;
    ImGuizmo::OPERATION gizmoOp = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE gizmoMode = ImGuizmo::LOCAL;
    // A goal back on its bone's reference pose (V's T-pose), not the bone's current one: that follows the goals.
    auto resetGoal = [&](LabGoal& g) {
        const int b = skeleton.Role(g.role);
        g.pos = b >= 0 ? skeleton.BoneAt(b).refModelPos : Vec3{};
        g.rot = b >= 0 ? skeleton.BoneAt(b).refModelRot : Quat{};
    };
    auto resetGoals = [&] { for (auto& g : goals) resetGoal(g); };
    auto load = [&] {
        std::string error;
        if (skeleton.Load(rigPath, &error)) {
            pose = skeleton.ReferencePose();
            mainBones = MainBones(skeleton);
            robot = lab::BuildRobot(skeleton);
            status = "loaded " + std::to_string(skeleton.Count()) + " bones";
            if (!stack.Init(skeleton)) status += " (IK: bones missing)";
            if (!calibrator.Init(skeleton)) status += " (calibration: bones missing)";
            loadMesh();
            // The goals onto the new skeleton's reference pose: left where they were (another rig's bones, or dragged),
            // the IK pulled the body into them and it collapsed.
            resetGoals();
            selected = -1;
        } else {
            status = error;
        }
    };
    load();

    lab::VrSession vr;
    std::vector<lab::EyeTarget> eyes;
    Camera camera;
    bool showMesh = vmesh.Empty(), showSkeleton = false, showHelpers = false, showNames = false, showDevices = true, showMirror = true;
    float mirrorDistance = 1.5f;
    // Which triad axis comes out of the tracker's large faces: 0 red, 1 green, 2 blue. Green, checked on the Vive
    // trackers (the pose's z axis is the puck's up).
    int puckUp = 1;
    bool done = false;
    while (!done) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) done = true;
        }
        if (done) break;
        if (g_resizeW != 0 && g_resizeH != 0) {
            CleanupRenderTarget();
            g_swapChain->ResizeBuffers(0, g_resizeW, g_resizeH, DXGI_FORMAT_UNKNOWN, 0);
            boxes.Resize(g_device, g_resizeW, g_resizeH);
            g_resizeW = g_resizeH = 0;
            CreateRenderTarget();
        }
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();
        ImGuiIO& io = ImGui::GetIO();

        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                camera.yaw -= io.MouseDelta.x * 0.005f;
                camera.pitch = std::clamp(camera.pitch + io.MouseDelta.y * 0.005f, -1.5f, 1.5f);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
                const Vec3 f = Normalize(camera.target - camera.Eye()), s = Normalize(Cross(f, {0, 0, 1})), u = Cross(s, f);
                const float k = camera.distance * 0.0015f;
                camera.target = camera.target - s * (io.MouseDelta.x * k) + u * (io.MouseDelta.y * k);
            }
            if (io.MouseWheel != 0) camera.distance = std::clamp(camera.distance * std::pow(0.9f, io.MouseWheel), 0.3f, 30.0f);
        }

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("FBT IK lab");
        ImGui::InputText("rig", rigPath, sizeof(rigPath));
        ImGui::SameLine();
        if (ImGui::Button("Load")) load();
        ImGui::TextUnformatted(status.c_str());
        ImGui::Checkbox("V body", &showBody);
        ImGui::SameLine();
        ImGui::Checkbox("robot", &showMesh);
        ImGui::SameLine();
        ImGui::Checkbox("skeleton", &showSkeleton);
        ImGui::SameLine();
        ImGui::Checkbox("helper bones", &showHelpers);
        ImGui::SameLine();
        ImGui::Checkbox("names", &showNames);
        ImGui::TextUnformatted(meshStatus.c_str());
        if (!vr.Running()) {
            if (ImGui::Button("Start VR") && vr.Start(g_device, nullptr)) {
                calibrating = true;   // VR starts in calibration: V in its reference pose, waiting for both triggers
                armsStep = false;
                triggersArmed = false;
                live = false;
                calibStatus = "stand straight, arms relaxed along the body, then both triggers";
            }
        } else if (ImGui::Button("Stop VR")) {
            saveRecording();
            vr.Stop();
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(vr.Status().c_str());
        if (vr.Running()) {
            // Calibration (on when VR starts): V stands at the room's centre; stand straight, arms along the body, pull
            // both triggers; then arms out to the sides, both triggers (the arm length). Then the devices drive the
            // goals ("live"); both triggers again start a new calibration.
            if (!calibrating) {
                if (ImGui::Button("Calibrate")) {
                    calibrating = true;
                    live = false;
                    vr.SetRoomTransform({}, {}, 1.0f);
                    calibStatus = "stand straight, arms relaxed along the body, then both triggers";
                }
            } else if (ImGui::Button("Cancel calibration")) {
                calibrating = false;
                calibStatus = calibration.done ? "calibrated" : "not calibrated";
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(!calibration.done);
            ImGui::Checkbox("live", &live);
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::TextDisabled("%s", calibStatus.c_str());
            if (calibration.done)
                ImGui::TextDisabled("scale %.4f (V eye %.3f m / yours %.3f m)", calibration.scale, calibration.avatarEyeHeight, calibration.playerEyeHeight);
            if (recordingOn) {
                ImGui::Text("recording: %d frames (%.0f s)", static_cast<int>(recording.frames.size()), ImGui::GetTime() - recordingStart);
                ImGui::SameLine();
                if (ImGui::Button("Save recording")) saveRecording();
            } else {
                ImGui::TextDisabled("%s", recordingStatus.c_str());
            }
            ImGui::Checkbox("filter", &filterOptions.enabled);
            ImGui::SameLine();
            ImGui::TextDisabled("rejected L/R foot %d/%d, hands %d/%d, waist %d", filters[kCalLeftFoot].Rejected(),
                                filters[kCalRightFoot].Rejected(), filters[kCalLeftHand].Rejected(), filters[kCalRightHand].Rejected(),
                                filters[kCalWaist].Rejected());
            ImGui::Checkbox("devices", &showDevices);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90);
            ImGui::Combo("puck up", &puckUp, "red\0green\0blue\0");
            ImGui::SameLine();
            ImGui::Checkbox("mirror", &showMirror);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("at", &mirrorDistance, 0.8f, 4.0f, "%.1f m");
            ImGui::SameLine();
            auto mark = [&](lab::Device d) { return vr.Pose(d).valid ? "+" : "-"; };
            ImGui::TextDisabled("head %s  hands %s%s  waist %s  feet %s%s%s  triggers %.2f %.2f", mark(lab::kDevHead),
                                mark(lab::kDevLeftHand), mark(lab::kDevRightHand), mark(lab::kDevWaist), mark(lab::kDevLeftFoot),
                                mark(lab::kDevRightFoot), vr.TrackersAvailable() ? "" : " (no tracker extension)", vr.Trigger(0),
                                vr.Trigger(1));
            ImGui::Checkbox("hands from the wrist joints", &handsFromWrist);
            ImGui::SameLine();
            ImGui::TextDisabled(vr.HandTrackingAvailable() ? "hand tracking: wrists %s%s (else the grip poses)" : "no hand tracking: grip poses%s%s",
                                vr.HandTrackingAvailable() ? mark(lab::kDevLeftWrist) : "", vr.HandTrackingAvailable() ? mark(lab::kDevRightWrist) : "");
        }
        if (ImGui::Button("Reset camera")) camera = Camera{};
        ImGui::SameLine();
        if (ImGui::Button("Reset goals")) resetGoals();
        ImGui::SeparatorText("stack");
        ImGui::Checkbox("1 set hips", &options.setHips);
        ImGui::SameLine();
        ImGui::Checkbox("4f hips give way", &options.hipsGiveWay);
        ImGui::SameLine();
        ImGui::TextDisabled("shift %.1f cm", Length(debug.hipsShift) * 100);
        ImGui::TextDisabled("2-3 knee plane: yellow arrows");
        ImGui::Checkbox("4 legs", &options.legs);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("soft reach", &options.legSoftness, 0.0f, 0.10f, "%.3f");
        ImGui::TextDisabled("reach L %.2f  R %.2f  (> 1: out of reach)", debug.legReach[0], debug.legReach[1]);
        ImGui::Checkbox("5 spine -> head", &options.spine);
        ImGui::SameLine();
        ImGui::TextDisabled("reach %.2f  %d passes  error %.1f mm", debug.spineReach, debug.spineIterations, debug.spineError * 1000);
        ImGui::Checkbox("5b shoulders protract", &options.clavicle);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::SliderFloat("arm length", &options.armLengthTune, 0.85f, 1.15f, "x %.2f");
        ImGui::TextDisabled("arms x %.2f %.2f  reach %.2f %.2f  clavicle %.0f %.0f deg", options.armScale[0] * options.armLengthTune,
                            options.armScale[1] * options.armLengthTune, debug.armExtension[0], debug.armExtension[1], debug.clavicleDeg[0],
                            debug.clavicleDeg[1]);
        ImGui::Checkbox("6-7 arms", &options.arms);
        ImGui::SameLine();
        ImGui::Checkbox("elbow zones", &options.elbowZones);
        ImGui::SameLine();
        ImGui::TextDisabled("reach L %.2f  R %.2f   elbows: orange arrows", debug.armReach[0], debug.armReach[1]);
        if (ImGui::CollapsingHeader("experiments (arms v1, off by default)")) {
            ImGui::Checkbox("elbow from the hand's position (v2)", &options.elbowFromPosition);
            ImGui::Checkbox("elbow from thumb (v1)", &options.elbowFromThumb);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("elbow down", &options.elbowDown, 0.0f, 2.0f, "%.2f");
            ImGui::Checkbox("8-9 twist solver (v1)", &options.twist);
            ImGui::SameLine();
            ImGui::TextDisabled("wrist twist L %+.0f R %+.0f  elbow roll L %+.0f R %+.0f deg", debug.twistDeg[0], debug.twistDeg[1],
                                debug.elbowRollDeg[0], debug.elbowRollDeg[1]);
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("10 wrist limit", &options.wristLimitDeg, 0.0f, 120.0f, "%.0f deg");
            ImGui::SameLine();
            ImGui::TextDisabled("bend L %.0f R %.0f%s", debug.wristBendDeg[0], debug.wristBendDeg[1],
                                debug.wristClamped[0] || debug.wristClamped[1] ? "  (clamped)" : "");
        }
        ImGui::SeparatorText("goals");
        for (int i = 0; i < kGoals; ++i) {
            ImGui::PushID(i);
            if (ImGui::SmallButton("reset")) resetGoal(goals[i]);
            ImGui::PopID();
            ImGui::SameLine();
            if (ImGui::Selectable(goals[i].label, selected == i, 0, ImVec2(110, 0))) selected = selected == i ? -1 : i;
            ImGui::SameLine();
            ImGui::TextDisabled("%+.3f %+.3f %+.3f", goals[i].pos.x, goals[i].pos.y, goals[i].pos.z);
        }
        if (ImGui::RadioButton("move (W)", gizmoOp == ImGuizmo::TRANSLATE)) gizmoOp = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("rotate (E)", gizmoOp == ImGuizmo::ROTATE)) gizmoOp = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("local", gizmoMode == ImGuizmo::LOCAL)) gizmoMode = ImGuizmo::LOCAL;
        ImGui::SameLine();
        if (ImGui::RadioButton("world", gizmoMode == ImGuizmo::WORLD)) gizmoMode = ImGuizmo::WORLD;
        ImGui::TextDisabled("right drag: orbit   middle drag: pan   wheel: zoom");
        ImGui::End();

        // The devices in room space (calibration order), the calibration's trigger, and the goals: from the devices when
        // live, else from the gizmos. While calibrating V stays in its reference pose.
        RoomPose roomHead, room[kCalCount], roomWrist[2], input[kCalCount];
        if (vr.Running()) {
            auto take = [&](lab::Device d, RoomPose& out) { const auto& dp = vr.Pose(d); out.valid = dp.valid; out.pos = dp.pos; out.rot = dp.rot; };
            take(lab::kDevHead, roomHead);
            take(lab::kDevWaist, room[kCalWaist]);
            take(lab::kDevLeftFoot, room[kCalLeftFoot]);
            take(lab::kDevRightFoot, room[kCalRightFoot]);
            take(lab::kDevLeftHand, room[kCalLeftHand]);
            take(lab::kDevRightHand, room[kCalRightHand]);
            take(lab::kDevLeftWrist, roomWrist[0]);
            take(lab::kDevRightWrist, roomWrist[1]);
            roomWrist[0].wrist = roomWrist[0].valid;
            roomWrist[1].wrist = roomWrist[1].valid;
            // The hands from the hand-tracking wrist joints when on and tracked, else the grip poses.
            for (int i = 0; i < kCalCount; ++i) input[i] = room[i];
            if (handsFromWrist) {
                if (roomWrist[0].valid) input[kCalLeftHand] = roomWrist[0];
                if (roomWrist[1].valid) input[kCalRightHand] = roomWrist[1];
            }
            // Both triggers toggle calibration: while calibrating a pull captures (and goes live); otherwise a pull starts
            // calibrating again. A pull counts once: both must be released before the next one.
            {
                const float l = vr.Trigger(0), r = vr.Trigger(1);
                if (!triggersArmed) triggersArmed = l < 0.2f && r < 0.2f;
                else if (l > 0.8f && r > 0.8f) {
                    triggersArmed = false;
                    if (calibrating) {
                        const char* why = nullptr;
                        if (calibrator.Capture(roomHead, input, &calibration, &why)) {
                            calibStatus = "calibrated; now arms straight out to the sides, then both triggers";
                            live = true;
                            calibrating = false;
                            armsStep = true;
                            vr.SetRoomTransform(calibration.origin, calibration.yawInverse, calibration.scale);
                            recording = {calibration, {}};
                            recordingOn = true;
                            for (auto& f : filters) f.Reset();
                            headFilter.Reset();
                            lastFrameTime = 0;
                            recordingStart = ImGui::GetTime();
                        } else {
                            calibStatus = std::string("not calibrated: ") + why;   // stays in calibration
                        }
                    } else if (armsStep) {
                        armsStep = false;
                        if (calibrator.MeasureArmsOut(calibration, roomHead, input)) {
                            recording.calibration = calibration;   // the arm lengths into the recording's header
                            recording.armsOut = true;
                            calibStatus = "calibrated; arms out " + std::to_string(calibration.armLength[0]).substr(0, 5) + " / " +
                                          std::to_string(calibration.armLength[1]).substr(0, 5) + " m = V's x " +
                                          std::to_string(calibrator.ArmScale(calibration, 0)).substr(0, 4) + " / " +
                                          std::to_string(calibrator.ArmScale(calibration, 1)).substr(0, 4);
                        } else {
                            calibStatus = "arms not out to the sides: V's arms kept (both triggers: recalibrate)";
                        }
                    } else {
                        saveRecording();
                        calibrating = true;
                        live = false;
                        vr.SetRoomTransform({}, {}, 1.0f);   // back to the room: V at its centre, your eyes your own
                        calibStatus = "stand straight, arms relaxed along the body, then both triggers";
                    }
                }
            }
        }
        {
            wrpuppets::fbt::Goal ik[kGoalCount];
            if (live && calibration.done && vr.Running()) {
                const double now = ImGui::GetTime();
                const float dt = lastFrameTime > 0 ? static_cast<float>(now - lastFrameTime) : 0.0f;
                lastFrameTime = now;
                RoomPose filtered[kCalCount];
                for (int i = 0; i < kCalCount; ++i) filtered[i] = filters[i].Filter(input[i], dt, filterOptions);
                const RoomPose headGoal = headFilter.Filter(roomHead, dt, HeadGoalFilter());
                calibrator.Goals(calibration, headGoal, filtered, ik);
                options.armScale[0] = armsStep ? 1.0f : calibrator.ArmScale(calibration, 0);
                options.armScale[1] = armsStep ? 1.0f : calibrator.ArmScale(calibration, 1);
                if (recordingOn && recording.frames.size() < 10 * 60 * 90) {   // at most ~10 minutes at 90 Hz
                    lab::RecFrame fr;
                    fr.t = ImGui::GetTime() - recordingStart;
                    fr.head = roomHead;
                    for (int i = 0; i < kCalCount; ++i) fr.dev[i] = room[i];
                    fr.wrist[0] = roomWrist[0];
                    fr.wrist[1] = roomWrist[1];
                    recording.frames.push_back(fr);
                }
            } else {
                for (int i = 0; i < kGoals; ++i) { ik[i].active = true; ik[i].pos = goals[i].pos; ik[i].rot = goals[i].rot; }
            }
            pose = skeleton.ReferencePose();
            if (!calibrating) stack.Solve(pose, ik, options, &debug);
        }

        Projector pr;
        pr.size = io.DisplaySize;
        LookAt(camera.Eye(), camera.target, pr.view);
        Perspective(camera.fovY, io.DisplaySize.x / std::max(1.0f, io.DisplaySize.y), 0.05f, 100.0f, pr.proj);
        ImDrawList* dl = ImGui::GetBackgroundDrawList();

        // Goals: an axis triad and a label each, on top; the selected one gets the gizmo.
        if (!io.WantCaptureKeyboard) {
            if (ImGui::IsKeyPressed(ImGuiKey_W)) gizmoOp = ImGuizmo::TRANSLATE;
            if (ImGui::IsKeyPressed(ImGuiKey_E)) gizmoOp = ImGuizmo::ROTATE;
            if (ImGui::IsKeyPressed(ImGuiKey_R) && selected >= 0) resetGoal(goals[selected]);
        }
        if (selected >= 0) {
            float m[16];
            ToMatrix(goals[selected].pos, goals[selected].rot, m);
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
            if (ImGuizmo::Manipulate(pr.view, pr.proj, gizmoOp, gizmoMode, m)) FromMatrix(m, &goals[selected].pos, &goals[selected].rot);
        }
        if (!io.WantCaptureMouse && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGuizmo::IsOver()) {
            int best = -1;
            float bestD = 30.0f;
            for (int i = 0; i < kGoals; ++i) {
                ImVec2 sp;
                if (!pr(goals[i].pos, &sp)) continue;
                const float d = std::hypot(sp.x - io.MousePos.x, sp.y - io.MousePos.y);
                if (d < bestD) { bestD = d; best = i; }
            }
            selected = best;
        }
        for (int side = 0; side < 2; ++side) {
            if (!debug.legPlaneValid[side]) continue;
            const Vec3 o = debug.legPlaneOrigin[side], tip = o + debug.legKneeDir[side] * 0.25f;
            Line3(dl, pr, o, tip, IM_COL32(255, 220, 0, 255), 3.0f);
            ImVec2 sp;
            if (pr(tip, &sp)) dl->AddCircleFilled(sp, 4.0f, IM_COL32(255, 220, 0, 255));
        }
        for (int side = 0; side < 2; ++side) {
            if (!debug.elbowValid[side]) continue;
            const Vec3 o = debug.elbowOrigin[side], tip = o + debug.elbowDir[side] * 0.2f;
            Line3(dl, pr, o, tip, IM_COL32(255, 140, 0, 255), 3.0f);
            ImVec2 sp;
            if (pr(tip, &sp)) dl->AddCircleFilled(sp, 4.0f, IM_COL32(255, 140, 0, 255));
        }
        for (int i = 0; i < kGoals; ++i) {
            const LabGoal& g = goals[i];
            const float len = 0.10f;
            Line3(dl, pr, g.pos, g.pos + Rotate(g.rot, {len, 0, 0}), IM_COL32(240, 70, 70, 255), 3.0f);
            Line3(dl, pr, g.pos, g.pos + Rotate(g.rot, {0, len, 0}), IM_COL32(70, 230, 90, 255), 3.0f);
            Line3(dl, pr, g.pos, g.pos + Rotate(g.rot, {0, 0, len}), IM_COL32(80, 130, 255, 255), 3.0f);
            ImVec2 sp;
            if (pr(g.pos, &sp)) {
                dl->AddCircleFilled(sp, i == selected ? 7.0f : 5.0f, i == selected ? IM_COL32(255, 230, 60, 255) : IM_COL32(255, 255, 255, 230));
                dl->AddText(ImVec2(sp.x + 8, sp.y + 4), IM_COL32(255, 255, 255, 230), g.label);
            }
        }

        // The skeleton (on top of the robot): body bones thick, helpers thin (optional), joints as dots.
        for (int i = 0; i < skeleton.Count() && (showSkeleton || showHelpers); ++i) {
            const int parent = skeleton.BoneAt(i).parent;
            if (parent < 0) continue;
            const bool main = mainBones[i];
            if (main ? !showSkeleton : !showHelpers) continue;
            Line3(dl, pr, pose.modelPos[parent], pose.modelPos[i], main ? IM_COL32(230, 230, 230, 255) : IM_COL32(120, 120, 160, 160),
                  main ? 3.0f : 1.0f);
        }
        for (int i = 0; i < skeleton.Count() && showSkeleton; ++i) {
            if (!mainBones[i]) continue;
            ImVec2 s;
            if (!pr(pose.modelPos[i], &s)) continue;
            dl->AddCircleFilled(s, 4.0f, IM_COL32(255, 200, 60, 255));
            if (showNames) dl->AddText(ImVec2(s.x + 6, s.y - 6), IM_COL32(200, 200, 200, 255), skeleton.BoneAt(i).name.c_str());
        }

        ImGui::Render();
        const float clear[4] = {0.11f, 0.12f, 0.14f, 1.0f};
        // The 3D scene for one view: the floor grid (thin flat boxes, 10 cm cells within 2 m, metres brighter), the origin
        // axes (x red, y forward green, z up blue) and the robot. The window and each headset eye draw it.
        if (showBody && !vmesh.Empty()) {   // skinned once per frame, drawn by every view
            vmesh.Skin(pose);
            boxes.UpdateMesh(g_context, vmesh.Vertices().data(), static_cast<UINT>(vmesh.Vertices().size()));
        }
        auto drawScene = [&](ID3D11RenderTargetView* target, ID3D11DepthStencilView* depth, float width, float height,
                             const float viewProj[16], Vec3 eye) {
            g_context->ClearRenderTargetView(target, clear);
            const D3D11_VIEWPORT vp{0, 0, width, height, 0, 1};
            g_context->RSSetViewports(1, &vp);
            boxes.Begin(g_context, target, viewProj, eye, depth);
            for (int i = -20; i <= 20; ++i) {
                const float c = i * 0.1f, w = i % 10 == 0 ? 0.006f : 0.003f;
                const float g = i == 0 ? 0.42f : (i % 10 == 0 ? 0.32f : 0.22f);
                const float col[3] = {g, g, g};
                const Vec3 alongY[3] = {{w, 0, 0}, {0, 4, 0}, {0, 0, 0.001f}}, alongX[3] = {{4, 0, 0}, {0, w, 0}, {0, 0, 0.001f}};
                boxes.Draw(g_context, {c, 0, -0.001f}, alongY, col);
                boxes.Draw(g_context, {0, c, -0.001f}, alongX, col);
            }
            const float red[3] = {0.85f, 0.25f, 0.25f}, green[3] = {0.25f, 0.8f, 0.3f}, blue[3] = {0.3f, 0.45f, 0.95f};
            const Vec3 ax[3] = {{0.3f, 0, 0}, {0, 0.012f, 0}, {0, 0, 0.012f}}, ay[3] = {{0.012f, 0, 0}, {0, 0.3f, 0}, {0, 0, 0.012f}},
                       az[3] = {{0.012f, 0, 0}, {0, 0.012f, 0}, {0, 0, 0.3f}};
            boxes.Draw(g_context, {0.15f, 0, 0}, ax, red);
            boxes.Draw(g_context, {0, 0.15f, 0}, ay, green);
            boxes.Draw(g_context, {0, 0, 0.15f}, az, blue);
            // The body (devices + robot), drawn once as it is and, with the mirror, once reflected through the mirror's
            // plane (y = mirrorDistance in V's frame): positions and axes mirrored in y. The boxes are two-sided, so the
            // reflected (left-handed) ones render as they should.
            auto drawBody = [&](bool reflected) {
            auto reflect = [&](Vec3 v, bool point) {
                if (!reflected) return v;
                return Vec3{v.x, point ? 2 * mirrorDistance - v.y : -v.y, v.z};
            };
            auto box = [&](Vec3 c, const Vec3 ax[3], const float* col) {
                const Vec3 r[3] = {reflect(ax[0], false), reflect(ax[1], false), reflect(ax[2], false)};
                boxes.Draw(g_context, reflect(c, true), r, col);
            };
            // The devices: controllers yellow, trackers green, headset white; each with an axis triad (x red, y green,
            // z blue) in its own frame.
            if (vr.Running() && showDevices) {
                const float yellow[3] = {0.95f, 0.85f, 0.25f}, green[3] = {0.3f, 0.85f, 0.35f}, white[3] = {0.9f, 0.9f, 0.9f};
                const float axisCol[3][3] = {{0.9f, 0.25f, 0.25f}, {0.25f, 0.85f, 0.3f}, {0.3f, 0.45f, 0.95f}};
                auto device = [&](lab::Device d, Vec3 size, const float* col) {
                    lab::DevicePose dp = vr.Pose(d);
                    if (!dp.valid) return;
                    if (calibration.done) {   // into V's frame, next to the bones they drive
                        dp.pos = calibrator.ToModel(calibration, dp.pos);
                        dp.rot = Normalize(calibration.yawInverse * dp.rot);
                    }
                    const Vec3 ax[3] = {Rotate(dp.rot, {size.x, 0, 0}), Rotate(dp.rot, {0, size.y, 0}), Rotate(dp.rot, {0, 0, size.z})};
                    box(dp.pos, ax, col);
                    for (int k = 0; k < 3; ++k) {
                        Vec3 dir{};
                        (k == 0 ? dir.x : k == 1 ? dir.y : dir.z) = 1;
                        const Vec3 tip = Rotate(dp.rot, dir) * 0.10f;
                        const Vec3 thin[3] = {k == 0 ? tip : Rotate(dp.rot, {0.006f, 0, 0}), k == 1 ? tip : Rotate(dp.rot, {0, 0.006f, 0}),
                                              k == 2 ? tip : Rotate(dp.rot, {0, 0, 0.006f})};
                        box(dp.pos + tip * 0.5f, thin, axisCol[k]);
                    }
                };
                // Grip poses: in model space the controller's length runs along its own -y (OpenXR -z).
                device(lab::kDevLeftHand, {0.04f, 0.12f, 0.04f}, yellow);
                device(lab::kDevRightHand, {0.04f, 0.12f, 0.04f}, yellow);
                // Trackers: 8 cm across, 4 cm thick along the axis chosen as their up (the large faces' normal).
                const Vec3 puck = puckUp == 0 ? Vec3{0.04f, 0.08f, 0.08f} : puckUp == 1 ? Vec3{0.08f, 0.04f, 0.08f} : Vec3{0.08f, 0.08f, 0.04f};
                for (lab::Device d : {lab::kDevWaist, lab::kDevLeftFoot, lab::kDevRightFoot}) device(d, puck, green);
                if (eye.x != 0 || eye.y != 0 || eye.z != 0) {   // the headset only in the window (in VR you are inside it)
                    const lab::DevicePose& h = vr.Pose(lab::kDevHead);
                    const Vec3 hp = calibration.done ? calibrator.ToModel(calibration, h.pos) : h.pos;
                    if (h.valid && (reflected || Length(hp - eye) > 0.3f)) device(lab::kDevHead, {0.18f, 0.10f, 0.10f}, white);
                }
            }
            if (showBody && !vmesh.Empty()) {
                const Vec3 identity[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
                const Vec3 r[3] = {reflect(identity[0], false), reflect(identity[1], false), reflect(identity[2], false)};
                for (const auto& part : vmesh.Parts()) {
                    const float tinted[3] = {part.color[0] * 0.55f, part.color[1] * 0.75f + 0.25f, part.color[2] * 0.55f};
                    boxes.DrawMesh(g_context, part.firstIndex, part.indexCount, reflect({0, 0, 0}, true), r, calibrating ? tinted : part.color);
                }
            }
            if (showMesh)
                for (const auto& b : robot) {
                    Vec3 center, axes[3];
                    lab::Place(b, pose, &center, axes);
                    // Greenish while calibrating, so the mode shows in the headset.
                    const float tinted[3] = {b.color[0] * 0.55f, b.color[1] * 0.75f + 0.25f, b.color[2] * 0.55f};
                    box(center, axes, calibrating ? tinted : b.color);
                }
            };
            drawBody(false);
            if (showMirror && vr.Running()) {
                drawBody(true);
                // The mirror's frame: 1 m wide, 0.1 .. 2.1 m high, at y = mirrorDistance.
                const float frameCol[3] = {0.35f, 0.38f, 0.45f};
                const float y = mirrorDistance;
                const Vec3 vertical[3] = {{0.03f, 0, 0}, {0, 0.03f, 0}, {0, 0, 2.0f}}, horizontal[3] = {{1.06f, 0, 0}, {0, 0.03f, 0}, {0, 0, 0.03f}};
                boxes.Draw(g_context, {-0.515f, y, 1.1f}, vertical, frameCol);
                boxes.Draw(g_context, {0.515f, y, 1.1f}, vertical, frameCol);
                boxes.Draw(g_context, {0, y, 0.1f}, horizontal, frameCol);
                boxes.Draw(g_context, {0, y, 2.1f}, horizontal, frameCol);
            }
        };
        // The headset first (it paces the loop), then the window.
        if (vr.BeginFrame(&eyes)) {
            for (const auto& e : eyes) drawScene(e.rtv, e.dsv, float(e.width), float(e.height), e.viewProj, e.eye);
        }
        vr.EndFrame();
        {
            float viewProj[16];
            Multiply(pr.proj, pr.view, viewProj);
            drawScene(g_rtv, nullptr, io.DisplaySize.x, io.DisplaySize.y, viewProj, camera.Eye());
        }
        g_context->OMSetRenderTargets(1, &g_rtv, nullptr);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_swapChain->Present(vr.Running() ? 0 : 1, 0);   // the headset paces the loop while it runs
    }

    saveRecording();
    vr.Stop();
    boxes.Release();
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDevice();
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}
