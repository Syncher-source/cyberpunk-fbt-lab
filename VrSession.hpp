#pragma once

// The lab in the headset: an OpenXR session on the lab's D3D11 device, one swapchain (and depth buffer) per eye, views
// located in STAGE space (SteamVR's floor centre). Everything the lab draws is in V's model space (x right, y forward,
// z up, metres); STAGE (x right, y up, -z forward) maps onto it as model = (x, -z, y), the same mapping the port uses.
#include "Fbt/FbtMath.hpp"
#include <d3d11.h>
#include <string>
#include <vector>

namespace lab {

// One eye to draw this frame.
struct EyeTarget {
    ID3D11RenderTargetView* rtv = nullptr;
    ID3D11DepthStencilView* dsv = nullptr;
    UINT width = 0, height = 0;
    float viewProj[16]{};   // column-major, D3D clip depth 0..1
    wrpuppets::fbt::Vec3 eye;     // the eye's position, model space
};

// A tracked device this frame, model space.
struct DevicePose {
    bool valid = false;
    wrpuppets::fbt::Vec3 pos;
    wrpuppets::fbt::Quat rot;
};
// The wrists: the hand-tracking skeleton's wrist joint (XR_EXT_hand_tracking; SteamVR derives it from the Index
// controllers' finger sensing or from gloves): anatomical, the same for every device -- VRChat places the hands from it.
enum Device { kDevHead, kDevLeftHand, kDevRightHand, kDevWaist, kDevLeftFoot, kDevRightFoot, kDevLeftWrist, kDevRightWrist, kDevCount };

class VrSession {
public:
    ~VrSession() { Stop(); }
    bool Start(ID3D11Device* device, std::string* status);
    void Stop();
    bool Running() const { return m_session != nullptr; }
    const std::string& Status() const { return m_status; }
    // Once per lab frame: handles the runtime's events; when the session wants a frame, fills the eyes and returns
    // true -- draw them, then call EndFrame().
    bool BeginFrame(std::vector<EyeTarget>* eyes);
    void EndFrame();
    // The headset's pose this frame, model space (valid after BeginFrame returned true).
    bool HeadPose(wrpuppets::fbt::Vec3* pos, wrpuppets::fbt::Quat* rot) const;
    // Headset, controllers (grip pose) and Vive trackers (waist, feet; XR_HTCX_vive_tracker_interaction), located in
    // STAGE at this frame's display time, model space. Valid after BeginFrame.
    const DevicePose& Pose(Device d) const { return m_devices[d]; }
    float Trigger(int hand) const { return m_trigger[hand]; }   // 0 left, 1 right
    bool TrackersAvailable() const { return m_trackerExt; }
    bool HandTrackingAvailable() const { return m_handTrackers[0] != nullptr; }
    // Room -> V's frame for the eyes (after calibration): position (p - origin) turned by yawInverse and scaled; rotation
    // yawInverse * q. Identity until set. Device poses stay in room space.
    void SetRoomTransform(wrpuppets::fbt::Vec3 origin, wrpuppets::fbt::Quat yawInverse, float scale) {
        m_roomOrigin = origin;
        m_roomYawInverse = yawInverse;
        m_roomScale = scale;
    }

private:
    struct Eye {
        void* swapchain = nullptr;   // XrSwapchain
        std::vector<ID3D11RenderTargetView*> rtvs;
        ID3D11Texture2D* depth = nullptr;
        ID3D11DepthStencilView* dsv = nullptr;
        UINT width = 0, height = 0;
    };
    void PollEvents();
    void Release();
    void CreateActions();
    void UpdateDevices();

    ID3D11Device* m_device = nullptr;
    void* m_instance = nullptr;   // XrInstance
    void* m_session = nullptr;    // XrSession
    void* m_stage = nullptr;      // XrSpace
    void* m_viewSpace = nullptr;  // XrSpace
    unsigned long long m_system = 0;
    int m_state = 0;              // XrSessionState
    bool m_begun = false, m_frameOpen = false, m_shouldRender = false;
    long long m_displayTime = 0;
    Eye m_eyes[2];
    unsigned m_imageIndex[2]{};
    struct ViewPose { float pos[3]; float rot[4]; float fov[4]; } m_views[2]{};
    std::string m_status = "VR off";
    bool m_trackerExt = false, m_handExt = false;
    void* m_handTrackers[2]{};   // XrHandTrackerEXT
    void* m_locateJoints = nullptr, *m_destroyTracker = nullptr;   // PFN_xrLocateHandJointsEXT, PFN_xrDestroyHandTrackerEXT
    void* m_actionSet = nullptr;                 // XrActionSet
    void* m_handPose = nullptr, *m_trigger_ = nullptr, *m_trackerPose = nullptr;   // XrAction
    unsigned long long m_handPaths[2]{}, m_rolePaths[3]{};
    void* m_handSpaces[2]{}, *m_trackerSpaces[3]{};
    DevicePose m_devices[kDevCount];
    float m_trigger[2]{};
    wrpuppets::fbt::Vec3 m_roomOrigin;
    wrpuppets::fbt::Quat m_roomYawInverse;
    float m_roomScale = 1.0f;
};

}  // namespace lab
