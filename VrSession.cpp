#include "VrSession.hpp"
#include <windows.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <cmath>
#include <cstring>

using namespace wrpuppets::fbt;

namespace lab {
namespace {
XrInstance Inst(void* p) { return static_cast<XrInstance>(p); }
XrSession Sess(void* p) { return static_cast<XrSession>(p); }
XrSpace Space(void* p) { return static_cast<XrSpace>(p); }
XrSwapchain Chain(void* p) { return static_cast<XrSwapchain>(p); }

// STAGE (x right, y up, -z forward) -> model space (x right, y forward, z up).
Vec3 ToModel(const XrVector3f& v) { return {v.x, -v.z, v.y}; }
Quat ToModel(const XrQuaternionf& q) { return Normalize(Quat{q.x, -q.z, q.y, q.w}); }

std::string Fail(const char* what, XrResult r) { return std::string(what) + " failed (" + std::to_string(static_cast<int>(r)) + ")"; }
}  // namespace

bool VrSession::Start(ID3D11Device* device, std::string* status) {
    Stop();
    m_device = device;
    // HTC's Vive software installs an implicit OpenXR layer (XR_APILAYER_VIVE_hand_tracking) that implements
    // XR_EXT_hand_tracking itself and drops the hands for seconds at a time, Index controllers too; this is its own off
    // switch, read by the loader at instance creation (as the port does).
    SetEnvironmentVariableW(L"DISABLE_XR_APILAYER_VIVE_HAND_TRACKING_1", L"1");
    // D3D11, plus the Vive trackers and hand tracking when the runtime has them.
    std::vector<const char*> extensions = {XR_KHR_D3D11_ENABLE_EXTENSION_NAME};
    {
        uint32_t n = 0;
        xrEnumerateInstanceExtensionProperties(nullptr, 0, &n, nullptr);
        std::vector<XrExtensionProperties> props(n, {XR_TYPE_EXTENSION_PROPERTIES});
        xrEnumerateInstanceExtensionProperties(nullptr, n, &n, props.data());
        m_trackerExt = m_handExt = false;
        for (const auto& p : props) {
            if (std::strcmp(p.extensionName, XR_HTCX_VIVE_TRACKER_INTERACTION_EXTENSION_NAME) == 0) m_trackerExt = true;
            if (std::strcmp(p.extensionName, XR_EXT_HAND_TRACKING_EXTENSION_NAME) == 0) m_handExt = true;
        }
        if (m_trackerExt) extensions.push_back(XR_HTCX_VIVE_TRACKER_INTERACTION_EXTENSION_NAME);
        if (m_handExt) extensions.push_back(XR_EXT_HAND_TRACKING_EXTENSION_NAME);
    }
    XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy_s(ci.applicationInfo.applicationName, "FBT IK lab");
    ci.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    ci.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    ci.enabledExtensionNames = extensions.data();
    XrInstance instance = XR_NULL_HANDLE;
    XrResult r = xrCreateInstance(&ci, &instance);
    if (XR_FAILED(r)) { m_status = Fail("xrCreateInstance", r); if (status) *status = m_status; return false; }
    m_instance = instance;

    XrSystemGetInfo si{XR_TYPE_SYSTEM_GET_INFO};
    si.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    if (XR_FAILED(r = xrGetSystem(instance, &si, &system))) { m_status = Fail("xrGetSystem (headset off?)", r); Release(); if (status) *status = m_status; return false; }
    m_system = system;

    // The runtime wants the device on its adapter; the lab's device is on the default one.
    PFN_xrGetD3D11GraphicsRequirementsKHR getReq = nullptr;
    xrGetInstanceProcAddr(instance, "xrGetD3D11GraphicsRequirementsKHR", reinterpret_cast<PFN_xrVoidFunction*>(&getReq));
    XrGraphicsRequirementsD3D11KHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    if (!getReq || XR_FAILED(r = getReq(instance, system, &req))) { m_status = Fail("D3D11 requirements", r); Release(); if (status) *status = m_status; return false; }
    {
        IDXGIDevice* dxgi = nullptr;
        IDXGIAdapter* adapter = nullptr;
        DXGI_ADAPTER_DESC desc{};
        if (SUCCEEDED(device->QueryInterface(IID_PPV_ARGS(&dxgi))) && SUCCEEDED(dxgi->GetAdapter(&adapter))) adapter->GetDesc(&desc);
        if (adapter) adapter->Release();
        if (dxgi) dxgi->Release();
        if (std::memcmp(&desc.AdapterLuid, &req.adapterLuid, sizeof(LUID)) != 0) {
            m_status = "the headset's GPU is not the lab's GPU";
            Release();
            if (status) *status = m_status;
            return false;
        }
    }

    XrGraphicsBindingD3D11KHR binding{XR_TYPE_GRAPHICS_BINDING_D3D11_KHR};
    binding.device = device;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    sci.next = &binding;
    sci.systemId = system;
    XrSession session = XR_NULL_HANDLE;
    if (XR_FAILED(r = xrCreateSession(instance, &sci, &session))) { m_status = Fail("xrCreateSession", r); Release(); if (status) *status = m_status; return false; }
    m_session = session;

    XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rs.poseInReferenceSpace.orientation.w = 1;
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    XrSpace stage = XR_NULL_HANDLE, view = XR_NULL_HANDLE;
    if (XR_FAILED(r = xrCreateReferenceSpace(session, &rs, &stage))) { m_status = Fail("STAGE space", r); Release(); if (status) *status = m_status; return false; }
    m_stage = stage;
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    xrCreateReferenceSpace(session, &rs, &view);
    m_viewSpace = view;
    CreateActions();

    // One swapchain and depth buffer per eye, at the runtime's recommended size.
    uint32_t viewCount = 0;
    XrViewConfigurationView vcv[2]{{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    if (XR_FAILED(r = xrEnumerateViewConfigurationViews(instance, system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &viewCount, vcv)) || viewCount != 2) {
        m_status = Fail("view configuration", r); Release(); if (status) *status = m_status; return false;
    }
    uint32_t formatCount = 0;
    xrEnumerateSwapchainFormats(session, 0, &formatCount, nullptr);
    std::vector<int64_t> formats(formatCount);
    xrEnumerateSwapchainFormats(session, formatCount, &formatCount, formats.data());
    int64_t format = 0;
    for (int64_t f : formats) if (f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB) { format = f; break; }
    if (!format && !formats.empty()) format = formats[0];
    for (int e = 0; e < 2; ++e) {
        Eye& eye = m_eyes[e];
        eye.width = vcv[e].recommendedImageRectWidth;
        eye.height = vcv[e].recommendedImageRectHeight;
        XrSwapchainCreateInfo sc{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        sc.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
        sc.format = format;
        sc.sampleCount = 1;
        sc.width = eye.width;
        sc.height = eye.height;
        sc.faceCount = 1;
        sc.arraySize = 1;
        sc.mipCount = 1;
        XrSwapchain chain = XR_NULL_HANDLE;
        if (XR_FAILED(r = xrCreateSwapchain(session, &sc, &chain))) { m_status = Fail("xrCreateSwapchain", r); Release(); if (status) *status = m_status; return false; }
        eye.swapchain = chain;
        uint32_t images = 0;
        xrEnumerateSwapchainImages(chain, 0, &images, nullptr);
        std::vector<XrSwapchainImageD3D11KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR});
        xrEnumerateSwapchainImages(chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
        for (auto& img : list) {
            D3D11_RENDER_TARGET_VIEW_DESC rd{};
            rd.Format = static_cast<DXGI_FORMAT>(format);
            rd.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
            ID3D11RenderTargetView* rtv = nullptr;
            device->CreateRenderTargetView(img.texture, &rd, &rtv);
            eye.rtvs.push_back(rtv);
        }
        D3D11_TEXTURE2D_DESC td{};
        td.Width = eye.width;
        td.Height = eye.height;
        td.MipLevels = 1;
        td.ArraySize = 1;
        td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        td.SampleDesc.Count = 1;
        td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        if (SUCCEEDED(device->CreateTexture2D(&td, nullptr, &eye.depth))) device->CreateDepthStencilView(eye.depth, nullptr, &eye.dsv);
    }
    m_status = "VR started, waiting for the headset";
    if (status) *status = m_status;
    return true;
}

// One action set: the controllers' grip pose and trigger per hand, and one pose action for the trackers' body roles.
void VrSession::CreateActions() {
    XrInstance instance = Inst(m_instance);
    XrSession session = Sess(m_session);
    XrActionSetCreateInfo asi{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy_s(asi.actionSetName, "lab");
    strcpy_s(asi.localizedActionSetName, "FBT IK lab");
    XrActionSet set = XR_NULL_HANDLE;
    if (XR_FAILED(xrCreateActionSet(instance, &asi, &set))) return;
    m_actionSet = set;
    XrPath hands[2];
    xrStringToPath(instance, "/user/hand/left", &hands[0]);
    xrStringToPath(instance, "/user/hand/right", &hands[1]);
    m_handPaths[0] = hands[0];
    m_handPaths[1] = hands[1];
    auto action = [&](XrActionType type, const char* name, const XrPath* sub, uint32_t subCount) {
        XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
        ai.actionType = type;
        strcpy_s(ai.actionName, name);
        strcpy_s(ai.localizedActionName, name);
        ai.countSubactionPaths = subCount;
        ai.subactionPaths = sub;
        XrAction a = XR_NULL_HANDLE;
        xrCreateAction(set, &ai, &a);
        return a;
    };
    XrAction pose = action(XR_ACTION_TYPE_POSE_INPUT, "hand_pose", hands, 2);
    XrAction trigger = action(XR_ACTION_TYPE_FLOAT_INPUT, "trigger", hands, 2);
    m_handPose = pose;
    m_trigger_ = trigger;
    auto suggest = [&](const char* profile, std::initializer_list<std::pair<XrAction, const char*>> list) {
        std::vector<XrActionSuggestedBinding> b;
        for (const auto& [a, path] : list) {
            XrPath p;
            if (XR_SUCCEEDED(xrStringToPath(instance, path, &p))) b.push_back({a, p});
        }
        XrPath prof;
        xrStringToPath(instance, profile, &prof);
        XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        sb.interactionProfile = prof;
        sb.suggestedBindings = b.data();
        sb.countSuggestedBindings = static_cast<uint32_t>(b.size());
        xrSuggestInteractionProfileBindings(instance, &sb);
    };
    suggest("/interaction_profiles/valve/index_controller",
            {{pose, "/user/hand/left/input/grip/pose"}, {pose, "/user/hand/right/input/grip/pose"},
             {trigger, "/user/hand/left/input/trigger/value"}, {trigger, "/user/hand/right/input/trigger/value"}});
    suggest("/interaction_profiles/htc/vive_controller",
            {{pose, "/user/hand/left/input/grip/pose"}, {pose, "/user/hand/right/input/grip/pose"},
             {trigger, "/user/hand/left/input/trigger/value"}, {trigger, "/user/hand/right/input/trigger/value"}});
    suggest("/interaction_profiles/khr/simple_controller",
            {{pose, "/user/hand/left/input/grip/pose"}, {pose, "/user/hand/right/input/grip/pose"},
             {trigger, "/user/hand/left/input/select/click"}, {trigger, "/user/hand/right/input/select/click"}});
    if (m_trackerExt) {
        const char* roles[3] = {"/user/vive_tracker_htcx/role/waist", "/user/vive_tracker_htcx/role/left_foot",
                                "/user/vive_tracker_htcx/role/right_foot"};
        XrPath rolePaths[3];
        for (int i = 0; i < 3; ++i) { xrStringToPath(instance, roles[i], &rolePaths[i]); m_rolePaths[i] = rolePaths[i]; }
        XrAction tracker = action(XR_ACTION_TYPE_POSE_INPUT, "tracker_pose", rolePaths, 3);
        m_trackerPose = tracker;
        suggest("/interaction_profiles/htc/vive_tracker_htcx",
                {{tracker, "/user/vive_tracker_htcx/role/waist/input/grip/pose"},
                 {tracker, "/user/vive_tracker_htcx/role/left_foot/input/grip/pose"},
                 {tracker, "/user/vive_tracker_htcx/role/right_foot/input/grip/pose"}});
    }
    XrSessionActionSetsAttachInfo at{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    at.countActionSets = 1;
    at.actionSets = &set;
    xrAttachSessionActionSets(session, &at);
    auto space = [&](XrAction a, XrPath sub) {
        XrActionSpaceCreateInfo sci{XR_TYPE_ACTION_SPACE_CREATE_INFO};
        sci.action = a;
        sci.subactionPath = sub;
        sci.poseInActionSpace.orientation.w = 1;
        XrSpace s = XR_NULL_HANDLE;
        xrCreateActionSpace(session, &sci, &s);
        return s;
    };
    for (int h = 0; h < 2; ++h) m_handSpaces[h] = space(pose, hands[h]);
    if (m_handExt) {
        PFN_xrCreateHandTrackerEXT create = nullptr;
        xrGetInstanceProcAddr(instance, "xrCreateHandTrackerEXT", reinterpret_cast<PFN_xrVoidFunction*>(&create));
        xrGetInstanceProcAddr(instance, "xrLocateHandJointsEXT", reinterpret_cast<PFN_xrVoidFunction*>(&m_locateJoints));
        xrGetInstanceProcAddr(instance, "xrDestroyHandTrackerEXT", reinterpret_cast<PFN_xrVoidFunction*>(&m_destroyTracker));
        for (int h = 0; create && m_locateJoints && h < 2; ++h) {
            XrHandTrackerCreateInfoEXT hi{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
            hi.hand = h == 0 ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
            hi.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
            XrHandTrackerEXT t = XR_NULL_HANDLE;
            if (XR_SUCCEEDED(create(session, &hi, &t))) m_handTrackers[h] = t;
        }
    }
    if (m_trackerPose)
        for (int i = 0; i < 3; ++i) m_trackerSpaces[i] = space(static_cast<XrAction>(m_trackerPose), m_rolePaths[i]);
}

// Sync the actions and locate every device in STAGE at the frame's display time.
void VrSession::UpdateDevices() {
    for (auto& d : m_devices) d.valid = false;
    m_trigger[0] = m_trigger[1] = 0;
    if (!m_actionSet) return;
    XrActiveActionSet active{static_cast<XrActionSet>(m_actionSet), XR_NULL_PATH};
    XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    if (XR_FAILED(xrSyncActions(Sess(m_session), &sync))) return;
    auto locate = [&](void* s, DevicePose& out) {
        if (!s) return;
        XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
        constexpr XrSpaceLocationFlags kNeed = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        if (XR_SUCCEEDED(xrLocateSpace(Space(s), Space(m_stage), m_displayTime, &loc)) && (loc.locationFlags & kNeed) == kNeed) {
            out.valid = true;
            out.pos = ToModel(loc.pose.position);
            out.rot = ToModel(loc.pose.orientation);
        }
    };
    locate(m_viewSpace, m_devices[kDevHead]);
    locate(m_handSpaces[0], m_devices[kDevLeftHand]);
    locate(m_handSpaces[1], m_devices[kDevRightHand]);
    for (int i = 0; i < 3; ++i) locate(m_trackerSpaces[i], m_devices[kDevWaist + i]);
    // The wrist joints of the hand-tracking skeleton.
    for (int h = 0; h < 2 && m_locateJoints; ++h) {
        if (!m_handTrackers[h]) continue;
        XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT]{};
        XrHandJointLocationsEXT locs{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
        locs.jointCount = XR_HAND_JOINT_COUNT_EXT;
        locs.jointLocations = joints;
        XrHandJointsLocateInfoEXT li{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
        li.baseSpace = Space(m_stage);
        li.time = m_displayTime;
        constexpr XrSpaceLocationFlags kNeed = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        const auto locateFn = reinterpret_cast<PFN_xrLocateHandJointsEXT>(m_locateJoints);
        if (XR_SUCCEEDED(locateFn(static_cast<XrHandTrackerEXT>(m_handTrackers[h]), &li, &locs)) && locs.isActive &&
            (joints[XR_HAND_JOINT_WRIST_EXT].locationFlags & kNeed) == kNeed) {
            DevicePose& out = m_devices[h == 0 ? kDevLeftWrist : kDevRightWrist];
            out.valid = true;
            out.pos = ToModel(joints[XR_HAND_JOINT_WRIST_EXT].pose.position);
            out.rot = ToModel(joints[XR_HAND_JOINT_WRIST_EXT].pose.orientation);
        }
    }
    for (int h = 0; h < 2; ++h) {
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = static_cast<XrAction>(m_trigger_);
        gi.subactionPath = m_handPaths[h];
        XrActionStateFloat st{XR_TYPE_ACTION_STATE_FLOAT};
        if (XR_SUCCEEDED(xrGetActionStateFloat(Sess(m_session), &gi, &st)) && st.isActive) m_trigger[h] = st.currentState;
    }
}

void VrSession::Release() {
    for (Eye& eye : m_eyes) {
        for (auto* rtv : eye.rtvs) if (rtv) rtv->Release();
        eye.rtvs.clear();
        if (eye.dsv) { eye.dsv->Release(); eye.dsv = nullptr; }
        if (eye.depth) { eye.depth->Release(); eye.depth = nullptr; }
        if (eye.swapchain) { xrDestroySwapchain(Chain(eye.swapchain)); eye.swapchain = nullptr; }
    }
    for (auto& s : m_handSpaces) if (s) { xrDestroySpace(Space(s)); s = nullptr; }
    for (auto& s : m_trackerSpaces) if (s) { xrDestroySpace(Space(s)); s = nullptr; }
    for (auto& t : m_handTrackers)
        if (t) {
            if (m_destroyTracker) reinterpret_cast<PFN_xrDestroyHandTrackerEXT>(m_destroyTracker)(static_cast<XrHandTrackerEXT>(t));
            t = nullptr;
        }
    m_locateJoints = m_destroyTracker = nullptr;
    if (m_actionSet) { xrDestroyActionSet(static_cast<XrActionSet>(m_actionSet)); m_actionSet = nullptr; }
    m_handPose = m_trigger_ = m_trackerPose = nullptr;
    for (auto& d : m_devices) d.valid = false;
    if (m_viewSpace) { xrDestroySpace(Space(m_viewSpace)); m_viewSpace = nullptr; }
    if (m_stage) { xrDestroySpace(Space(m_stage)); m_stage = nullptr; }
    if (m_session) { xrDestroySession(Sess(m_session)); m_session = nullptr; }
    if (m_instance) { xrDestroyInstance(Inst(m_instance)); m_instance = nullptr; }
    m_begun = m_frameOpen = false;
    m_state = 0;
}

void VrSession::Stop() {
    if (m_session && m_begun) xrEndSession(Sess(m_session));
    Release();
    m_status = "VR off";
}

void VrSession::PollEvents() {
    XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(Inst(m_instance), &ev) == XR_SUCCESS) {
        if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            const auto* s = reinterpret_cast<const XrEventDataSessionStateChanged*>(&ev);
            m_state = s->state;
            if (s->state == XR_SESSION_STATE_READY) {
                XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                m_begun = XR_SUCCEEDED(xrBeginSession(Sess(m_session), &bi));
                m_status = m_begun ? "VR running" : "xrBeginSession failed";
            } else if (s->state == XR_SESSION_STATE_STOPPING) {
                xrEndSession(Sess(m_session));
                m_begun = false;
                m_status = "VR stopped by the runtime";
            } else if (s->state == XR_SESSION_STATE_EXITING || s->state == XR_SESSION_STATE_LOSS_PENDING) {
                Release();
                m_status = "VR ended by the runtime";
                return;
            }
        }
        ev = {XR_TYPE_EVENT_DATA_BUFFER};
    }
}

bool VrSession::BeginFrame(std::vector<EyeTarget>* eyes) {
    eyes->clear();
    if (!m_session) return false;
    PollEvents();
    if (!m_session || !m_begun) return false;
    XrFrameState fs{XR_TYPE_FRAME_STATE};
    if (XR_FAILED(xrWaitFrame(Sess(m_session), nullptr, &fs))) return false;
    if (XR_FAILED(xrBeginFrame(Sess(m_session), nullptr))) return false;
    m_frameOpen = true;
    m_displayTime = fs.predictedDisplayTime;
    m_shouldRender = fs.shouldRender == XR_TRUE;
    if (m_state == XR_SESSION_STATE_FOCUSED) UpdateDevices();
    if (!m_shouldRender) return false;

    XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
    li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    li.displayTime = fs.predictedDisplayTime;
    li.space = Space(m_stage);
    XrViewState vs{XR_TYPE_VIEW_STATE};
    XrView views[2]{{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
    uint32_t count = 0;
    if (XR_FAILED(xrLocateViews(Sess(m_session), &li, &vs, 2, &count, views)) || count != 2 ||
        !(vs.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) || !(vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT)) {
        m_shouldRender = false;
        return false;
    }
    for (int e = 0; e < 2; ++e) {
        const XrView& v = views[e];
        ViewPose& vp = m_views[e];
        vp.pos[0] = v.pose.position.x; vp.pos[1] = v.pose.position.y; vp.pos[2] = v.pose.position.z;
        vp.rot[0] = v.pose.orientation.x; vp.rot[1] = v.pose.orientation.y; vp.rot[2] = v.pose.orientation.z; vp.rot[3] = v.pose.orientation.w;
        vp.fov[0] = v.fov.angleLeft; vp.fov[1] = v.fov.angleRight; vp.fov[2] = v.fov.angleUp; vp.fov[3] = v.fov.angleDown;

        Eye& eye = m_eyes[e];
        XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        xrAcquireSwapchainImage(Chain(eye.swapchain), &ai, &m_imageIndex[e]);
        XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wi.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(Chain(eye.swapchain), &wi);

        // The eye's camera in model space: its right, up and back axes, from the STAGE pose.
        // Through the room transform (identity until calibrated): the player's eyes into V's frame.
        const Quat q = Normalize(m_roomYawInverse * ToModel(v.pose.orientation));
        const Vec3 p = Rotate(m_roomYawInverse, ToModel(v.pose.position) - m_roomOrigin) * m_roomScale;
        // The camera's own axes (OpenXR view: x right, y up, +z back) in model axes, turned by the eye's rotation.
        const Vec3 right = Rotate(q, ToModel(XrVector3f{1, 0, 0})), up = Rotate(q, ToModel(XrVector3f{0, 1, 0})),
                   back = Rotate(q, ToModel(XrVector3f{0, 0, 1}));
        const float view[16] = {right.x, up.x, back.x, 0, right.y, up.y, back.y, 0, right.z, up.z, back.z, 0,
                                -Dot(right, p), -Dot(up, p), -Dot(back, p), 1};
        // Asymmetric projection from the eye's field of view, right-handed, D3D depth 0..1.
        const float l = std::tan(v.fov.angleLeft), rr = std::tan(v.fov.angleRight), u = std::tan(v.fov.angleUp),
                    d = std::tan(v.fov.angleDown), zn = 0.05f, zf = 100.0f;
        const float proj[16] = {2 / (rr - l), 0, 0, 0, 0, 2 / (u - d), 0, 0, (rr + l) / (rr - l), (u + d) / (u - d), zf / (zn - zf), -1,
                                0, 0, zn * zf / (zn - zf), 0};
        EyeTarget t;
        t.rtv = eye.rtvs[m_imageIndex[e]];
        t.dsv = eye.dsv;
        t.width = eye.width;
        t.height = eye.height;
        t.eye = p;
        for (int c = 0; c < 4; ++c)
            for (int row = 0; row < 4; ++row) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += proj[k * 4 + row] * view[c * 4 + k];
                t.viewProj[c * 4 + row] = s;
            }
        eyes->push_back(t);
    }
    return true;
}

void VrSession::EndFrame() {
    if (!m_session || !m_frameOpen) return;
    XrCompositionLayerProjectionView pv[2]{{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    const XrCompositionLayerBaseHeader* layers[1] = {reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer)};
    XrFrameEndInfo ei{XR_TYPE_FRAME_END_INFO};
    ei.displayTime = m_displayTime;
    ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    if (m_shouldRender) {
        for (int e = 0; e < 2; ++e) {
            XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            xrReleaseSwapchainImage(Chain(m_eyes[e].swapchain), &ri);
            const ViewPose& vp = m_views[e];
            pv[e].pose.position = {vp.pos[0], vp.pos[1], vp.pos[2]};
            pv[e].pose.orientation = {vp.rot[0], vp.rot[1], vp.rot[2], vp.rot[3]};
            pv[e].fov = {vp.fov[0], vp.fov[1], vp.fov[2], vp.fov[3]};
            pv[e].subImage.swapchain = Chain(m_eyes[e].swapchain);
            pv[e].subImage.imageRect = {{0, 0}, {static_cast<int32_t>(m_eyes[e].width), static_cast<int32_t>(m_eyes[e].height)}};
        }
        layer.space = Space(m_stage);
        layer.viewCount = 2;
        layer.views = pv;
        ei.layerCount = 1;
        ei.layers = layers;
    }
    xrEndFrame(Sess(m_session), &ei);
    m_frameOpen = false;
}

bool VrSession::HeadPose(Vec3* pos, Quat* rot) const {
    if (!m_session || !m_viewSpace) return false;
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_FAILED(xrLocateSpace(Space(m_viewSpace), Space(m_stage), m_displayTime, &loc))) return false;
    if (pos) *pos = ToModel(loc.pose.position);
    if (rot) *rot = ToModel(loc.pose.orientation);
    return true;
}

}  // namespace lab
