# CYBERPUNK FBT LAB -- Cyberpunk 2077's full-body tracking outside the game: the full-body IK (the wrpuppets-fbt-ik
# library, a submodule) on V's skeleton, a desktop and VR tool to develop, check and report on it. It fetches its own dependencies (ImGui, ImGuizmo, cgltf: MIT; the OpenXR
# SDK: Apache 2.0).
#   fbt_ik_lab   the lab: V's rig as a body (the game's mesh when exported) or a robot, goals with gizmos, VR with the
#                real devices, calibration, mirror, flight recorder
#   fbt_ik_test  the stress test, the recording replay and the mesh check (console)
# Build (Visual Studio 2022 or later, CMake 3.24+):
#   git submodule update --init
#   cmake -S . -B build -A x64
#   cmake --build build --config Release
# See README.md.
cmake_minimum_required(VERSION 3.24)
project(fbt_ik_lab VERSION 0.1 LANGUAGES CXX)

# The IK: the library wrpuppets-fbt-ik, the submodule in external/ (git submodule update --init), else
# -DWRPUPPETS_FBT_IK_DIR=<a checkout of it>.
set(WRPUPPETS_FBT_IK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/wrpuppets-fbt-ik" CACHE PATH "The wrpuppets-fbt-ik checkout")
if(NOT EXISTS "${WRPUPPETS_FBT_IK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "WRPUPPETS_FBT_IK_DIR (${WRPUPPETS_FBT_IK_DIR}) is not a wrpuppets-fbt-ik checkout: git submodule update --init, or pass -DWRPUPPETS_FBT_IK_DIR=<path>")
endif()
add_subdirectory(${WRPUPPETS_FBT_IK_DIR} wrpuppets-fbt-ik)

include(FetchContent)
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.90.9)
FetchContent_Declare(imguizmo GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
                     GIT_TAG 105e4fde45e4e74aae8973d98750820ef7aca7e2)  # master, 2026 (the tags predate ImGui 1.90)
# OpenXR (Apache 2.0): the same SDK release the port uses; the loader is built static.
# cgltf (MIT, one header): V's body mesh (glTF binary from WolvenKit).
FetchContent_Declare(cgltf GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git GIT_TAG v1.14)
FetchContent_Declare(openxr GIT_REPOSITORY https://github.com/KhronosGroup/OpenXR-SDK.git GIT_TAG release-1.0.34)
set(DYNAMIC_LOADER OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(openxr)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)
endif()
FetchContent_GetProperties(cgltf)
if(NOT cgltf_POPULATED)
  FetchContent_Populate(cgltf)
endif()
FetchContent_GetProperties(imguizmo)
if(NOT imguizmo_POPULATED)
  FetchContent_Populate(imguizmo)
endif()

add_executable(fbt_ik_lab WIN32
    main.cpp
    Robot.cpp
    BoxRenderer.cpp
    VMesh.cpp
    VrSession.cpp
    ${imguizmo_SOURCE_DIR}/src/ImGuizmo.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
)
target_compile_features(fbt_ik_lab PRIVATE cxx_std_20)
target_compile_definitions(fbt_ik_lab PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE IMGUI_DEFINE_MATH_OPERATORS)
target_include_directories(fbt_ik_lab PRIVATE ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends
                           ${imguizmo_SOURCE_DIR}/src ${cgltf_SOURCE_DIR})
target_link_libraries(fbt_ik_lab PRIVATE wrpuppets_fbt_ik d3d11 dxgi d3dcompiler openxr_loader)

add_executable(fbt_ik_test ik_test.cpp VMesh.cpp)
target_link_libraries(fbt_ik_test PRIVATE wrpuppets_fbt_ik)
target_compile_features(fbt_ik_test PRIVATE cxx_std_20)
target_include_directories(fbt_ik_test PRIVATE ${cgltf_SOURCE_DIR})
target_compile_definitions(fbt_ik_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
