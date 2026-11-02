#pragma once

// Shaded boxes in DX11 for the lab's robot: one unit cube, drawn once per box with its own transform, a depth buffer,
// two-sided faces (a box's axes may be left-handed), a key light plus a headlight, and every face tinted by its own
// axis (x red, y green, z blue, negative side darker) so rotations show.
#include "Fbt/FbtMath.hpp"
#include <d3d11.h>

namespace lab {

class BoxRenderer {
public:
    bool Init(ID3D11Device* device);
    void Release();
    // Call after the back buffer is (re)created; the depth buffer follows its size.
    void Resize(ID3D11Device* device, UINT width, UINT height);
    // viewProj: column-major, D3D clip depth 0..1. eye: the camera position (model space), for the headlight. depth: the
    // target's own depth buffer (a headset eye), else the renderer's (the window).
    void Begin(ID3D11DeviceContext* context, ID3D11RenderTargetView* target, const float viewProj[16], wrpuppets::fbt::Vec3 eye,
               ID3D11DepthStencilView* depth = nullptr);
    // A box: centre and its three edge vectors (axis * size), model space.
    void Draw(ID3D11DeviceContext* context, wrpuppets::fbt::Vec3 center, const wrpuppets::fbt::Vec3 axes[3], const float color[3]);
    ID3D11DepthStencilView* Depth() const { return m_dsv; }

    // A triangle mesh (V's body) through the same shader, untinted: its vertices are placed by centre + axes as a box's
    // corners are (identity axes: model space as is; the mirror passes its reflection). SetMesh once (sizes the
    // buffers), UpdateMesh once per frame, DrawMesh per view and part; Draw after it rebinds the cube.
    struct MeshVertex { float pos[3], normal[3]; };
    bool SetMesh(ID3D11Device* device, const unsigned* indices, UINT indexCount, UINT vertexCount);
    void UpdateMesh(ID3D11DeviceContext* context, const MeshVertex* vertices, UINT vertexCount);
    void DrawMesh(ID3D11DeviceContext* context, UINT firstIndex, UINT indexCount, wrpuppets::fbt::Vec3 center, const wrpuppets::fbt::Vec3 axes[3],
                  const float color[3]);

private:
    ID3D11Buffer *m_vb = nullptr, *m_ib = nullptr, *m_cb = nullptr;
    ID3D11Buffer *m_meshVb = nullptr, *m_meshIb = nullptr;
    UINT m_meshVertices = 0;
    void Constants(ID3D11DeviceContext* context, wrpuppets::fbt::Vec3 center, const wrpuppets::fbt::Vec3 axes[3], const float color[3], float tint);
    void BindCube(ID3D11DeviceContext* context);
    bool m_cubeBound = true;
    ID3D11VertexShader* m_vs = nullptr;
    ID3D11PixelShader* m_ps = nullptr;
    ID3D11InputLayout* m_layout = nullptr;
    ID3D11RasterizerState* m_raster = nullptr;
    ID3D11DepthStencilState* m_depthState = nullptr;
    ID3D11Texture2D* m_depth = nullptr;
    ID3D11DepthStencilView* m_dsv = nullptr;
    float m_viewProj[16]{};
    wrpuppets::fbt::Vec3 m_eye;
};

}  // namespace lab
