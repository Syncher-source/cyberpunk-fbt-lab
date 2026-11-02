#include "BoxRenderer.hpp"
#include <d3dcompiler.h>
#include <cstring>

using namespace wrpuppets::fbt;

namespace lab {
namespace {
const char* kShader = R"(
cbuffer Draw : register(b0) {
    float4x4 viewProj;
    float4 center;
    float4 axisX, axisY, axisZ;   // edge vectors (axis * size)
    float4 color;
    float4 eye;
};
struct VsIn { float3 pos : POSITION; float3 normal : NORMAL; };
struct VsOut { float4 pos : SV_POSITION; float3 normal : NORMAL; float3 world : TEXCOORD0; float3 face : TEXCOORD1; };
VsOut vs(VsIn i) {
    VsOut o;
    float3 w = center.xyz + axisX.xyz * i.pos.x + axisY.xyz * i.pos.y + axisZ.xyz * i.pos.z;
    // Normal of a scaled box: each face normal follows its axis direction (sizes differ per axis).
    float3 n = normalize(axisX.xyz) * i.normal.x + normalize(axisY.xyz) * i.normal.y + normalize(axisZ.xyz) * i.normal.z;
    o.pos = mul(viewProj, float4(w, 1));
    o.normal = n;
    o.world = w;
    o.face = i.normal;   // the face in the box's own frame: +-x, +-y, +-z
    return o;
}
float4 ps(VsOut i, bool front : SV_IsFrontFace) : SV_TARGET {
    float3 n = normalize(i.normal);
    float3 toEye = normalize(eye.xyz - i.world);
    if (dot(n, toEye) < 0) n = -n;                       // two-sided
    float3 key = normalize(float3(0.4, 0.6, 0.7));
    float light = 0.28 + 0.45 * saturate(dot(n, key)) + 0.35 * saturate(dot(n, toEye));
    // Each face tinted by its own axis (x red, y green, z blue; the negative side darker), half and half with the
    // box's colour: a box that rolls or flips shows the wrong colours at once.
    float3 f = i.face;
    float3 axis = abs(f.x) > 0.5 ? float3(1, 0.25, 0.25) : (abs(f.y) > 0.5 ? float3(0.25, 1, 0.3) : float3(0.3, 0.45, 1));
    float sideShade = (f.x + f.y + f.z) > 0 ? 1.0 : 0.5;
    float3 tinted = lerp(color.rgb, axis * sideShade, color.a);   // a: the tint (boxes 0.5, the mesh none)
    return float4(tinted * light, 1);
}
)";

struct Vertex { float pos[3], normal[3]; };
struct DrawConstants {
    float viewProj[16];
    float center[4], axisX[4], axisY[4], axisZ[4], color[4], eye[4];
};
}  // namespace

bool BoxRenderer::Init(ID3D11Device* device) {
    // A unit cube, -0.5..0.5, four vertices per face so each face has its own normal.
    Vertex v[24];
    unsigned short idx[36];
    const float n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int f = 0; f < 6; ++f) {
        const int a = f / 2, b = (a + 1) % 3, c = (a + 2) % 3;
        const float s = n[f][a];
        const float corners[4][2] = {{-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
        for (int k = 0; k < 4; ++k) {
            Vertex& vx = v[f * 4 + k];
            vx.pos[a] = 0.5f * s;
            vx.pos[b] = corners[k][0];
            vx.pos[c] = corners[k][1];
            for (int j = 0; j < 3; ++j) vx.normal[j] = n[f][j];
        }
        const unsigned short base = static_cast<unsigned short>(f * 4);
        const unsigned short tri[6] = {0, 1, 2, 0, 2, 3};
        for (int k = 0; k < 6; ++k) idx[f * 6 + k] = base + tri[k];
    }
    D3D11_BUFFER_DESC bd{};
    D3D11_SUBRESOURCE_DATA init{};
    bd.Usage = D3D11_USAGE_IMMUTABLE;
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    bd.ByteWidth = sizeof(v);
    init.pSysMem = v;
    if (FAILED(device->CreateBuffer(&bd, &init, &m_vb))) return false;
    bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
    bd.ByteWidth = sizeof(idx);
    init.pSysMem = idx;
    if (FAILED(device->CreateBuffer(&bd, &init, &m_ib))) return false;
    bd = {};
    bd.Usage = D3D11_USAGE_DYNAMIC;
    bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bd.ByteWidth = sizeof(DrawConstants);
    if (FAILED(device->CreateBuffer(&bd, nullptr, &m_cb))) return false;

    ID3DBlob *vsBlob = nullptr, *psBlob = nullptr, *errors = nullptr;
    if (FAILED(D3DCompile(kShader, std::strlen(kShader), "box", nullptr, nullptr, "vs", "vs_5_0", 0, 0, &vsBlob, &errors)) ||
        FAILED(D3DCompile(kShader, std::strlen(kShader), "box", nullptr, nullptr, "ps", "ps_5_0", 0, 0, &psBlob, &errors))) {
        if (errors) {
            OutputDebugStringA(static_cast<const char*>(errors->GetBufferPointer()));
            errors->Release();
        }
        if (vsBlob) vsBlob->Release();
        return false;
    }
    device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, &m_vs);
    device->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), nullptr, &m_ps);
    const D3D11_INPUT_ELEMENT_DESC layout[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
    };
    device->CreateInputLayout(layout, 2, vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &m_layout);
    vsBlob->Release();
    psBlob->Release();

    D3D11_RASTERIZER_DESC rd{};
    rd.FillMode = D3D11_FILL_SOLID;
    rd.CullMode = D3D11_CULL_NONE;
    rd.DepthClipEnable = TRUE;
    device->CreateRasterizerState(&rd, &m_raster);
    D3D11_DEPTH_STENCIL_DESC dd{};
    dd.DepthEnable = TRUE;
    dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    dd.DepthFunc = D3D11_COMPARISON_LESS;
    device->CreateDepthStencilState(&dd, &m_depthState);
    return m_vs && m_ps && m_layout && m_raster && m_depthState;
}

void BoxRenderer::Resize(ID3D11Device* device, UINT width, UINT height) {
    if (m_dsv) { m_dsv->Release(); m_dsv = nullptr; }
    if (m_depth) { m_depth->Release(); m_depth = nullptr; }
    D3D11_TEXTURE2D_DESC td{};
    td.Width = width;
    td.Height = height;
    td.MipLevels = 1;
    td.ArraySize = 1;
    td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    td.SampleDesc.Count = 1;
    td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    if (SUCCEEDED(device->CreateTexture2D(&td, nullptr, &m_depth))) device->CreateDepthStencilView(m_depth, nullptr, &m_dsv);
}

void BoxRenderer::Begin(ID3D11DeviceContext* context, ID3D11RenderTargetView* target, const float viewProj[16], Vec3 eye,
                        ID3D11DepthStencilView* depth) {
    std::memcpy(m_viewProj, viewProj, sizeof(m_viewProj));
    m_eye = eye;
    ID3D11DepthStencilView* dsv = depth ? depth : m_dsv;
    if (dsv) context->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH, 1.0f, 0);
    context->OMSetRenderTargets(1, &target, dsv);
    context->OMSetDepthStencilState(m_depthState, 0);
    context->RSSetState(m_raster);
    BindCube(context);
    context->IASetInputLayout(m_layout);
    context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context->VSSetShader(m_vs, nullptr, 0);
    context->PSSetShader(m_ps, nullptr, 0);
    context->VSSetConstantBuffers(0, 1, &m_cb);
    context->PSSetConstantBuffers(0, 1, &m_cb);
}

void BoxRenderer::Constants(ID3D11DeviceContext* context, Vec3 center, const Vec3 axes[3], const float color[3], float tint) {
    D3D11_MAPPED_SUBRESOURCE mapped{};
    if (FAILED(context->Map(m_cb, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) return;
    DrawConstants c{};
    std::memcpy(c.viewProj, m_viewProj, sizeof(c.viewProj));
    auto put = [](float* dst, Vec3 v) { dst[0] = v.x; dst[1] = v.y; dst[2] = v.z; dst[3] = 0; };
    put(c.center, center);
    put(c.axisX, axes[0]);
    put(c.axisY, axes[1]);
    put(c.axisZ, axes[2]);
    c.color[0] = color[0]; c.color[1] = color[1]; c.color[2] = color[2]; c.color[3] = tint;
    put(c.eye, m_eye);
    std::memcpy(mapped.pData, &c, sizeof(c));
    context->Unmap(m_cb, 0);
}

void BoxRenderer::BindCube(ID3D11DeviceContext* context) {
    const UINT stride = sizeof(Vertex), offset = 0;
    context->IASetVertexBuffers(0, 1, &m_vb, &stride, &offset);
    context->IASetIndexBuffer(m_ib, DXGI_FORMAT_R16_UINT, 0);
    m_cubeBound = true;
}

void BoxRenderer::Draw(ID3D11DeviceContext* context, Vec3 center, const Vec3 axes[3], const float color[3]) {
    if (!m_cubeBound) BindCube(context);
    Constants(context, center, axes, color, 0.5f);
    context->DrawIndexed(36, 0, 0);
}

bool BoxRenderer::SetMesh(ID3D11Device* device, const unsigned* indices, UINT indexCount, UINT vertexCount) {
    if (m_meshVb) { m_meshVb->Release(); m_meshVb = nullptr; }
    if (m_meshIb) { m_meshIb->Release(); m_meshIb = nullptr; }
    m_meshVertices = 0;
    if (!indexCount || !vertexCount) return false;
    D3D11_BUFFER_DESC bd{};
    D3D11_SUBRESOURCE_DATA init{};
    bd.Usage = D3D11_USAGE_IMMUTABLE;
    bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
    bd.ByteWidth = indexCount * sizeof(unsigned);
    init.pSysMem = indices;
    if (FAILED(device->CreateBuffer(&bd, &init, &m_meshIb))) return false;
    bd = {};
    bd.Usage = D3D11_USAGE_DYNAMIC;
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bd.ByteWidth = vertexCount * sizeof(MeshVertex);
    if (FAILED(device->CreateBuffer(&bd, nullptr, &m_meshVb))) return false;
    m_meshVertices = vertexCount;
    return true;
}

void BoxRenderer::UpdateMesh(ID3D11DeviceContext* context, const MeshVertex* vertices, UINT vertexCount) {
    if (!m_meshVb || vertexCount > m_meshVertices) return;
    D3D11_MAPPED_SUBRESOURCE mapped{};
    if (FAILED(context->Map(m_meshVb, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) return;
    std::memcpy(mapped.pData, vertices, vertexCount * sizeof(MeshVertex));
    context->Unmap(m_meshVb, 0);
}

void BoxRenderer::DrawMesh(ID3D11DeviceContext* context, UINT firstIndex, UINT indexCount, Vec3 center, const Vec3 axes[3],
                           const float color[3]) {
    if (!m_meshVb || !m_meshIb) return;
    const UINT stride = sizeof(MeshVertex), offset = 0;
    context->IASetVertexBuffers(0, 1, &m_meshVb, &stride, &offset);
    context->IASetIndexBuffer(m_meshIb, DXGI_FORMAT_R32_UINT, 0);
    m_cubeBound = false;
    Constants(context, center, axes, color, 0.0f);
    context->DrawIndexed(indexCount, firstIndex, 0);
}

template <class T> void SafeRelease(T*& p) {
    if (p) { p->Release(); p = nullptr; }
}

void BoxRenderer::Release() {
    SafeRelease(m_vb); SafeRelease(m_ib); SafeRelease(m_cb); SafeRelease(m_vs); SafeRelease(m_ps); SafeRelease(m_layout);
    SafeRelease(m_raster); SafeRelease(m_depthState); SafeRelease(m_dsv); SafeRelease(m_depth);
    SafeRelease(m_meshVb); SafeRelease(m_meshIb);
}

}  // namespace lab
