/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

struct AttributeQuantizationInfo {
    // quantized base
    unsigned int quantizedBase;
    // base for dequantization
    float        base;
    // factor for dequantization; value = quantize * factor + lower
    float        factor;
    // number of bits for quantized value
    unsigned int bits;
};

struct QuantizationInfo {
    // Offset of meshlet vertex data in bytes
    unsigned int              byteOffset;
    // Quantization info for all 8 attributes:
    //   [0:2] = vertex position
    //   [3:5] = normal
    //   [6:7] = texCoord
    AttributeQuantizationInfo quantizationInfo[8];
};

struct MeshletInfo {
    // Offset of of meshlet indices in index buffer in bytes
    unsigned int primitiveOffset;

    // Number of vertices in meshlet
    unsigned int vertexCount;
    // Number of primitives in meshlet
    unsigned int primitiveCount;

    // Quantization info for meshlet
    QuantizationInfo quantizationInfo;

    // ...
};

StructuredBuffer<MeshletInfo> Meshlets : register();
ByteAddressBuffer Vertices : register();

struct ConstantBufferData {
    float4x4 viewProjectionMatrix;
};

ConstantBuffer<ConstantBufferData> DynamicConst : register(b1);

Vertex LoadVertex(in uint vertexId, in QuantizationInfo quantizationInfo)
{
    // Result
    Vertex vertex;

    // 16 bytes per vertex
    const uint vertexByteAddress = quantizationInfo.byteOffset + vertexId * 16;
    // ByteAddressBuffer address must be 4 byte aligned
    const uint byteAddress = vertexByteAddress & ~0x3;
    
    const uint16_t4 data0 = Vertices.Load<uint16_t4>(byteAddress);
    const uint16_t4 data1 = Vertices.Load<uint16_t4>(byteAddress + 8);
    
    // Vertex Position
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[0];
        vertex.position[0] = (channel.quantizedBase + data0[0]) * channel.factor + channel.base;
    }
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[1];
        vertex.position[1] = (channel.quantizedBase + data0[1]) * channel.factor + channel.base;
    }
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[2];
        vertex.position[2] = (channel.quantizedBase + data0[2]) * channel.factor + channel.base;
    }
    
    // Normal
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[3];
        vertex.attributes.normal[0] = (channel.quantizedBase + data0[3]) * channel.factor + channel.base;
    }
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[4];
        vertex.attributes.normal[1] = (channel.quantizedBase + data1[0]) * channel.factor + channel.base;
    }
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[5];
        vertex.attributes.normal[2] = (channel.quantizedBase + data1[1]) * channel.factor + channel.base;
    }

    // TexCoord
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[6];
        vertex.attributes.texCoord[0] = (channel.quantizedBase + data1[2]) * channel.factor + channel.base;
    }
    {
        const AttributeQuantizationInfo channel = quantizationInfo.quantizationInfo[7];
        vertex.attributes.texCoord[1] = (channel.quantizedBase + data1[3]) * channel.factor + channel.base;
    }

    vertex.position.w = 1.0;
    vertex.position = mul(DynamicConst.viewProjectionMatrix, vertex.position);

    return vertex;
}

[NumThreads(128, 1, 1)]
[OutputTopology("triangle")]
void MeshShader(
    uint threadId : SV_GroupThreadID,
    uint meshletId : SV_GroupID,
    out indices uint3 tris[256],
    out vertices Vertex verts[128])
{
    const MeshletInfo meshlet = Meshlets[meshletId];
    
    SetMeshOutputCounts(meshlet.vertexCount, meshlet.primitiveCount);

    // Load triangle indices here. See GTS.hlsl and GTS-Reuse.hlsl
    // ...

    if (threadId < meshlet.vertexCount) {
        verts[threadId] = LoadVertex(threadId, meshlet.quantizationInfo);
    }
}
