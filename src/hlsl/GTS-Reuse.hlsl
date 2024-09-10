/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

struct MeshletInfo {
    // Offset of of meshlet indices in index buffer in bytes
    unsigned int primitiveOffset;

    // Number of vertices in meshlet
    unsigned int vertexCount;
    // Number of primitives in meshlet
    unsigned int primitiveCount;

    // Number of reused indices in lookup table
    unsigned int reusedIndexCount;

    // ...
};

StructuredBuffer<MeshletInfo> Meshlets : register();
StructuredBuffer<uint> Indices : register();

struct Vertex {
    float4 position : SV_POSITION;
    float3 normal   : NORMAL0;
    float2 texCoord : TEXCOORD0;
}

// 256 x 1 bit = 8 DWORDs
#define TRIANGLE_LR_FLAG_SIZE          8
#define TRIANGLE_INCREMENT_FLAG_OFFSET (TRIANGLE_LR_FLAG_SIZE)
// 256 x 1 bit = 8 DWORDs
#define TRIANGLE_INCREMENT_FLAG_SIZE   8
#define TRIANGLE_INDEX_OFFSET          (TRIANGLE_INCREMENT_FLAG_OFFSET + TRIANGLE_INCREMENT_FLAG_SIZE)
// 256 x 8 bit = 64 DWORDs
#define TRIANGLE_INDEX_SIZE            64

#define INDEX_BUFFER_CACHE_SIZE        (TRIANGLE_LR_FLAG_SIZE + TRIANGLE_INCREMENT_FLAG_SIZE + TRIANGLE_INDEX_SIZE)

// IndexBufferCache[0:8]   = L/R flags
// IndexBufferCache[9:16]  = increment flags
// IndexBufferCache[17:80] = index lookup table
groupshared uint IndexBufferCache[INDEX_BUFFER_CACHE_SIZE];

// Prefix sums of increment flags
// IncrementPrefixCache[0] = countbits(incrementFlags[0:32])
// IncrementPrefixCache[1] = countbits(incrementFlags[0:64])
// ...
// IncrementPrefixCache[7] = countbits(incrementFlags[0:224])
groupshared uint IncrementPrefixCache[7];


// Load L/R-flag for a single triangle
// 
// * ------ i2 ------ *
//  \  L   /  \   R  /
//   \    /    \    /
//    \  /      \  /
//     i0 ------ i1 
//
// L = false
// R = true
bool LoadTriangleFlag(in int triangleIndex)
{
    // Index of DWORD which contains L/R flag for triangleIndex
    const uint dwordIndex = triangleIndex / 32;
    // Bit index of L/R flag in DWORD
    const uint bitIndex   = triangleIndex % 32;
    
    // Extract triangle flag bit
    return IndexBufferCache[dwordIndex] & (1U << bitIndex);
}

// Return last 32 L/R-flags for a triangle
uint LoadLast32TriangleFlags(in int triangleIndex)
{
    // Index of DWORD which contains L/R flag for triangleIndex
    int dwordIndex = triangleIndex / 32;
    // Bit index of L/R flag in DWORD
    int bitIndex = triangleIndex % 32;
    
    // Shift triangle L/R flags such that lastFlags only contains flag prior to triangleIndex
    uint lastFlags = IndexBufferCache[dwordIndex] << (31 - bitIndex);

    if (dwordIndex != 0 && bitIndex != 31)
    {
        // Fill remaining bits with previous L/R flag DWORD
        lastFlags |= IndexBufferCache[dwordIndex - 1] >> (bitIndex + 1);
    }
    
    return lastFlags;
}

// Calculate offset of current triangle in triangle fan
// Returns 0 if triangle is not in a triangle fan
uint LoadTriangleFanOffset(in int triangleIndex, in bool triangleFlag)
{
    uint lastFlags;
    int fanOffset = 1;
    
    do
    {
        // Load last 32 L/R-flags
        lastFlags = LoadLast32TriangleFlags(triangleIndex);
        // Flip L/R-flags, such that
        //  0 = same L/R flag
        //  1 = different L/R flags, i.e. end of triangle fan
        lastFlags = triangleFlag ? ~lastFlags : lastFlags;
        // 31 - firstbithigh counts leading 0 bits, i.e. length of triangle fan
        fanOffset += 31 - firstbithigh(lastFlags);
        // Move to next 32 L/R flags
        triangleIndex -= 32;
    }
    // Continue util
    // - lastFlags contains different L/R flag (end of triangle fan)
    // - start of triangle strip was reached (triangleIndex < 0)
    while ((lastFlags == 0) && (triangleIndex >= 0));
    
    return fanOffset;
}

bool LoadTriangleIncrementFlag(in int triangleIndex)
{
    // Index of DWORD which contains increment flag for triangleIndex
    const uint dwordIndex = 8 + (triangleIndex / 32);
    // Bit index of increment flag in DWORD
    const uint bitIndex = triangleIndex % 32;
    
    // Extract triangle increment bit
    return IndexBufferCache[dwordIndex] & (1U << bitIndex);
}

uint LoadTriangleIncrementPrefix(in int triangleIndex)
{
    // Index of DWORD which contains L/R flag for triangleIndex
    const uint dwordIndex = triangleIndex / 32;
    // Bit index of L/R flag in DWORD
    const uint bitIndex   = triangleIndex % 32;
    
    // Count active bits (i.e. increments) in all prior triangles
    const uint prefix = countbits(
        IndexBufferCache[TRIANGLE_INCREMENT_FLAG_OFFSET + dwordIndex] << (31 - bitIndex));
    
    if (dwordIndex >= 1)
    {
        // IncrementPrefixCache stores prefix sum of all prior DWORDs
        return prefix + IncrementPrefixCache[dwordIndex - 1];
    }
    else
    {
        return prefix;
    }
}

uint LoadTriangleIndex(in int triangleIndex)
{
    if (triangleIndex <= 0)
    {
        return max(triangleIndex + 2, 0);
    }
    else
    {
        const bool increment = LoadTriangleIncrementFlag(triangleIndex);
        const uint prefix = LoadTriangleIncrementPrefix(triangleIndex);
        
        if (increment)
        {
            return prefix + 2;
        }
        else
        {
            const uint index = triangleIndex - (prefix + 1);
            
            const uint dwordIndex = index / 4;
            const uint byteIndex = index % 4;

            const uint value = IndexBufferCache[TRIANGLE_INDEX_OFFSET + dwordIndex];

            // Extract 8 bit vertex index
            return (value >> (byteIndex * 8)) & 0xFF;
        }
    }
}

uint3 LoadTriangle(in int triangleIndex)
{
    // Triangle strip L/R flag for current triangle
    const bool triangleFlag = LoadTriangleFlag(triangleIndex);
    
    // Offset in triangle fan
    const uint triangleFanOffset = LoadTriangleFanOffset(triangleIndex, triangleFlag);
    
    // New vertex index
    uint i = LoadTriangleIndex(triangleIndex);
    // Previous vertex index
    uint p = LoadTriangleIndex(triangleIndex - 1);
    // Triangle fan offset vertex index
    uint o = LoadTriangleIndex(triangleIndex - triangleFanOffset);
    
    // Left triangle
    // i ----- p
    //  \  t  / \
    //   \   /   \
    //    \ / t-1 \
    //     o ----- *
    // 
    // Right triangle
    //     p ----- i
    //    / \  t  /
    //   /   \   /
    //  / t-1 \ /
    // *------ o
    
    return triangleFlag ?
        // Right triangle
        uint3(p, o, i) :
        // Left triangle
        uint3(o, p, i);
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

    // Load 8 DWORDs for L/R flags, 8 DWORDs for increments flags and all 8-bit indices
    const uint dwordsToLoad = TRIANGLE_LR_FLAG_SIZE + 
                              TRIANGLE_INCREMENT_FLAG_SIZE +
                              ((meshlet.reusedIndexCount + 3) / 4);
    
    if (threadId < dwordsToLoad)
    {
        IndexBufferCache[threadId] = Indices[(meshlet.primitiveOffset / 4) + threadId];
    }
    
    GroupMemoryBarrierWithGroupSync();

    // Initialize prefix sum cache
    if (threadId < 7) {
        const uint bits = countbits(IndexBufferCache[TRIANGLE_INCREMENT_FLAG_OFFSET + threadId]);
        
        IncrementPrefixCache[threadId] = bits + WavePrefixSum(bits);
    }

    GroupMemoryBarrierWithGroupSync();
    
    // Every thread writes two triangles
    for (uint i = 0; i < 2; ++i)
    {
        uint triangleIndex = threadId + i * 128;
        
        if (triangleIndex < meshlet.primitiveCount)
        {
            tris[triangleIndex] = LoadTriangle(triangleIndex);
        }
    }

    // Load vertices here. See Quantization.hlsl
    // ...
}