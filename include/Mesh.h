#ifndef MESH_H
#define MESH_H

#include "RHI/RHI_Buffer.h"
#include <vector>

struct Vertex {
    float position[3];
    float normal[3];
    float texCoords[2];
    float tangent[3];
};

class Mesh {
public:
    RHI_Buffer vertexBuffer;
    RHI_Buffer indexBuffer;
    Uint32 vertexCount;
    Uint32 indexCount;

    Mesh(const std::vector<float>& vertices, const std::vector<unsigned int>& indices, bool hasNormals, bool hasUVs) {
        indexCount = static_cast<Uint32>(indices.size());
        
        // Re-pack generic float vector into Vertex structs for simpler RHI binding
        // Assuming the input vertices were tightly packed according to the old stride
        std::vector<Vertex> packedVertices;
        int stride = 3; 
        if (hasNormals) stride += 3;
        if (hasUVs) stride += 2;
        bool hasTangents = (hasNormals && hasUVs); 
        if (hasTangents) stride += 3;

        vertexCount = static_cast<Uint32>(vertices.size() / stride);
        packedVertices.reserve(vertexCount);
        
        for (size_t i = 0; i < vertexCount; ++i) {
            size_t base = i * stride;
            Vertex v = {};
            
            v.position[0] = vertices[base + 0];
            v.position[1] = vertices[base + 1];
            v.position[2] = vertices[base + 2];
            
            int offset = 3;
            if (hasNormals) {
                v.normal[0] = vertices[base + offset + 0];
                v.normal[1] = vertices[base + offset + 1];
                v.normal[2] = vertices[base + offset + 2];
                offset += 3;
            }
            if (hasUVs) {
                v.texCoords[0] = vertices[base + offset + 0];
                v.texCoords[1] = vertices[base + offset + 1];
                offset += 2;
            }
            if (hasTangents) {
                v.tangent[0] = vertices[base + offset + 0];
                v.tangent[1] = vertices[base + offset + 1];
                v.tangent[2] = vertices[base + offset + 2];
            } else {
                v.tangent[0] = 1.0f; v.tangent[1] = 0.0f; v.tangent[2] = 0.0f;
            }
            packedVertices.push_back(v);
        }

        // Create & Upload Vertex Buffer
        Uint32 vSize = vertexCount * sizeof(Vertex);
        vertexBuffer.create(SDL_GPU_BUFFERUSAGE_VERTEX, vSize);
        vertexBuffer.uploadData(packedVertices.data(), vSize);

        // Create & Upload Index Buffer
        Uint32 iSize = indexCount * sizeof(Uint32);
        indexBuffer.create(SDL_GPU_BUFFERUSAGE_INDEX, iSize);
        indexBuffer.uploadData(indices.data(), iSize);
    }
    
    // The renderer will now handle drawing by binding these buffers to a command buffer
};

#endif