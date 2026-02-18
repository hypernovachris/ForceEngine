#ifndef PRIMITIVE_BUILDER_H
#define PRIMITIVE_BUILDER_H

#include "Mesh.h"
#include <vector>
#include <cmath>
#include <memory>

class PrimitiveBuilder {
public:
    static std::shared_ptr<Mesh> createSphere(int sectors, int stacks, float radius, bool hasNormals, bool hasUVs) {
        std::vector<float> vertices;
        std::vector<unsigned int> indices;

        const float PI = 3.14159265358979323846f;

        float sectorStep = 2 * PI / sectors;
        float stackStep = PI / stacks;
        float sectorAngle, stackAngle;

        // 1. Generate Vertices
        for (int i = 0; i <= stacks; ++i) {
            stackAngle = PI / 2 - i * stackStep;        // From pi/2 to -pi/2
            float xz = radius * cosf(stackAngle);       // r * cos(phi)
            float y = radius * sinf(stackAngle);        // r * sin(phi) (Up is Y)

            for (int j = 0; j <= sectors; ++j) {
                sectorAngle = j * sectorStep;           // From 0 to 2pi

                // Position
                float x = xz * cosf(sectorAngle);       // r * cos(phi) * cos(theta)
                float z = xz * sinf(sectorAngle);       // r * cos(phi) * sin(theta)
                vertices.push_back(x);
                vertices.push_back(y);
                vertices.push_back(z);

                // Normals
                if (hasNormals) {
                    float nx = x / radius;
                    float ny = y / radius;
                    float nz = z / radius;
                    vertices.push_back(nx);
                    vertices.push_back(ny);
                    vertices.push_back(nz);
                }

                // UVs
                if (hasUVs) {
                    float u = 1.0f - (float)j / sectors;
                    float v = 1.0f - (float)i / stacks;
                    vertices.push_back(u);
                    vertices.push_back(v);
                }

                // Tangents (Only calculate if we have both Normals and UVs)
                if (hasNormals && hasUVs) {
                    // Tangent points in the direction of increasing sectorAngle (U-axis)
                    // Since U is inverted (1 - ...), tangent is also inverted relative to sectorAngle
                    float tx = sinf(sectorAngle); 
                    float ty = 0.0f;
                    float tz = -cosf(sectorAngle);
                    
                    vertices.push_back(tx);
                    vertices.push_back(ty);
                    vertices.push_back(tz);
                }
            }
        }

        // 2. Generate Indices (Connect the dots into triangles)
        int k1, k2;
        for (int i = 0; i < stacks; ++i) {
            k1 = i * (sectors + 1);     // Beginning of current stack
            k2 = k1 + sectors + 1;      // Beginning of next stack

            for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
                // 2 triangles per sector, excluding the very top and bottom poles
                if (i != 0) {
                    indices.push_back(k1);
                    indices.push_back(k2);
                    indices.push_back(k1 + 1);
                }
                if (i != (stacks - 1)) {
                    indices.push_back(k1 + 1);
                    indices.push_back(k2);
                    indices.push_back(k2 + 1);
                }
            }
        }

        return std::make_shared<Mesh>(vertices, indices, hasNormals, hasUVs);
    }
    
    static std::shared_ptr<Mesh> createCube(float sizeX, float sizeY, float sizeZ, bool hasNormals, bool hasUVs) {
        std::vector<float> vertices;
        std::vector<unsigned int> indices;

        // Calculate half-extents so the cube is centered at (0,0,0)
        float w = sizeX / 2.0f;
        float h = sizeY / 2.0f;
        float d = sizeZ / 2.0f;

        // The 24 vertices of a cube (4 per face)
        // Format: px, py, pz,   nx, ny, nz,   u, v,   tx, ty, tz
        float rawData[] = {
            // Front face (Z = +d) | Tangent = +X
            -w, -h,  d,   0.0f,  0.0f,  1.0f,   0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w, -h,  d,   0.0f,  0.0f,  1.0f,   1.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w,  h,  d,   0.0f,  0.0f,  1.0f,   1.0f, 1.0f,   1.0f, 0.0f, 0.0f,
            -w,  h,  d,   0.0f,  0.0f,  1.0f,   0.0f, 1.0f,   1.0f, 0.0f, 0.0f,
            // Back face (Z = -d) | Tangent = -X
             w, -h, -d,   0.0f,  0.0f, -1.0f,   0.0f, 0.0f,  -1.0f, 0.0f, 0.0f,
            -w, -h, -d,   0.0f,  0.0f, -1.0f,   1.0f, 0.0f,  -1.0f, 0.0f, 0.0f,
            -w,  h, -d,   0.0f,  0.0f, -1.0f,   1.0f, 1.0f,  -1.0f, 0.0f, 0.0f,
             w,  h, -d,   0.0f,  0.0f, -1.0f,   0.0f, 1.0f,  -1.0f, 0.0f, 0.0f,
            // Left face (X = -w) | Tangent = +Z
            -w, -h, -d,  -1.0f,  0.0f,  0.0f,   0.0f, 0.0f,   0.0f, 0.0f, 1.0f,
            -w, -h,  d,  -1.0f,  0.0f,  0.0f,   1.0f, 0.0f,   0.0f, 0.0f, 1.0f,
            -w,  h,  d,  -1.0f,  0.0f,  0.0f,   1.0f, 1.0f,   0.0f, 0.0f, 1.0f,
            -w,  h, -d,  -1.0f,  0.0f,  0.0f,   0.0f, 1.0f,   0.0f, 0.0f, 1.0f,
            // Right face (X = w) | Tangent = -Z
             w, -h,  d,   1.0f,  0.0f,  0.0f,   0.0f, 0.0f,   0.0f, 0.0f, -1.0f,
             w, -h, -d,   1.0f,  0.0f,  0.0f,   1.0f, 0.0f,   0.0f, 0.0f, -1.0f,
             w,  h, -d,   1.0f,  0.0f,  0.0f,   1.0f, 1.0f,   0.0f, 0.0f, -1.0f,
             w,  h,  d,   1.0f,  0.0f,  0.0f,   0.0f, 1.0f,   0.0f, 0.0f, -1.0f,
            // Top face (Y = h) | Tangent = +X
            -w,  h,  d,   0.0f,  1.0f,  0.0f,   0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w,  h,  d,   0.0f,  1.0f,  0.0f,   1.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w,  h, -d,   0.0f,  1.0f,  0.0f,   1.0f, 1.0f,   1.0f, 0.0f, 0.0f,
            -w,  h, -d,   0.0f,  1.0f,  0.0f,   0.0f, 1.0f,   1.0f, 0.0f, 0.0f,
            // Bottom face (Y = -h) | Tangent = +X
            -w, -h, -d,   0.0f, -1.0f,  0.0f,   0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w, -h, -d,   0.0f, -1.0f,  0.0f,   1.0f, 0.0f,   1.0f, 0.0f, 0.0f,
             w, -h,  d,   0.0f, -1.0f,  0.0f,   1.0f, 1.0f,   1.0f, 0.0f, 0.0f,
            -w, -h,  d,   0.0f, -1.0f,  0.0f,   0.0f, 1.0f,   1.0f, 0.0f, 0.0f
        };

        bool hasTangents = (hasNormals && hasUVs);

        // 1. Dynamically build the Vertex Array in ONE interleaved loop
        for (int i = 0; i < 24; ++i) {
            int strideIndex = i * 11; // 11 floats per raw vertex
            
            // Always push Position (x, y, z)
            vertices.push_back(rawData[strideIndex]);
            vertices.push_back(rawData[strideIndex + 1]);
            vertices.push_back(rawData[strideIndex + 2]);

            // Push Normals if requested (nx, ny, nz)
            if (hasNormals) {
                vertices.push_back(rawData[strideIndex + 3]);
                vertices.push_back(rawData[strideIndex + 4]);
                vertices.push_back(rawData[strideIndex + 5]);
            }

            // Push UVs if requested (u, v)
            if (hasUVs) {
                vertices.push_back(rawData[strideIndex + 6]);
                vertices.push_back(rawData[strideIndex + 7]);
            }

            // Push Tangents if requested (tx, ty, tz)
            if (hasTangents) {
                vertices.push_back(rawData[strideIndex + 8]);
                vertices.push_back(rawData[strideIndex + 9]);
                vertices.push_back(rawData[strideIndex + 10]);
            }
        }

        // 2. Generate Indices
        // We have 6 faces, each with 4 vertices.
        // We need 2 triangles per face.
        for (int i = 0; i < 6; ++i) {
            unsigned int offset = i * 4;
            
            // Triangle 1
            indices.push_back(offset + 0);
            indices.push_back(offset + 1);
            indices.push_back(offset + 2);

            // Triangle 2
            indices.push_back(offset + 2);
            indices.push_back(offset + 3);
            indices.push_back(offset + 0);
        }

        return std::make_shared<Mesh>(vertices, indices, hasNormals, hasUVs);
    }
};

#endif