#ifndef PRIMITIVE_BUILDER_H
#define PRIMITIVE_BUILDER_H

#include "Mesh.h"
#include "Model.h"
#include <vector>
#include <cmath>
#include <memory>
#include <map>
#include <cstdint>
#include <glm/glm/glm.hpp>

class PrimitiveBuilder {
public:
    static std::shared_ptr<Model> createSphere(int sectors, int stacks, float radius, bool hasNormals, bool hasUVs) {
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

        auto mesh = std::make_shared<Mesh>(vertices, indices, hasNormals, hasUVs);
        auto model = std::make_shared<Model>();
        model->meshes.push_back(mesh);
        model->materials.push_back(nullptr);
        return model;
    }
    
    static std::shared_ptr<Model> createCube(float sizeX, float sizeY, float sizeZ, bool hasNormals, bool hasUVs) {
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

        auto mesh = std::make_shared<Mesh>(vertices, indices, hasNormals, hasUVs);
        auto model = std::make_shared<Model>();
        model->meshes.push_back(mesh);
        model->materials.push_back(nullptr);
        return model;
    }

    static std::shared_ptr<Model> createIcosphere(int subdivisions, float radius, bool hasNormals, bool hasUVs) {
        std::vector<float> vertices;
        std::vector<unsigned int> indices;

        const float X = .525731112119133606f;
        const float Z = .850650808352039932f;
        const float N = 0.f;

        std::vector<glm::vec3> initialVertices = {
            {-X,N,Z}, {X,N,Z}, {-X,N,-Z}, {X,N,-Z},
            {0.f,Z,X}, {0.f,Z,-X}, {0.f,-Z,X}, {0.f,-Z,-X},
            {Z,X,N}, {-Z,X,N}, {Z,-X,N}, {-Z,-X,N}
        };

        std::vector<unsigned int> initialIndices = {
            0,4,1, 0,9,4, 9,5,4, 4,5,8, 4,8,1,
            8,10,1, 8,3,10, 5,3,8, 5,2,3, 2,7,3,
            7,10,3, 7,6,10, 7,11,6, 11,0,6, 0,1,6,
            6,1,10, 9,0,11, 9,11,2, 9,2,5, 7,2,11
        };

        // Helper to find midpoint
        auto getMiddlePoint = [&](unsigned int p1, unsigned int p2, std::map<int64_t, unsigned int>& cache) -> unsigned int {
            bool firstIsSmaller = p1 < p2;
            int64_t smallerIndex = firstIsSmaller ? p1 : p2;
            int64_t greaterIndex = firstIsSmaller ? p2 : p1;
            int64_t key = (smallerIndex << 32) + greaterIndex;

            if (cache.find(key) != cache.end()) {
                return cache[key];
            }

            glm::vec3 point1 = initialVertices[p1];
            glm::vec3 point2 = initialVertices[p2];
            glm::vec3 middle = glm::normalize(point1 + point2);

            initialVertices.push_back(middle);

            unsigned int index = static_cast<unsigned int>(initialVertices.size() - 1);
            cache[key] = index;
            return index;
        };

        // Refine triangles
        std::map<int64_t, unsigned int> middlePointIndexCache;
        for (int i = 0; i < subdivisions; ++i) {
            std::vector<unsigned int> newIndices;
            for (size_t j = 0; j < initialIndices.size(); j += 3) {
                unsigned int v1 = initialIndices[j];
                unsigned int v2 = initialIndices[j + 1];
                unsigned int v3 = initialIndices[j + 2];

                unsigned int a = getMiddlePoint(v1, v2, middlePointIndexCache);
                unsigned int b = getMiddlePoint(v2, v3, middlePointIndexCache);
                unsigned int c = getMiddlePoint(v3, v1, middlePointIndexCache);

                newIndices.push_back(v1); newIndices.push_back(a); newIndices.push_back(c);
                newIndices.push_back(v2); newIndices.push_back(b); newIndices.push_back(a);
                newIndices.push_back(v3); newIndices.push_back(c); newIndices.push_back(b);
                newIndices.push_back(a); newIndices.push_back(b); newIndices.push_back(c);
            }
            initialIndices = newIndices;
        }

        const float PI = 3.14159265359f;

        // "Weld" vertices helper: Struct to map vertex attributes to an index
        struct VertexSignature {
            float x, y, z;
            float u, v;
            // Equality operator for map key
            bool operator<(const VertexSignature& other) const {
                if (x != other.x) return x < other.x;
                if (y != other.y) return y < other.y;
                if (z != other.z) return z < other.z;
                if (u != other.u) return u < other.u;
                return v < other.v;
            }
        };
        std::map<VertexSignature, unsigned int> uniqueVertices;
        
        auto addVertex = [&](glm::vec3 p, glm::vec2 uv) -> unsigned int {
            VertexSignature sig = { p.x, p.y, p.z, uv.x, uv.y };
            if (uniqueVertices.find(sig) != uniqueVertices.end()) {
                return uniqueVertices[sig];
            }
            
            unsigned int idx = static_cast<unsigned int>(uniqueVertices.size());
            uniqueVertices[sig] = idx;

            // Add straight to flattened vertex buffer
            // Position
            glm::vec3 finalPos = p * radius;
            vertices.push_back(finalPos.x);
            vertices.push_back(finalPos.y);
            vertices.push_back(finalPos.z);
            
            // Normal
            if (hasNormals) {
                vertices.push_back(p.x);
                vertices.push_back(p.y);
                vertices.push_back(p.z);
            }

            // UV
            if (hasUVs) {
                vertices.push_back(uv.x);
                vertices.push_back(uv.y);
            }

            // Tangent
            if (hasNormals && hasUVs) {
                glm::vec3 tangent;
                tangent.x = -p.z;
                tangent.y = 0.0f;
                tangent.z = p.x;
                if (glm::length(tangent) > 0.0001f) {
                     tangent = glm::normalize(tangent);
                } else {
                     tangent = glm::vec3(1.0f, 0.0f, 0.0f);
                }
                vertices.push_back(tangent.x);
                vertices.push_back(tangent.y);
                vertices.push_back(tangent.z);
            }

            return idx;
        };

        // Process triangles to fix UV seams
        for (size_t i = 0; i < initialIndices.size(); i += 3) {
            glm::vec3 p0 = initialVertices[initialIndices[i]];
            glm::vec3 p1 = initialVertices[initialIndices[i+1]];
            glm::vec3 p2 = initialVertices[initialIndices[i+2]];

            auto getUV = [&](glm::vec3 p) -> glm::vec2 {
                float u = 0.5f + (atan2f(p.z, p.x) / (2.0f * PI));
                float v = 0.5f - (asinf(p.y) / PI);
                return glm::vec2(u, v);
            };

            glm::vec2 uv0 = getUV(p0);
            glm::vec2 uv1 = getUV(p1);
            glm::vec2 uv2 = getUV(p2);

            // Fix seam
            // If the triangle spans the texture edge (one U ~ 0, one U ~ 1), 
            // shift the small ones up by 1.0 or the large ones down.
            // But usually we just shift the small ones up to keep them continuous 
            // if we consider 0.9 -> 1.1 wrapping.
            
            // Check delta U
            const float threshold = 0.75f; // If distance > 0.75, it wrapped
            
            if (uv1.x - uv0.x > threshold)  uv0.x += 1.0f;
            if (uv0.x - uv1.x > threshold)  uv1.x += 1.0f;
            
            if (uv2.x - uv0.x > threshold)  uv0.x += 1.0f;
            if (uv0.x - uv2.x > threshold)  uv2.x += 1.0f;
            
            if (uv2.x - uv1.x > threshold)  uv1.x += 1.0f;
            if (uv1.x - uv2.x > threshold)  uv2.x += 1.0f;

            // Fix poles
            // If a vertex is a pole (y ~ 1 or -1), its U is undefined/arbitrary.
            // We should match the U of the other vertices in the triangle.
            // (p0 is pole?)
            const float poleY = 0.999f;
            if (abs(p0.y) > poleY) uv0.x = (uv1.x + uv2.x) * 0.5f;
            if (abs(p1.y) > poleY) uv1.x = (uv0.x + uv2.x) * 0.5f;
            if (abs(p2.y) > poleY) uv2.x = (uv0.x + uv1.x) * 0.5f;

            unsigned int i0 = addVertex(p0, uv0);
            unsigned int i1 = addVertex(p1, uv1);
            unsigned int i2 = addVertex(p2, uv2);

            indices.push_back(i0);
            indices.push_back(i1);
            indices.push_back(i2);
        }

        auto mesh = std::make_shared<Mesh>(vertices, indices, hasNormals, hasUVs);
        auto model = std::make_shared<Model>();
        model->meshes.push_back(mesh);
        model->materials.push_back(nullptr);
        return model;
    }
};

#endif