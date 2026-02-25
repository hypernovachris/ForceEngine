#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>

class Mesh {
public:
    unsigned int VAO, VBO, EBO;
    int indexCount;

    Mesh(const std::vector<float>& vertices, const std::vector<unsigned int>& indices, bool hasNormals, bool hasUVs) {
        indexCount = static_cast<int>(indices.size());

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO); // The new Index Buffer

        glBindVertexArray(VAO);

        // Load Vertex Data
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

        // Load Index Data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        // Calculate dynamic stride
        int stride = 3; // Position (x, y, z)
        if (hasNormals) stride += 3;
        if (hasUVs) stride += 2;
        
        // We will assume that if you request both Normals AND UVs, 
        // you want Tangents for Normal Mapping.
        bool hasTangents = (hasNormals && hasUVs); 
        if (hasTangents) stride += 3; 
        
        int strideBytes = stride * sizeof(float);

        // 1. Position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, strideBytes, (void*)0);
        glEnableVertexAttribArray(0);

        int offset = 3;
        // 2. Normal 
        if (hasNormals) {
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, strideBytes, (void*)(offset * sizeof(float)));
            glEnableVertexAttribArray(1);
            offset += 3;
        }

        // 3. UV 
        if (hasUVs) {
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, strideBytes, (void*)(offset * sizeof(float)));
            glEnableVertexAttribArray(2);
            offset += 2;
        } else {
            glDisableVertexAttribArray(2);
            glVertexAttrib2f(2, 0.0f, 0.0f); // Default UV
        }
        
        // 4. Tangent
        if (hasTangents) {
            glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, strideBytes, (void*)(offset * sizeof(float)));
            glEnableVertexAttribArray(3);
        } else {
            glDisableVertexAttribArray(3);
            glVertexAttrib3f(3, 1.0f, 0.0f, 0.0f); // Default Tangent (Tangent X-axis)
        }

        // Unbind VAO
        glBindVertexArray(0);
    }

    void draw() {
        glBindVertexArray(VAO);
        // We now use glDrawElements instead of glDrawArrays
        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0); 
        glBindVertexArray(0);
    }
};

#endif