#ifndef MATERIAL_H
#define MATERIAL_H

#include "Shader.h"
#include "Texture.h"
#include <glm/glm/glm.hpp>
#include <memory>

// This struct matches the MSL FragmentUniforms expectations for material properties
// We align to 16 bytes for GPU safety
struct MaterialUniformData {
    float shininess = 32.0f;
    float hasDiffuse = 0.0f;
    float hasSpecular = 0.0f;
    float hasNormalMap = 0.0f;
};

class Material {
public:
    std::shared_ptr<Shader> shader;
    
    // Texture Maps
    std::shared_ptr<Texture> diffuseMap;
    std::shared_ptr<Texture> specularMap;
    std::shared_ptr<Texture> normalMap;

    MaterialUniformData materialData;
    glm::vec2 textureScale = glm::vec2(1.0f);

    Material(std::shared_ptr<Shader> s) 
        : shader(s) {}

    // In the new command-driven RHI architecture, we don't bind immediately.
    // Instead, the Renderer will extract the maps and `materialData` when recording commands.
    void updateData() {
        materialData.hasDiffuse = diffuseMap != nullptr ? 1.0f : 0.0f;
        materialData.hasSpecular = specularMap != nullptr ? 1.0f : 0.0f;
        materialData.hasNormalMap = normalMap != nullptr ? 1.0f : 0.0f;
    }
};

#endif