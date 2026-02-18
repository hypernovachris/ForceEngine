#ifndef MATERIAL_H
#define MATERIAL_H

#include "Shader.h"
#include "Texture.h"
#include <glm/glm/glm.hpp>

#include <memory>

class Material {
public:
    std::shared_ptr<Shader> shader;
    
    // Texture Maps
    std::shared_ptr<Texture> diffuseMap;
    std::shared_ptr<Texture> specularMap;
    std::shared_ptr<Texture> normalMap;

    // Material Properties
    float shininess; // How tight the reflection reflection is (e.g., 32.0f or 64.0f)
    glm::vec2 textureScale; 

    Material(std::shared_ptr<Shader> s) 
        : shader(s), diffuseMap(nullptr), specularMap(nullptr), normalMap(nullptr), shininess(32.0f), textureScale(1.0f) {}

    void apply() {
        shader->use();
        
        // We must bind different textures to different "Texture Units" in OpenGL hardware
        if (diffuseMap) {
            shader->setFloat("hasDiffuse", 1.0f);
            diffuseMap->bind(0); // Bind to GL_TEXTURE0
            shader->setInt("material.diffuse", 0);
        } else {
            shader->setFloat("hasDiffuse", 0.0f);
        }

        if (specularMap) {
            shader->setFloat("hasSpecular", 1.0f);
            specularMap->bind(1); // Bind to GL_TEXTURE1
            shader->setInt("material.specular", 1);
        } else {
            shader->setFloat("hasSpecular", 0.0f);
        }

        if (normalMap) {
            shader->setFloat("hasNormalMap", 1.0f);
            normalMap->bind(2); // Bind to GL_TEXTURE2
            shader->setInt("material.normal", 2);
        } else {
            shader->setFloat("hasNormalMap", 0.0f);
        }

        shader->setFloat("material.shininess", shininess);
        shader->setVec2("textureScale", textureScale);
    }
};

#endif