#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include <map>
#include <string>
#include <memory>
#include <iostream>

#include "Texture.h"
#include "Shader.h"
#include "Mesh.h"
#include "Model.h"

class ResourceManager {
public:
    static std::shared_ptr<Shader> loadShader(const char* vShaderFile, const char* fShaderFile, std::string name) {
        Shaders[name] = std::make_shared<Shader>(vShaderFile, fShaderFile);
        return Shaders[name];
    }

    static std::shared_ptr<Shader> getShader(std::string name) {
        return Shaders[name];
    }

    static std::shared_ptr<Texture> loadTexture(const char* file, std::string name) {
        Textures[name] = std::make_shared<Texture>(file);
        return Textures[name];
    }

    static std::shared_ptr<Texture> getTexture(std::string name) {
        return Textures[name];
    }

    static std::shared_ptr<Model> loadModel(const std::string& path, const std::string& name) {
        // Prevent loading the same model twice
        if (Models.find(name) != Models.end()) {
            return Models[name];
        }
        
        std::shared_ptr<Model> model = std::make_shared<Model>(path);
        Models[name] = model;
        return model;
    }
    
    static std::shared_ptr<Model> getModel(std::string name) {
        return Models[name];
    }

    static std::shared_ptr<Material> createMaterial(const char* vShaderFile, const char* fShaderFile) {
        // Simple non-cached material creation for now (or cache by shader path?)
        std::string name = std::string(vShaderFile) + std::string(fShaderFile);
        auto shader = loadShader(vShaderFile, fShaderFile, name);
        return std::make_shared<Material>(shader);
    }
    
    // Clear all resources (optional, as shared_ptr handles cleanup)
    static void clear() {
        Shaders.clear();
        Textures.clear();
        Models.clear();
    }



private:
    static std::map<std::string, std::shared_ptr<Shader>> Shaders;
    static std::map<std::string, std::shared_ptr<Texture>> Textures;
    static std::map<std::string, std::shared_ptr<Model>> Models;

    ResourceManager() {}
};



#endif
