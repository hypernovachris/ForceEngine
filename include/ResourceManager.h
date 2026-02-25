#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include <map>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

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
    
    static std::vector<std::shared_ptr<Mesh>> loadRawMeshes(const std::string& path) {
        Model rawModel(path);
        return rawModel.meshes;
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
    
    static std::shared_ptr<Material> loadForceMaterial(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Error opening material file: " << filepath << std::endl;
            return nullptr;
        }
        std::string line;
        std::shared_ptr<Shader> shader = nullptr;
        std::shared_ptr<Texture> diffuse = nullptr;
        std::shared_ptr<Texture> specular = nullptr;
        std::shared_ptr<Texture> normal = nullptr;
        float shininess = 2.0f;
        glm::vec2 textureScale = glm::vec2(1.0f, 1.0f);
    
        while (std::getline(file, line)) {
            if(line.empty() || line[0] == '#') continue;
    
            std::istringstream iss(line);
            std::string tag;
            iss >> tag;
            
            if (tag == "SHADER") {
                std::string vsPath, fsPath;
                iss >> vsPath >> fsPath;
                shader = loadShader(vsPath.c_str(), fsPath.c_str(), vsPath); // Assuming path as name for simplicity
            } 
            else if (tag == "DIFFUSE") {
                std::string texPath;
                iss >> texPath;
                diffuse = loadTexture(texPath.c_str(), texPath);
            }
            else if (tag == "SPECULAR") {
                std::string texPath;
                iss >> texPath;
                specular = loadTexture(texPath.c_str(), texPath);
            }
            else if (tag == "NORMAL") {
                std::string texPath;
                iss >> texPath;
                normal = loadTexture(texPath.c_str(), texPath);
            }
            else if (tag == "SHININESS") {
                iss >> shininess;
            }
            else if (tag == "TEXTURE_SCALE") {
                iss >> textureScale.x >> textureScale.y;
            }
        }
    
        // Assemble and return the material
        auto material = std::make_shared<Material>(shader);
        material->diffuseMap = diffuse; // Note: Assumes your Material class takes raw pointers or adapt to shared_ptr
        material->specularMap = specular;
        material->normalMap = normal;
        material->shininess = shininess;
        material->textureScale = textureScale;
    
        return material;
    }
    
    // Replaces the old loadForceModel function
    static void parseForceModelFile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cout << "Failed to open model file: " << filepath << std::endl;
            return;
        }

        std::string line;
        std::shared_ptr<Model> currentModel = nullptr;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string tag;
            iss >> tag;

            if (tag == "MODEL") {
                std::string modelName;
                iss >> modelName;
                
                // Create a new empty model and register it in the central map
                currentModel = std::make_shared<Model>();
                Models[modelName] = currentModel;
            }
            else if (tag == "MESH" && currentModel != nullptr) {
                std::string objPath;
                iss >> objPath;
                
                // Assimp extracts the raw geometry (could be 1 mesh, could be 5)
                std::vector<std::shared_ptr<Mesh>> rawMeshes = loadRawMeshes(objPath); 
                
                for (auto& mesh : rawMeshes) {
                    currentModel->meshes.push_back(mesh);
                    // Push a null material as a placeholder so our parallel lists stay synced
                    currentModel->materials.push_back(nullptr); 
                }
            }
            else if (tag == "MATERIAL" && currentModel != nullptr) {
                std::string matPath;
                iss >> matPath;
                
                auto material = loadForceMaterial(matPath); 
                
                // Apply this material to all recently added meshes that don't have one yet
                for (int i = currentModel->materials.size() - 1; i >= 0; --i) {
                    if (currentModel->materials[i] == nullptr) {
                        currentModel->materials[i] = material;
                    } else {
                        // We hit a mesh that already has a material paired to it, so stop searching backwards
                        break; 
                    }
                }
            }
        }
    }

    private:
    static std::map<std::string, std::shared_ptr<Shader>> Shaders;
    static std::map<std::string, std::shared_ptr<Texture>> Textures;
    static std::map<std::string, std::shared_ptr<Model>> Models;

    ResourceManager() {}
};



#endif
