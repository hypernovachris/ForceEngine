#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include "Entity.h"
#include "ResourceManager.h"
#include "ComponentRegistry.h"
// Include all your components here...
#include "RendererComponent.h"
#include "PhysicsComponent.h"
#include "ColliderComponent.h"
#include "FlapControllerComponent.h"
#include "CameraComponent.h"
#include "LightComponent.h"

#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <iostream>

class SceneLoader {
public:
    // We pass the rootEntity so the loader knows where to put the base objects
    // We pass the GLFWwindow so the FlapController can be initialized
    static void loadScene(const std::string& filepath, std::shared_ptr<Entity> rootEntity, GLFWwindow* window) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Failed to open scene file: " << filepath << std::endl;
            return;
        }

        std::string line;
        std::map<std::string, std::shared_ptr<Entity>> entityMap;
        std::shared_ptr<Entity> currentEntity = nullptr;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string tag;
            iss >> tag;

            if (tag == "ENTITY") {
                std::string entityName;
                iss >> entityName;
                
                currentEntity = std::make_shared<Entity>();
                currentEntity->name = entityName;
                entityMap[entityName] = currentEntity;
                
                // By default, attach it to the root. If it has a PARENT tag later, we will move it.
                rootEntity->addChild(currentEntity); 
            }
            else if (tag == "PARENT" && currentEntity) {
                std::string parentName;
                iss >> parentName;
                
                if (entityMap.find(parentName) != entityMap.end()) {
                    // Remove from root and attach to the specified parent
                    rootEntity->removeChild(currentEntity); // (You'll need to implement removeChild in Entity.h)
                    entityMap[parentName]->addChild(currentEntity);
                } else {
                    std::cerr << "Error: Parent '" << parentName << "' not found before child!" << std::endl;
                }
            }
            // --- TRANSFORM PARSING ---
            else if (tag == "POSITION" && currentEntity) {
                iss >> currentEntity->position.x >> currentEntity->position.y >> currentEntity->position.z;
            }
            else if (tag == "ROTATION" && currentEntity) {
                iss >> currentEntity->rotation.x >> currentEntity->rotation.y >> currentEntity->rotation.z;
            }
            else if (tag == "SCALE" && currentEntity) {
                iss >> currentEntity->scale.x >> currentEntity->scale.y >> currentEntity->scale.z;
            }
            // --- COMPONENT FACTORY ---
            else if (tag == "COMPONENT" && currentEntity) {
                std::string compType;
                iss >> compType;

                // Ask the registry to build it. We don't need to know what it is!
                auto newComponent = ComponentRegistry::create(compType, iss, window);
                
                if (newComponent) {
                    currentEntity->addComponent(newComponent);
                } else {
                    std::cout << "Warning: Unknown component type in scene file: " << compType << std::endl;
                }
            }
        }
    }
};

#endif