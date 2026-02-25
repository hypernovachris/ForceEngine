#ifndef RENDERER_COMPONENT_H
#define RENDERER_COMPONENT_H

#include "Component.h"
#include "Model.h"
#include "ResourceManager.h"
#include <memory>

class RendererComponent : public Component {
public:
    std::shared_ptr<Model> model;

    RendererComponent(std::shared_ptr<Model> mod) 
        : model(mod) {}

    // The factory function required by the ComponentRegistry
    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        std::string modelName;
        
        // Read the model name from the text stream (e.g., "david" or "sci_fi_crate")
        if (iss >> modelName) {
            // Ask the ResourceManager for the pre-loaded model
            auto loadedModel = ResourceManager::getModel(modelName);
            
            if (!loadedModel) {
                std::cout << "Warning: RendererComponent in scene file requested unknown model: '" << modelName << "'" << std::endl;
            }
            
            // Return the newly constructed component
            return std::make_shared<RendererComponent>(loadedModel);
        }

        std::cout << "Error: RendererComponent in scene file is missing a model name." << std::endl;
        return nullptr;
    }
};

#endif