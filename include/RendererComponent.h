#ifndef RENDERER_COMPONENT_H
#define RENDERER_COMPONENT_H

#include "Component.h"
#include "Model.h"
#include <memory>

class RendererComponent : public Component {
public:
    std::shared_ptr<Model> model;
    std::shared_ptr<Material> material; // Optional override

    RendererComponent(std::shared_ptr<Model> mod, std::shared_ptr<Material> mat = nullptr) 
        : model(mod), material(mat) {}
};

#endif