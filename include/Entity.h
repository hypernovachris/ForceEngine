#ifndef ENTITY_H
#define ENTITY_H

#include <vector>
#include <memory>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "Component.h" // <-- NEW: We need to know what a Component is

class Entity {
public:

    // Transform data (local space)
    glm::vec3 position;
    glm::vec3 rotation; // Euler angles (pitch, yaw, roll)
    glm::vec3 scale;

    // The cached matrices
    glm::mat4 localTransform;
    glm::mat4 worldTransform;

    // Scene graph hierarchy
    Entity* parent;
    std::vector<std::shared_ptr<Entity>> children;

    // --- NEW: The Component List ---
    // This is the "backpack" that holds this entity's behaviors (Renderer, Physics, etc.)
    std::vector<std::shared_ptr<Component>> components;

    Entity() : position(0.0f), rotation(0.0f), scale(1.0f),
        localTransform(1.0f), worldTransform(1.0f), parent(nullptr) {}

    virtual ~Entity() {}

    // --- Graph methods ---
    void addChild(std::shared_ptr<Entity> child) {
        child->parent = this;
        children.push_back(child);
    }

    // --- NEW: Component Management ---
    // Adds a component and tells the component that THIS entity owns it
    template <typename T>
    void addComponent(std::shared_ptr<T> component) {
        component->owner = this;
        components.push_back(component);
        component->awake(); // Run any setup code the component has
    }

    // Searches the backpack to see if the entity has a specific type of component
    template <typename T>
    std::shared_ptr<T> getComponent() {
        for (auto& comp : components) {
            std::shared_ptr<T> target = std::dynamic_pointer_cast<T>(comp);
            if (target) return target;
        }
        return nullptr; // Returns null if not found
    }

    // --- Matrix Math ---
    void updateSelfAndChild() {
        // 1. Calculate this entity's Local Transform
        localTransform = glm::mat4(1.0f);
        localTransform = glm::translate(localTransform, position);
        localTransform = glm::rotate(localTransform, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        localTransform = glm::rotate(localTransform, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        localTransform = glm::rotate(localTransform, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
        localTransform = glm::scale(localTransform, scale);

        // 2. Calculate World Transform (Parent's World * My Local)
        if (parent) {
            worldTransform = parent->worldTransform * localTransform;
        } else {
            worldTransform = localTransform; // If no parent, local is world
        }

        // 3. Recursively update all children
        for (auto& child : children) {
            child->updateSelfAndChild();
        }
    }

    // --- REPLACED: The Engine Loop ---
    // Instead of relying on a subclass to define what to do, 
    // the Entity just tells all its components to do their jobs.
    void update(float deltaTime) {
        for (auto& comp : components) {
            comp->update(deltaTime);
        }
    }
};

#endif