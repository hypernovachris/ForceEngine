#ifndef LINEAR_MOVEMENT_COMPONENT_H
#define LINEAR_MOVEMENT_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include <glm/glm/glm.hpp>
#include <GLFW/glfw3.h>

class LinearMovementComponent : public Component {
public:
    glm::vec3 velocity;

    // Pass in the speed and direction. 
    // For Flappy Bird pipes, this will be something like vec3(-5.0f, 0.0f, 0.0f)
    LinearMovementComponent(glm::vec3 vel) : velocity(vel) {}

    void update(float deltaTime) override {
        if (!owner) return;

        // Continuously push the entity in the designated direction
        owner->position += velocity * deltaTime;
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        float x, y, z;
        if (iss >> x >> y >> z) {
            return std::make_shared<LinearMovementComponent>(glm::vec3(x, y, z));
        }
        return nullptr;
    }
};

#endif