#ifndef PHYSICS_COMPONENT_H
#define PHYSICS_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include <glm/glm/glm.hpp>
#include <GLFW/glfw3.h>

class PhysicsComponent : public Component {
public:
    glm::vec3 velocity{0.0f};
    float gravity = -16.0f; 
    
    // Optional: Flappy Bird usually has a terminal velocity so the bird doesn't fall through the floor in one frame
    float terminalVelocity = -20.0f; 

    void update(float deltaTime) override {
        if (!owner) return;

        // 1. Apply gravity to the velocity (Acceleration)
        velocity.y += gravity * deltaTime;

        // Clamp to terminal velocity
        if (velocity.y < terminalVelocity) {
            velocity.y = terminalVelocity;
        }

        // 2. Apply velocity to the Entity's position
        owner->position += velocity * deltaTime;

        // Tilt the bird based on velocity
        float tiltAngle = velocity.y * -45.0f / terminalVelocity;
        owner->rotation.z = tiltAngle;
    }
    
    // A helper function for the FlapController to call
    void applyImpulse(float upwardForce) {
        velocity.y = upwardForce;
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        // PhysicsComponent has no parameters in its constructor
        return std::make_shared<PhysicsComponent>();
    }
};

#endif