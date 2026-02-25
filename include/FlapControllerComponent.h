#ifndef FLAP_CONTROLLER_COMPONENT_H
#define FLAP_CONTROLLER_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include "PhysicsComponent.h"
#include <GLFW/glfw3.h> // Assuming you are using GLFW for your window
#include <iostream>

class FlapControllerComponent : public Component {
public:
    GLFWwindow* window;
    float flapForce;
    
    // We use this to ensure the player has to let go of the spacebar before flapping again
    bool spaceWasPressed = false;

    FlapControllerComponent(GLFWwindow* win, float force = 7.0f) 
        : window(win), flapForce(force) {}

    void update(float deltaTime) override {
        if (!owner) return;

        // 1. Poll the keyboard state
        bool spaceIsPressed = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);

        // 2. Check if the key was JUST pressed this frame
        if (spaceIsPressed && !spaceWasPressed) {
            
            // Ask the Entity for its PhysicsComponent
            auto physics = owner->getComponent<PhysicsComponent>();
            
            if (physics) {
                physics->applyImpulse(flapForce);
            } else {
                std::cout << "Warning: FlapController tried to flap, but no PhysicsComponent was found on this entity!" << std::endl;
            }
        }

        // 3. Update the state for the next frame
        spaceWasPressed = spaceIsPressed;
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        float force = 7.0f;
        if (!iss.eof()) {
            iss >> force;
        }
        return std::make_shared<FlapControllerComponent>(window, force);
    }
};

#endif