#ifndef FLAP_CONTROLLER_COMPONENT_H
#define FLAP_CONTROLLER_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include "PhysicsComponent.h"
#include <SDL3/SDL.h>
#include <iostream>

class FlapControllerComponent : public Component {
public:
    SDL_Window* window;
    float flapForce;
    
    // We use this to ensure the player has to let go of the spacebar before flapping again
    bool spaceWasPressed = false;

    FlapControllerComponent(SDL_Window* win, float force = 7.0f) 
        : window(win), flapForce(force) {}

    void update(float deltaTime) override {
        if (!owner) return;

        // 1. Poll the keyboard state
        const bool* state = SDL_GetKeyboardState(NULL);
        bool spaceIsPressed = state[SDL_SCANCODE_SPACE];

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

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, SDL_Window* window) {
        float force = 7.0f;
        if (!iss.eof()) {
            iss >> force;
        }
        return std::make_shared<FlapControllerComponent>(window, force);
    }
};

#endif