#ifndef ENTITY_H
#define ENTITY_H

#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>

class Entity {
public:
    glm::vec3 position;
    glm::vec3 rotation; // Euler angles (pitch, yaw, roll)
    glm::vec3 scale;

    Entity() : position(0.0f), rotation(0.0f), scale(1.0f) {}

    // Generates the Model Matrix used to place the object in the 3D world
    glm::mat4 getModelMatrix() const {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, position);
        // Apply rotations (simplified for now)
        model = glm::rotate(model, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        model = glm::rotate(model, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::scale(model, scale);
        return model;
    }

    virtual void update(float deltaTime) {} // To be overridden by subclasses
};

class SpinningCube : public Entity {
public:
    void update(float deltaTime) override {
        // Rotate continuously on Y and Z axes
        rotation.y += 45.0f * deltaTime;
        rotation.z += 45.0f * deltaTime;
    }
};

class Projectile : public Entity {
public:
    Entity* target;
    glm::vec3 velocity;
    float gravityStrength;

    // Constructor takes the target, starting position, and the initial push (velocity)
    Projectile(Entity* t, glm::vec3 startPos, glm::vec3 startVelocity) 
        : target(t), velocity(startVelocity), gravityStrength(15.0f) {
        
        position = startPos;
        scale = glm::vec3(0.15f); // Make the projectiles tiny
    }

    void update(float deltaTime) override {
        if (!target) return;

        // 1. Find the direction vector to the target: target_pos - current_pos
        glm::vec3 direction = target->position - this->position;
        float distance = glm::length(direction);

        // 2. Only apply gravity if we aren't already colliding with the center
        if (distance > 0.2f) {
            glm::vec3 normalizedDirection = glm::normalize(direction);
            
            // 3. Add gravitational acceleration to our velocity
            velocity += normalizedDirection * gravityStrength * deltaTime;
        }

        // 4. Update position based on our current velocity
        position += velocity * deltaTime;
        
        // Bonus: Make the projectiles spin wildly as they fly
        rotation.x += 180.0f * deltaTime;
        rotation.y += 180.0f * deltaTime;
    }
};

#endif