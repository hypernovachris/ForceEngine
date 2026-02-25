#ifndef COLLIDER_COMPONENT_H
#define COLLIDER_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include <vector>
#include <algorithm>
#include <glm/glm/glm.hpp>
#include <GLFW/glfw3.h>

class ColliderComponent : public Component {
public:
    // This static list holds pointers to every collider in the scene
    static std::vector<ColliderComponent*> allColliders;
    
    // The width, height, and depth of the bounding box
    glm::vec3 size; 
    
    // An optional flag so the bird doesn't check collision against itself
    bool isTrigger; 

    ColliderComponent(glm::vec3 boundingBoxSize, bool trigger = false) 
        : size(boundingBoxSize), isTrigger(trigger) {}

    void awake() override {
        // Register this collider the moment it is attached to an entity
        allColliders.push_back(this);
    }

    ~ColliderComponent() override {
        // Remove this collider from the list when the entity is destroyed
        allColliders.erase(std::remove(allColliders.begin(), allColliders.end(), this), allColliders.end());
    }

    // The AABB Math (Assuming the Entity's position is the exact center of the box)
    bool isCollidingWith(ColliderComponent* other) {
        if (!owner || !other->owner) return false;

        glm::vec3 posA = owner->position;
        glm::vec3 posB = other->owner->position;

        bool collisionX = (posA.x + size.x / 2.0f >= posB.x - other->size.x / 2.0f) &&
                          (posB.x + other->size.x / 2.0f >= posA.x - size.x / 2.0f);
        
        bool collisionY = (posA.y + size.y / 2.0f >= posB.y - other->size.y / 2.0f) &&
                          (posB.y + other->size.y / 2.0f >= posA.y - size.y / 2.0f);

        // Flappy Bird is a 2D game in a 3D world, so we only care about X and Y overlaps!
        return collisionX && collisionY;
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        glm::vec3 size(1.0f);
        bool isTrigger = false;
        // The scene file will provide: width height depth [isTrigger]
        iss >> size.x >> size.y >> size.z;
        if (!iss.eof()) {
            iss >> isTrigger;
        }
        return std::make_shared<ColliderComponent>(size, isTrigger);
    }
};

#endif