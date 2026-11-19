#ifndef PHYSICS_COMPONENT_H
#define PHYSICS_COMPONENT_H

#include "Component.h"
#include <glm/glm/glm.hpp>
#include <sstream>
#include <memory>
#include <SDL3/SDL.h>

enum class PhysicsColliderType {
    SPHERE,
    BOX
};

class PhysicsComponent : public Component {
public:
    PhysicsColliderType colliderType = PhysicsColliderType::SPHERE;

    glm::vec3 velocity{0.0f};
    glm::vec3 acceleration{0.0f, -18.0f, 0.0f}; // Gravity

    float mass = 1.0f;
    float inverseMass = 1.0f;
    bool isStatic = false;

    float restitution = 0.75f;    // Bounciness (0 = no bounce, 1 = elastic)
    float friction = 0.25f;       // Surface friction
    float linearDamping = 0.998f; // Velocity retention per tick

    // Collider dimensions
    float radius = 0.5f;                    // For SPHERE
    glm::vec3 halfExtents{0.5f, 0.5f, 0.5f}; // For BOX (half width/height/depth)

    PhysicsComponent();
    ~PhysicsComponent() override;

    void awake() override;
    void update(float deltaTime) override;

    void setStatic(bool s);
    void setMass(float m);
    void setSphere(float r);
    void setBox(const glm::vec3& extents);

    void applyImpulse(const glm::vec3& impulse);

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, SDL_Window* window);
};

#endif // PHYSICS_COMPONENT_H