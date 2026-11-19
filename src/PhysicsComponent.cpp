#include "PhysicsComponent.h"
#include "PhysicsSystem.h"
#include "Entity.h"
#include <algorithm>

PhysicsComponent::PhysicsComponent() {
    setMass(1.0f);
}

PhysicsComponent::~PhysicsComponent() {
    PhysicsSystem::getInstance().unregisterBody(this);
}

void PhysicsComponent::awake() {
    PhysicsSystem::getInstance().registerBody(this);
}

void PhysicsComponent::update(float deltaTime) {
    // Global sub-stepped integration and collision resolution
    // is executed centrally in PhysicsSystem::update.
}

void PhysicsComponent::setStatic(bool s) {
    isStatic = s;
    if (isStatic) {
        inverseMass = 0.0f;
        velocity = glm::vec3(0.0f);
    } else {
        inverseMass = (mass > 0.0001f) ? (1.0f / mass) : 1.0f;
    }
}

void PhysicsComponent::setMass(float m) {
    mass = std::max(0.0001f, m);
    if (!isStatic) {
        inverseMass = 1.0f / mass;
    } else {
        inverseMass = 0.0f;
    }
}

void PhysicsComponent::setSphere(float r) {
    colliderType = PhysicsColliderType::SPHERE;
    radius = std::max(0.01f, r);
}

void PhysicsComponent::setBox(const glm::vec3& extents) {
    colliderType = PhysicsColliderType::BOX;
    halfExtents = glm::max(glm::vec3(0.01f), extents);
}

void PhysicsComponent::applyImpulse(const glm::vec3& impulse) {
    if (!isStatic) {
        velocity += impulse * inverseMass;
    }
}

std::shared_ptr<Component> PhysicsComponent::deserialize(std::istringstream& iss, SDL_Window* window) {
    auto comp = std::make_shared<PhysicsComponent>();
    std::string typeStr;
    if (iss >> typeStr) {
        if (typeStr == "SPHERE") {
            float r = 0.5f;
            iss >> r;
            comp->setSphere(r);
        } else if (typeStr == "BOX") {
            glm::vec3 extents(0.5f);
            iss >> extents.x >> extents.y >> extents.z;
            comp->setBox(extents);
        }
    }
    bool isStat = false;
    if (iss >> isStat) {
        comp->setStatic(isStat);
    }
    return comp;
}
