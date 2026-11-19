#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include <vector>
#include <memory>
#include <glm/glm/glm.hpp>

class PhysicsComponent;

class PhysicsSystem {
public:
    static PhysicsSystem& getInstance() {
        static PhysicsSystem instance;
        return instance;
    }

    void registerBody(PhysicsComponent* body);
    void unregisterBody(PhysicsComponent* body);
    void clear();

    void update(float deltaTime);

    void setFloorLevel(float y) { m_floorY = y; }
    float getFloorLevel() const { return m_floorY; }

    // World boundary limits (for bounding the procedural arena)
    void setWorldBounds(float minX, float maxX, float minZ, float maxZ) {
        m_hasBounds = true;
        m_minX = minX;
        m_maxX = maxX;
        m_minZ = minZ;
        m_maxZ = maxZ;
    }

    // Sphere sweep / collision check against static colliders (used by player controller)
    bool resolveSphereVsStatic(glm::vec3& position, float radius) const;

private:
    PhysicsSystem() = default;
    ~PhysicsSystem() = default;
    PhysicsSystem(const PhysicsSystem&) = delete;
    PhysicsSystem& operator=(const PhysicsSystem&) = delete;

    std::vector<PhysicsComponent*> m_dynamicBodies;
    std::vector<PhysicsComponent*> m_staticBodies;

    float m_floorY = 0.0f;
    bool m_hasBounds = false;
    float m_minX = -100.0f, m_maxX = 100.0f;
    float m_minZ = -100.0f, m_maxZ = 100.0f;

    void stepSimulation(float dt);
};

#endif // PHYSICS_SYSTEM_H
