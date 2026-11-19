#include "PhysicsSystem.h"
#include "PhysicsComponent.h"
#include "Entity.h"
#include <algorithm>
#include <cmath>

void PhysicsSystem::registerBody(PhysicsComponent* body) {
    if (!body) return;
    unregisterBody(body);
    if (body->isStatic) {
        m_staticBodies.push_back(body);
    } else {
        m_dynamicBodies.push_back(body);
    }
}

void PhysicsSystem::unregisterBody(PhysicsComponent* body) {
    auto itDyn = std::find(m_dynamicBodies.begin(), m_dynamicBodies.end(), body);
    if (itDyn != m_dynamicBodies.end()) {
        m_dynamicBodies.erase(itDyn);
    }
    auto itStat = std::find(m_staticBodies.begin(), m_staticBodies.end(), body);
    if (itStat != m_staticBodies.end()) {
        m_staticBodies.erase(itStat);
    }
}

void PhysicsSystem::clear() {
    m_dynamicBodies.clear();
    m_staticBodies.clear();
}

void PhysicsSystem::update(float deltaTime) {
    // Clamp delta time to avoid instability on long pauses
    float dtClamped = std::min(deltaTime, 0.05f);
    if (dtClamped <= 0.0f) return;

    // Sub-stepping for rock-solid stability
    const int subSteps = 4;
    float subDt = dtClamped / (float)subSteps;

    for (int s = 0; s < subSteps; ++s) {
        stepSimulation(subDt);
    }
}

void PhysicsSystem::stepSimulation(float dt) {
    // 1. Integrate dynamic bodies (velocity & position)
    for (auto* body : m_dynamicBodies) {
        if (!body || !body->owner) continue;

        body->velocity += body->acceleration * dt;
        body->owner->position += body->velocity * dt;
    }

    // 2. Dynamic Sphere vs Dynamic Sphere (Sphere-to-Sphere collisions)
    size_t numDyn = m_dynamicBodies.size();
    for (size_t i = 0; i < numDyn; ++i) {
        auto* bodyA = m_dynamicBodies[i];
        if (!bodyA || !bodyA->owner || bodyA->colliderType != PhysicsColliderType::SPHERE) continue;

        for (size_t j = i + 1; j < numDyn; ++j) {
            auto* bodyB = m_dynamicBodies[j];
            if (!bodyB || !bodyB->owner || bodyB->colliderType != PhysicsColliderType::SPHERE) continue;

            glm::vec3 delta = bodyB->owner->position - bodyA->owner->position;
            float distSq = glm::dot(delta, delta);
            float minDist = bodyA->radius + bodyB->radius;

            if (distSq < minDist * minDist) {
                if (distSq < 1e-8f) {
                    delta = glm::vec3(0.0f, 1.0f, 0.0f);
                    distSq = 1.0f;
                }

                float dist = std::sqrt(distSq);
                glm::vec3 normal = delta / dist; // Points from A to B
                float pen = minDist - dist;

                float invMassA = bodyA->inverseMass;
                float invMassB = bodyB->inverseMass;
                float totalInvMass = invMassA + invMassB;

                if (totalInvMass > 0.0f) {
                    // Position separation
                    bodyA->owner->position -= normal * (pen * (invMassA / totalInvMass));
                    bodyB->owner->position += normal * (pen * (invMassB / totalInvMass));

                    // Velocity impulse
                    glm::vec3 relVel = bodyA->velocity - bodyB->velocity;
                    float vRelN = glm::dot(relVel, normal);

                    if (vRelN > 0.0f) { // Moving toward each other
                        float e = (bodyA->restitution + bodyB->restitution) * 0.5f;
                        float j = (1.0f + e) * vRelN / totalInvMass;
                        glm::vec3 impulse = normal * j;

                        bodyA->velocity -= impulse * invMassA;
                        bodyB->velocity += impulse * invMassB;

                        // Tangential friction
                        glm::vec3 tangent = relVel - normal * vRelN;
                        float tanLen = glm::length(tangent);
                        if (tanLen > 1e-4f) {
                            tangent /= tanLen;
                            float jt = glm::dot(relVel, tangent) / totalInvMass;
                            float maxFriction = j * (bodyA->friction + bodyB->friction) * 0.5f;
                            jt = glm::clamp(jt, -maxFriction, maxFriction);
                            bodyA->velocity -= tangent * (jt * invMassA);
                            bodyB->velocity += tangent * (jt * invMassB);
                        }
                    }
                }
            }
        }
    }

    // 3. Dynamic Sphere vs Static Box Colliders
    for (auto* body : m_dynamicBodies) {
        if (!body || !body->owner || body->colliderType != PhysicsColliderType::SPHERE) continue;

        glm::vec3& c = body->owner->position;
        float r = body->radius;

        for (auto* statBox : m_staticBodies) {
            if (!statBox || !statBox->owner || statBox->colliderType != PhysicsColliderType::BOX) continue;

            glm::vec3 bPos = statBox->owner->position;
            glm::vec3 bMin = bPos - statBox->halfExtents;
            glm::vec3 bMax = bPos + statBox->halfExtents;

            glm::vec3 q = glm::clamp(c, bMin, bMax);
            glm::vec3 diff = c - q;
            float distSq = glm::dot(diff, diff);

            if (distSq < r * r) {
                if (distSq > 1e-6f) {
                    float dist = std::sqrt(distSq);
                    glm::vec3 normal = diff / dist;
                    float pen = r - dist;

                    c += normal * pen;

                    float vN = glm::dot(body->velocity, normal);
                    if (vN < 0.0f) {
                        body->velocity -= (1.0f + body->restitution) * vN * normal;
                        glm::vec3 tangent = body->velocity - normal * glm::dot(body->velocity, normal);
                        body->velocity -= tangent * std::min(1.0f, body->friction * 2.0f);
                    }
                } else {
                    // Center inside box: push out along minimal axis
                    float dx1 = c.x - bMin.x;
                    float dx2 = bMax.x - c.x;
                    float dy1 = c.y - bMin.y;
                    float dy2 = bMax.y - c.y;
                    float dz1 = c.z - bMin.z;
                    float dz2 = bMax.z - c.z;

                    float minPen = dx1;
                    glm::vec3 n(-1.0f, 0.0f, 0.0f);
                    if (dx2 < minPen) { minPen = dx2; n = glm::vec3(1.0f, 0.0f, 0.0f); }
                    if (dy1 < minPen) { minPen = dy1; n = glm::vec3(0.0f, -1.0f, 0.0f); }
                    if (dy2 < minPen) { minPen = dy2; n = glm::vec3(0.0f, 1.0f, 0.0f); }
                    if (dz1 < minPen) { minPen = dz1; n = glm::vec3(0.0f, 0.0f, -1.0f); }
                    if (dz2 < minPen) { minPen = dz2; n = glm::vec3(0.0f, 0.0f, 1.0f); }

                    c += n * (minPen + r);
                    float vN = glm::dot(body->velocity, n);
                    if (vN < 0.0f) {
                        body->velocity -= (1.0f + body->restitution) * vN * n;
                    }
                }
            }
        }
    }

    // 4. Dynamic Sphere vs Floor Plane
    for (auto* body : m_dynamicBodies) {
        if (!body || !body->owner || body->colliderType != PhysicsColliderType::SPHERE) continue;

        float bottom = body->owner->position.y - body->radius;
        if (bottom < m_floorY) {
            float pen = m_floorY - bottom;
            body->owner->position.y += pen;

            if (body->velocity.y < 0.0f) {
                body->velocity.y = -body->velocity.y * body->restitution;
                body->velocity.x *= (1.0f - body->friction * 0.4f);
                body->velocity.z *= (1.0f - body->friction * 0.4f);

                if (std::abs(body->velocity.y) < 0.25f) {
                    body->velocity.y = 0.0f;
                }
            }
        }
    }

    // 5. Arena Bounds Clamping
    if (m_hasBounds) {
        for (auto* body : m_dynamicBodies) {
            if (!body || !body->owner) continue;

            float r = (body->colliderType == PhysicsColliderType::SPHERE) ? body->radius : 0.5f;

            if (body->owner->position.x - r < m_minX) {
                body->owner->position.x = m_minX + r;
                if (body->velocity.x < 0.0f) body->velocity.x = -body->velocity.x * body->restitution;
            } else if (body->owner->position.x + r > m_maxX) {
                body->owner->position.x = m_maxX - r;
                if (body->velocity.x > 0.0f) body->velocity.x = -body->velocity.x * body->restitution;
            }

            if (body->owner->position.z - r < m_minZ) {
                body->owner->position.z = m_minZ + r;
                if (body->velocity.z < 0.0f) body->velocity.z = -body->velocity.z * body->restitution;
            } else if (body->owner->position.z + r > m_maxZ) {
                body->owner->position.z = m_maxZ - r;
                if (body->velocity.z > 0.0f) body->velocity.z = -body->velocity.z * body->restitution;
            }
        }
    }

    // 6. Linear Damping
    for (auto* body : m_dynamicBodies) {
        if (!body) continue;
        body->velocity *= std::pow(body->linearDamping, dt * 60.0f);
    }
}

bool PhysicsSystem::resolveSphereVsStatic(glm::vec3& position, float radius) const {
    bool collided = false;

    // Check against world boundary
    if (m_hasBounds) {
        if (position.x - radius < m_minX) {
            position.x = m_minX + radius;
            collided = true;
        } else if (position.x + radius > m_maxX) {
            position.x = m_maxX - radius;
            collided = true;
        }

        if (position.z - radius < m_minZ) {
            position.z = m_minZ + radius;
            collided = true;
        } else if (position.z + radius > m_maxZ) {
            position.z = m_maxZ - radius;
            collided = true;
        }
    }

    // Check against all static boxes (walls/pillars)
    for (const auto* statBox : m_staticBodies) {
        if (!statBox || !statBox->owner || statBox->colliderType != PhysicsColliderType::BOX) continue;

        glm::vec3 bPos = statBox->owner->position;
        glm::vec3 bMin = bPos - statBox->halfExtents;
        glm::vec3 bMax = bPos + statBox->halfExtents;

        // Player is vertical cylinder/capsule or sphere, we check horizontal box intersection
        glm::vec3 q = glm::clamp(position, bMin, bMax);
        glm::vec3 diff = position - q;
        // Ignore vertical overlap if player is above the box
        diff.y = 0.0f;
        float distSq = glm::dot(diff, diff);

        if (distSq < radius * radius && position.y < bMax.y + 0.1f && position.y > bMin.y - 1.5f) {
            collided = true;
            if (distSq > 1e-6f) {
                float dist = std::sqrt(distSq);
                glm::vec3 normal = diff / dist;
                float pen = radius - dist;
                position += normal * pen;
            } else {
                // Inside: push to closest X/Z face
                float dx1 = position.x - bMin.x;
                float dx2 = bMax.x - position.x;
                float dz1 = position.z - bMin.z;
                float dz2 = bMax.z - position.z;

                float minPen = dx1;
                glm::vec3 n(-1.0f, 0.0f, 0.0f);
                if (dx2 < minPen) { minPen = dx2; n = glm::vec3(1.0f, 0.0f, 0.0f); }
                if (dz1 < minPen) { minPen = dz1; n = glm::vec3(0.0f, 0.0f, -1.0f); }
                if (dz2 < minPen) { minPen = dz2; n = glm::vec3(0.0f, 0.0f, 1.0f); }

                position += n * (minPen + radius);
            }
        }
    }

    return collided;
}
