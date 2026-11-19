#include "Road/VehiclePhysics.h"
#include "PrimitiveBuilder.h"
#include "ResourceManager.h"
#include <glm/glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>

VehiclePhysics::VehiclePhysics() {
}

void VehiclePhysics::init() {
    buildCarModel();
}

void VehiclePhysics::reset(const glm::vec3& startPos, const glm::vec3& startForward) {
    m_position = startPos;
    m_speed = 0.0f;
    m_steer = 0.0f;

    glm::vec3 f = glm::normalize(startForward);
    if (glm::length(f) < 0.1f) f = glm::vec3(0.0f, 0.0f, 1.0f);
    m_forward = f;
    m_yaw = std::atan2(f.x, f.z);

    m_camPos = m_position - m_forward * 8.0f + glm::vec3(0.0f, 3.2f, 0.0f);
    m_onRoad = true;
}

void VehiclePhysics::update(float dt, const bool* keys,
                            const std::vector<ProcessedRoadSegment>& roadSegments,
                            const std::vector<RoadJunction>& junctions) {
    if (dt <= 0.0f) return;
    dt = std::min(dt, 0.05f);

    bool throttle = keys && (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]);
    bool brake    = keys && (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]);
    bool left     = keys && (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]);
    bool right    = keys && (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]);
    bool handbrake = keys && keys[SDL_SCANCODE_SPACE];

    // Longitudinal Acceleration / Braking
    const float maxForwardSpeed = 45.0f; // ~100 mph
    const float maxReverseSpeed = -12.0f; // ~27 mph
    const float accel = 14.0f;           // m/s^2
    const float braking = 24.0f;
    const float handbrakeBraking = 40.0f;

    if (throttle) {
        if (m_speed < 0.0f) {
            m_speed += braking * dt;
        } else {
            m_speed += accel * dt;
            if (m_speed > maxForwardSpeed) m_speed = maxForwardSpeed;
        }
    } else if (brake) {
        if (m_speed > 0.0f) {
            m_speed -= braking * dt;
            if (m_speed < 0.0f) m_speed = 0.0f;
        } else {
            m_speed -= accel * 0.6f * dt;
            if (m_speed < maxReverseSpeed) m_speed = maxReverseSpeed;
        }
    } else {
        // Natural rolling resistance and aerodynamic drag
        m_speed *= std::pow(0.975f, dt * 60.0f);
        if (std::abs(m_speed) < 0.05f) m_speed = 0.0f;
    }

    if (handbrake) {
        if (m_speed > 0.0f) {
            m_speed -= handbrakeBraking * dt;
            if (m_speed < 0.0f) m_speed = 0.0f;
        } else if (m_speed < 0.0f) {
            m_speed += handbrakeBraking * dt;
            if (m_speed > 0.0f) m_speed = 0.0f;
        }
    }

    // Steering
    float targetSteer = 0.0f;
    float maxSteerAngle = glm::radians(32.0f);
    // Taper steering at high speeds for realistic highway stability
    float speedFactor = std::clamp(std::abs(m_speed) / 25.0f, 0.0f, 1.0f);
    maxSteerAngle = glm::mix(maxSteerAngle, glm::radians(14.0f), speedFactor);

    if (left)  targetSteer += maxSteerAngle;
    if (right) targetSteer -= maxSteerAngle;

    m_steer = glm::mix(m_steer, targetSteer, 1.0f - std::exp(-12.0f * dt));

    // Bicycle kinematic model: dYaw = (v / wheelbase) * tan(steer)
    const float wheelbase = 2.7f;
    if (std::abs(m_speed) > 0.1f) {
        float dir = (m_speed >= 0.0f) ? 1.0f : -1.0f;
        float dYaw = dir * (m_speed / wheelbase) * std::tan(m_steer) * dt;
        m_yaw += dYaw;
    }

    m_forward = glm::normalize(glm::vec3(std::sin(m_yaw), 0.0f, std::cos(m_yaw)));
    m_position += m_forward * (m_speed * dt);

    // Check passability & nearest road segment, and track 3D road surface elevation
    m_onRoad = false;
    m_inJunction = false;
    m_currentJunctionId = -1;
    m_currentRoadName = "Off-road";
    float closestDistSq = 1e9f;
    float targetY = 0.05f;

    // 1. Check road segment ribbons
    for (const auto& seg : roadSegments) {
        float halfW = seg.totalWidth * 0.5f;
        for (const auto& sp : seg.splinePoints) {
            glm::vec2 delta(m_position.x - sp.pos.x, m_position.z - sp.pos.z);
            float d2 = glm::dot(delta, delta);
            if (d2 <= halfW * halfW && d2 < closestDistSq && std::abs(m_position.y - sp.pos.y) < 3.5f) {
                closestDistSq = d2;
                m_onRoad = true;
                targetY = sp.pos.y;
                m_currentRoadName = seg.name;
            }
        }
    }

    // 2. Check junction patch plate collisions
    for (const auto& junc : junctions) {
        glm::vec2 delta(m_position.x - junc.position.x, m_position.z - junc.position.z);
        float distSq = glm::dot(delta, delta);
        if (distSq > 80.0f * 80.0f) continue; // Broadphase distance filter

        bool inJuncTri = false;
        float triSurfaceY = junc.position.y;

        if (junc.patchIndices.size() >= 3) {
            for (size_t t = 0; t + 2 < junc.patchIndices.size(); t += 3) {
                const auto& v0 = junc.patchVertices[junc.patchIndices[t + 0]];
                const auto& v1 = junc.patchVertices[junc.patchIndices[t + 1]];
                const auto& v2 = junc.patchVertices[junc.patchIndices[t + 2]];

                glm::vec2 a(v0.position[0], v0.position[2]);
                glm::vec2 b(v1.position[0], v1.position[2]);
                glm::vec2 c(v2.position[0], v2.position[2]);
                glm::vec2 p(m_position.x, m_position.z);

                float cp0 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
                float cp1 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
                float cp2 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

                bool inside = ((cp0 >= -1e-3f && cp1 >= -1e-3f && cp2 >= -1e-3f) ||
                               (cp0 <=  1e-3f && cp1 <=  1e-3f && cp2 <=  1e-3f));

                if (inside) {
                    float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                    float curTriY = (v0.position[1] + v1.position[1] + v2.position[1]) / 3.0f;
                    if (std::abs(area) > 1e-5f) {
                        float u = ((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) / area;
                        float v = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) / area;
                        float w = 1.0f - u - v;
                        curTriY = u * v0.position[1] + v * v1.position[1] + w * v2.position[1];
                    }

                    if (std::abs(m_position.y - curTriY) < 3.5f) {
                        inJuncTri = true;
                        triSurfaceY = curTriY;
                        break;
                    }
                }
            }
        }

        // Also test proximity to junction centroid if within junction core
        if (!inJuncTri && distSq <= 10.0f * 10.0f && std::abs(m_position.y - junc.position.y) < 3.5f) {
            inJuncTri = true;
            triSurfaceY = junc.position.y;
        }

        if (inJuncTri) {
            m_onRoad = true;
            m_inJunction = true;
            m_currentJunctionId = (int)junc.id;
            targetY = triSurfaceY;
            m_currentRoadName = "Junction #" + std::to_string(junc.id) +
                                " (" + std::to_string(junc.connections.size()) + "-way)";
            break;
        }
    }

    if (m_onRoad) {
        m_position.y = glm::mix(m_position.y, targetY, std::min(1.0f, 15.0f * dt));
        m_lastRoadPos = m_position;
    } else {
        // If the car drove off an elevated structure (y > 2.0m), guardrail collision holds the vehicle on the road
        if (m_lastRoadPos.y > 2.0f && m_position.y > 1.5f) {
            m_position = m_lastRoadPos;
            m_speed *= -0.2f; // Rebound off guardrail
            m_onRoad = true;
        } else {
            // Off-road on ground plane: apply rough terrain drag
            m_speed *= std::pow(0.92f, dt * 60.0f);
            m_position.y = glm::mix(m_position.y, 0.05f, std::min(1.0f, 6.0f * dt));
        }
    }

    // Smooth Chase Camera
    glm::vec3 targetCamPos = m_position - m_forward * 6.8f + glm::vec3(0.0f, 2.4f, 0.0f);
    m_camPos = glm::mix(m_camPos, targetCamPos, 1.0f - std::exp(-8.0f * dt));
}

glm::mat4 VehiclePhysics::getTransform() const {
    glm::mat4 t = glm::translate(glm::mat4(1.0f), m_position);
    t = glm::rotate(t, m_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    return t;
}

glm::mat4 VehiclePhysics::getCameraViewMatrix() const {
    glm::vec3 lookTarget = m_position + m_forward * 2.5f + glm::vec3(0.0f, 0.8f, 0.0f);
    return glm::lookAt(m_camPos, lookTarget, glm::vec3(0.0f, 1.0f, 0.0f));
}

void VehiclePhysics::buildCarModel() {
    // Build a stylized sports coupe mesh
    // Chassis: 4.2m long, 1.9m wide, 1.2m tall
    auto body = PrimitiveBuilder::createCube(1.9f, 0.65f, 4.2f, true, true);
    auto mat = ResourceManager::loadForceMaterial("assets/materials/david.ForceMaterial");
    if (!mat) {
        mat = ResourceManager::loadForceMaterial("assets/materials/glossy_tile.ForceMaterial");
    }

    m_carModel = std::make_shared<Model>();
    if (!body->meshes.empty()) {
        m_carModel->meshes.push_back(body->meshes[0]);
        m_carModel->materials.push_back(mat);
    }
}

