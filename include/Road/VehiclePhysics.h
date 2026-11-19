#ifndef VEHICLE_PHYSICS_H
#define VEHICLE_PHYSICS_H

#include <vector>
#include <memory>
#include <glm/glm/glm.hpp>
#include <SDL3/SDL.h>
#include "Road/RoadTypes.h"
#include "Model.h"

class VehiclePhysics {
public:
    VehiclePhysics();

    void init();
    void reset(const glm::vec3& startPos, const glm::vec3& startForward);
    void update(float dt, const bool* keyboardState,
                const std::vector<ProcessedRoadSegment>& roadSegments,
                const std::vector<RoadJunction>& junctions = {});

    glm::vec3 getPosition() const { return m_position; }
    glm::vec3 getForward() const { return m_forward; }
    float getSpeedMph() const { return std::abs(m_speed) * 2.23694f; }
    float getHeading() const { return m_yaw; }

    glm::mat4 getTransform() const;
    glm::mat4 getCameraViewMatrix() const;

    std::shared_ptr<Model> getModel() const { return m_carModel; }

    bool isOnRoad() const { return m_onRoad; }
    bool isInJunction() const { return m_inJunction; }
    int getCurrentJunctionId() const { return m_currentJunctionId; }
    std::string getCurrentRoadName() const { return m_currentRoadName; }

private:
    glm::vec3 m_position{0.0f, 0.05f, 0.0f};
    glm::vec3 m_forward{0.0f, 0.0f, 1.0f};
    glm::vec3 m_up{0.0f, 1.0f, 0.0f};
    glm::vec3 m_camPos{0.0f, 3.0f, -8.0f};

    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_roll = 0.0f;

    float m_speed = 0.0f; // m/s
    float m_steer = 0.0f; // radians

    bool m_onRoad = false;
    bool m_inJunction = false;
    int m_currentJunctionId = -1;
    glm::vec3 m_lastRoadPos{0.0f, 0.05f, 0.0f};
    std::string m_currentRoadName = "None";

    std::shared_ptr<Model> m_carModel;

    void buildCarModel();
};

#endif // VEHICLE_PHYSICS_H

