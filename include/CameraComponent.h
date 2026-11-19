#ifndef CAMERA_COMPONENT_H
#define CAMERA_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include <glm/glm/glm.hpp>
#include <sstream>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <SDL3/SDL.h>

class CameraComponent : public Component {
public:
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;

    bool useExplicitVectors = false;
    glm::vec3 frontVector{0.0f, 0.0f, -1.0f};
    glm::vec3 upVector{0.0f, 1.0f, 0.0f};

    CameraComponent(float fov = 60.0f, float aspectRatio = 800.0f / 600.0f, float nearPlane = 0.1f, float farPlane = 200.0f)
        : fov(fov), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane) {}

    void setLookVectors(const glm::vec3& front, const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f)) {
        useExplicitVectors = true;
        frontVector = glm::normalize(front);
        upVector = glm::normalize(up);
    }

    glm::mat4 getViewMatrix() const {
        if (!owner) return glm::mat4(1.0f);
        
        glm::vec3 position = glm::vec3(owner->worldTransform[3]);

        if (useExplicitVectors) {
            return glm::lookAt(position, position + frontVector, upVector);
        }

        glm::vec3 forward = -glm::normalize(glm::vec3(owner->worldTransform[2]));
        glm::vec3 up = glm::normalize(glm::vec3(owner->worldTransform[1]));
        
        return glm::lookAt(position, position + forward, up);
    }

    glm::mat4 getProjectionMatrix() const {
        return glm::perspective(glm::radians(fov), aspectRatio, nearPlane, farPlane);
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, SDL_Window* window) {
        float fov = 60.0f;
        float aspect = 800.0f / 600.0f;
        float nearP = 0.1f;
        float farP = 200.0f;
        
        if (iss >> fov) {
            if (iss >> aspect) {
                if (iss >> nearP) {
                    iss >> farP;
                }
            }
        }
        
        return std::make_shared<CameraComponent>(fov, aspect, nearP, farP);
    }
};

#endif // CAMERA_COMPONENT_H
