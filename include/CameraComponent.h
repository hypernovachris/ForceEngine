#ifndef CAMERA_COMPONENT_H
#define CAMERA_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h> // Added GLFW include for deserialization

class CameraComponent : public Component {
public:
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;

    CameraComponent(float fov = 45.0f, float aspectRatio = 800.0f / 600.0f, float nearPlane = 0.1f, float farPlane = 100.0f)
        : fov(fov), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane) {}

    glm::mat4 getViewMatrix() const {
        if (!owner) return glm::mat4(1.0f);
        
        glm::vec3 position = glm::vec3(owner->worldTransform[3]);
        glm::vec3 forward = -glm::normalize(glm::vec3(owner->worldTransform[2]));
        glm::vec3 up = glm::normalize(glm::vec3(owner->worldTransform[1]));
        
        return glm::lookAt(position, position + forward, up);
    }

    glm::mat4 getProjectionMatrix() const {
        return glm::perspective(glm::radians(fov), aspectRatio, nearPlane, farPlane);
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        float fov = 45.0f;
        float aspect = 800.0f / 600.0f;
        float nearP = 0.1f;
        float farP = 100.0f;
        
        // Read up to 4 values
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

#endif
