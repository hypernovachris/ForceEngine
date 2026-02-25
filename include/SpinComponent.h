#ifndef SPIN_COMPONENT_H
#define SPIN_COMPONENT_H

#include "Component.h"
#include "Entity.h" // Required so we can access owner->localTransform
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm/gtx/euler_angles.hpp>
#include <GLFW/glfw3.h>

class SpinComponent : public Component {
public:
    glm::vec3 spinAxis;
    float speed; // In degrees per second

    glm::vec3 pivot;
    bool hasPivot;

    float currentAngle;
    glm::vec3 initialOffset;
    glm::mat4 initialRotationMat;

    // Constructor takes the axis of rotation and the speed
    SpinComponent(glm::vec3 axis, float rotationSpeed) 
        : spinAxis(glm::normalize(axis)), speed(rotationSpeed), pivot(0.0f), hasPivot(false), currentAngle(0.0f), initialOffset(0.0f), initialRotationMat(1.0f) {}

    // Constructor with pivot
    SpinComponent(glm::vec3 axis, float rotationSpeed, glm::vec3 pivotPoint) 
        : spinAxis(glm::normalize(axis)), speed(rotationSpeed), pivot(pivotPoint), hasPivot(true), currentAngle(0.0f), initialOffset(0.0f), initialRotationMat(1.0f) {}

    void awake() override {
        if (owner && hasPivot) {
            initialOffset = owner->position - pivot;

            // Build the initial rotation matrix using Entity's right-to-left XYZ convention
            glm::mat4 r(1.0f);
            r = glm::rotate(r, glm::radians(owner->rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
            r = glm::rotate(r, glm::radians(owner->rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
            r = glm::rotate(r, glm::radians(owner->rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
            initialRotationMat = r;
        }
    }

    void update(float deltaTime) override {
        // Safety check to ensure the component is attached to an entity
        if (!owner) return;

        if (hasPivot) {
            // 1. Accumulate angle based on speed
            currentAngle += speed * deltaTime;
            float angle = glm::radians(currentAngle);
            
            // 2. Generate the rotation matrix for the orbit
            glm::mat4 orbitRot = glm::rotate(glm::mat4(1.0f), angle, spinAxis);
            
            // 3. Apply rotation to position offset
            owner->position = pivot + glm::vec3(orbitRot * glm::vec4(initialOffset, 1.0f));

            // 4. Properly rotate the object's orientation around the pivot!
            // We multiply the orbit rotation by the initial rotation to get the rigid body rotation
            glm::mat4 newRotMat = orbitRot * initialRotationMat;
            
            // 5. Extract Euler angles back so Entity can build its matrix
            float rotX, rotY, rotZ;
            glm::extractEulerAngleXYZ(newRotMat, rotX, rotY, rotZ);
            owner->rotation = glm::degrees(glm::vec3(rotX, rotY, rotZ));
        } else {
            // Apply a continuous rotation to the owner's rotation euler angles
            owner->rotation += spinAxis * speed * deltaTime;
        }
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        float x, y, z, speed;
        
        // Read the axis and speed
        if (iss >> x >> y >> z >> speed) {
            // Check if there is a pivot point (4th parameter)
            if (iss >> std::ws && iss.peek() != EOF) {
                float px, py, pz;
                iss >> px >> py >> pz;
                return std::make_shared<SpinComponent>(glm::vec3(x, y, z), speed, glm::vec3(px, py, pz));
            } else {
                return std::make_shared<SpinComponent>(glm::vec3(x, y, z), speed);
            }
        }
        return nullptr;
    }
};

#endif