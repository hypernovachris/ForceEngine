#ifndef LIGHT_COMPONENT_H
#define LIGHT_COMPONENT_H

#include "Component.h"
#include <glm/glm/glm.hpp>
#include <sstream>
#include <memory>
#include <GLFW/glfw3.h>

class LightComponent : public Component {
public:
    glm::vec3 color;
    float intensity;

    // Attenuation variables (how the light fades over distance)
    float constant = 1.0f;
    float linear = 0.09f;
    float quadratic = 0.032f;

    LightComponent(glm::vec3 col, float i) : color(col), intensity(i) {}

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        float r, g, b, intensity;
        iss >> r >> g >> b >> intensity;
        return std::make_shared<LightComponent>(glm::vec3(r, g, b), intensity);
    }
};

#endif