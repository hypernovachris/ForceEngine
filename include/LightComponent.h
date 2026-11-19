#ifndef LIGHT_COMPONENT_H
#define LIGHT_COMPONENT_H

#include "Component.h"
#include <glm/glm/glm.hpp>
#include <sstream>
#include <memory>

class LightComponent : public Component {
public:
    glm::vec3 color;
    float intensity;

    // Shadow casting settings
    bool castsShadow = true;
    float fov = 105.0f;        // Wide downward cone (perspective frustum)
    float nearPlane = 0.5f;
    float farPlane = 30.0f;

    // Attenuation variables
    float constant = 1.0f;
    float linear = 0.045f;
    float quadratic = 0.0075f;

    LightComponent(glm::vec3 col, float i, bool shadow = true)
        : color(col), intensity(i), castsShadow(shadow) {}

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, SDL_Window* window) {
        float r, g, b, intensity;
        iss >> r >> g >> b >> intensity;
        bool shadow = true;
        if (!iss.eof()) {
            iss >> shadow;
        }
        return std::make_shared<LightComponent>(glm::vec3(r, g, b), intensity, shadow);
    }
};

#endif // LIGHT_COMPONENT_H