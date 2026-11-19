#ifndef FIRST_PERSON_CONTROLLER_COMPONENT_H
#define FIRST_PERSON_CONTROLLER_COMPONENT_H

#include "Component.h"
#include "CameraComponent.h"
#include "Model.h"
#include "Material.h"
#include <memory>
#include <deque>
#include <vector>
#include <glm/glm/glm.hpp>
#include <SDL3/SDL.h>

class FirstPersonControllerComponent : public Component {
public:
    SDL_Window* window = nullptr;

    float moveSpeed = 7.0f;
    float sprintMultiplier = 1.6f;
    float mouseSensitivity = 0.12f;
    float playerRadius = 0.45f;
    float eyeHeight = 1.7f;

    // View angles
    float yaw = 0.0f;   // Looking along -Z when yaw=0
    float pitch = 0.0f; // Horizon

    // Vertical / jumping motion
    float yVelocity = 0.0f;
    float gravity = -18.0f;
    float jumpForce = 6.5f;
    bool isGrounded = true;

    // Ball shooting
    float shootSpeed = 22.0f;
    float ballRadius = 0.35f;
    float ballRestitution = 0.80f;
    float shootCooldown = 0.16f;
    float shootTimer = 0.0f;

    FirstPersonControllerComponent(SDL_Window* win);
    ~FirstPersonControllerComponent() override = default;

    void awake() override;
    void update(float deltaTime) override;

    void shootBall();
    void clearAllBalls();

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, SDL_Window* window);

private:
    std::shared_ptr<CameraComponent> m_camera;
    std::deque<std::shared_ptr<Entity>> m_activeBalls;
    const size_t m_maxBalls = 70;

    std::vector<std::shared_ptr<Material>> m_ballMaterials;
    std::shared_ptr<Model> m_sphereModel;
    int m_materialIndex = 0;

    bool m_leftMouseDownLastFrame = false;
    bool m_spaceDownLastFrame = false;
    bool m_rDownLastFrame = false;

    void initBallAssets();
};

#endif // FIRST_PERSON_CONTROLLER_COMPONENT_H
