#include "FirstPersonControllerComponent.h"
#include "PhysicsComponent.h"
#include "PhysicsSystem.h"
#include "RendererComponent.h"
#include "PrimitiveBuilder.h"
#include "ResourceManager.h"
#include "Entity.h"
#include <cmath>
#include <iostream>

FirstPersonControllerComponent::FirstPersonControllerComponent(SDL_Window* win)
    : window(win) {
}

void FirstPersonControllerComponent::awake() {
    initBallAssets();
    if (owner) {
        m_camera = owner->getComponent<CameraComponent>();
    }
}

void FirstPersonControllerComponent::initBallAssets() {
    // 1. Create a single high-quality sphere mesh shared by all balls
    auto baseSphere = PrimitiveBuilder::createSphere(20, 20, ballRadius, true, true);
    if (baseSphere->meshes.empty()) return;
    auto sharedMesh = baseSphere->meshes[0];

    // 2. Load diverse materials for balls
    std::vector<std::string> matPaths = {
        "assets/materials/glossy_tile.ForceMaterial",
        "assets/materials/tung.ForceMaterial",
        "assets/materials/david.ForceMaterial",
        "assets/materials/sun.ForceMaterial"
    };

    for (const auto& path : matPaths) {
        auto mat = ResourceManager::loadForceMaterial(path);
        if (mat) {
            auto model = std::make_shared<Model>();
            model->meshes.push_back(sharedMesh);
            model->materials.push_back(mat);
            m_ballMaterials.push_back(mat);
            m_sphereModel = model; // Keep reference
        }
    }
}

void FirstPersonControllerComponent::update(float deltaTime) {
    if (!owner) return;

    // 1. Mouse Look via Relative Mouse Delta
    float mouseDx = 0.0f, mouseDy = 0.0f;
    SDL_MouseButtonFlags mouseButtons = SDL_GetRelativeMouseState(&mouseDx, &mouseDy);

    yaw += mouseDx * mouseSensitivity;
    pitch -= mouseDy * mouseSensitivity;

    // Clamp pitch to avoid screen flip
    pitch = glm::clamp(pitch, -89.0f, 89.0f);

    // Keep yaw normalized
    if (yaw > 360.0f) yaw -= 360.0f;
    if (yaw < 0.0f) yaw += 360.0f;

    // Compute Direction Vectors
    float radYaw = glm::radians(yaw);
    float radPitch = glm::radians(pitch);

    glm::vec3 front;
    front.x = sin(radYaw) * cos(radPitch);
    front.y = sin(radPitch);
    front.z = -cos(radYaw) * cos(radPitch);
    front = glm::normalize(front);

    glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
    glm::vec3 up = glm::normalize(glm::cross(right, front));

    // Update Camera
    if (!m_camera) {
        m_camera = owner->getComponent<CameraComponent>();
    }
    if (m_camera) {
        m_camera->setLookVectors(front, up);
    }

    // 2. WASD Movement
    glm::vec3 planarForward = glm::normalize(glm::vec3(front.x, 0.0f, front.z));
    glm::vec3 planarRight = glm::normalize(glm::vec3(right.x, 0.0f, right.z));

    const bool* keys = SDL_GetKeyboardState(nullptr);
    glm::vec3 moveDir(0.0f);
    if (keys[SDL_SCANCODE_W]) moveDir += planarForward;
    if (keys[SDL_SCANCODE_S]) moveDir -= planarForward;
    if (keys[SDL_SCANCODE_D]) moveDir += planarRight;
    if (keys[SDL_SCANCODE_A]) moveDir -= planarRight;

    float currentSpeed = moveSpeed;
    if (keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT]) {
        currentSpeed *= sprintMultiplier;
    }

    if (glm::length(moveDir) > 0.001f) {
        moveDir = glm::normalize(moveDir);
        glm::vec3 targetPos = owner->position + moveDir * currentSpeed * deltaTime;

        // Slide along static obstacles
        PhysicsSystem::getInstance().resolveSphereVsStatic(targetPos, playerRadius);

        owner->position.x = targetPos.x;
        owner->position.z = targetPos.z;
    }

    // 3. Jumping / Vertical Gravity
    bool spaceDown = keys[SDL_SCANCODE_SPACE];
    if (spaceDown && !m_spaceDownLastFrame && isGrounded) {
        yVelocity = jumpForce;
        isGrounded = false;
    }
    m_spaceDownLastFrame = spaceDown;

    if (!isGrounded) {
        yVelocity += gravity * deltaTime;
        owner->position.y += yVelocity * deltaTime;
        if (owner->position.y <= eyeHeight) {
            owner->position.y = eyeHeight;
            yVelocity = 0.0f;
            isGrounded = true;
        }
    } else {
        owner->position.y = eyeHeight;
    }

    // 4. Ball Shooting (Left Mouse Button)
    shootTimer -= deltaTime;
    bool leftMouseDown = (mouseButtons & SDL_BUTTON_LMASK) != 0;
    if (leftMouseDown && shootTimer <= 0.0f) {
        shootBall();
        shootTimer = shootCooldown;
    }
    m_leftMouseDownLastFrame = leftMouseDown;

    // 5. Clear Balls (Key 'R')
    bool rDown = keys[SDL_SCANCODE_R];
    if (rDown && !m_rDownLastFrame) {
        clearAllBalls();
    }
    m_rDownLastFrame = rDown;
}

void FirstPersonControllerComponent::shootBall() {
    if (!owner) return;

    // Find root entity to attach ball to
    Entity* root = owner;
    while (root->parent) {
        root = root->parent;
    }

    float radYaw = glm::radians(yaw);
    float radPitch = glm::radians(pitch);
    glm::vec3 front;
    front.x = sin(radYaw) * cos(radPitch);
    front.y = sin(radPitch);
    front.z = -cos(radYaw) * cos(radPitch);
    front = glm::normalize(front);

    auto ball = std::make_shared<Entity>();
    ball->name = "Ball";
    ball->position = owner->position + front * 0.9f;

    // Build model with current material
    if (!m_ballMaterials.empty()) {
        auto mat = m_ballMaterials[m_materialIndex % m_ballMaterials.size()];
        m_materialIndex++;

        auto model = std::make_shared<Model>();
        model->meshes = m_sphereModel->meshes;
        model->materials.push_back(mat);

        ball->addComponent(std::make_shared<RendererComponent>(model));
    }

    // Add 3D RigidBody Physics
    auto phys = std::make_shared<PhysicsComponent>();
    phys->setSphere(ballRadius);
    phys->setMass(1.0f);
    phys->restitution = ballRestitution;
    phys->friction = 0.25f;
    phys->linearDamping = 0.998f;
    phys->velocity = front * shootSpeed + glm::vec3(0.0f, 1.0f, 0.0f);

    ball->addComponent(phys);

    root->addChild(ball);
    m_activeBalls.push_back(ball);

    // FIFO recycling to prevent infinite accumulation
    if (m_activeBalls.size() > m_maxBalls) {
        auto oldest = m_activeBalls.front();
        m_activeBalls.pop_front();
        oldest->pendingDestroy = true;
    }
}

void FirstPersonControllerComponent::clearAllBalls() {
    for (auto& ball : m_activeBalls) {
        if (ball) {
            ball->pendingDestroy = true;
        }
    }
    m_activeBalls.clear();
}

std::shared_ptr<Component> FirstPersonControllerComponent::deserialize(std::istringstream& iss, SDL_Window* window) {
    return std::make_shared<FirstPersonControllerComponent>(window);
}
