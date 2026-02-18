#include "../include/Game.h"
#include <iostream>
#include <Material.h>
#include <Mesh.h>
#include <PrimitiveBuilder.h>
#include <ResourceManager.h>
#include <RendererComponent.h>
#include <memory>

Game::Game() : camera(glm::vec3(0.0f, 0.0f, 3.0f)) {
    // Camera starts 3 units back on the Z axis
}

Game::~Game() {
    // Clean up entities
    // Automatic with shared_ptr
    entities.clear();
}

void Game::init() {
    
    auto root_entity = std::make_shared<Entity>();
    entities.push_back(root_entity);

    // 1. Load Assets via your ResourceManager
    auto sahurModel = ResourceManager::loadModel("assets/models/BetterSahur.fbx", "sahur");
    auto metalMaterial = ResourceManager::createMaterial("assets/shaders/shader.vs", "assets/shaders/shader.fs");

    // 2. Create the Entity
    auto sahurEntity = std::make_shared<Entity>();


    // 3. Assemble the Components
    auto renderer = std::make_shared<RendererComponent>(sahurModel, metalMaterial);
    
    // Load Textures
    auto diffuseMap = ResourceManager::loadTexture("assets/textures/glossy-marble-tile_albedo.png", "marble_diffuse");
    auto specularMap = ResourceManager::loadTexture("assets/textures/glossy-marble-tile_specular.png", "marble_specular");
    auto normalMap = ResourceManager::loadTexture("assets/textures/glossy-marble-tile_normal.png", "marble_normal");

    metalMaterial->diffuseMap = diffuseMap;
    metalMaterial->specularMap = specularMap;
    metalMaterial->normalMap = normalMap;
    metalMaterial->shininess = 64.0f;

    std::cout << "Texture IDs: " 
              << diffuseMap->ID << " " 
              << specularMap->ID << " " 
              << normalMap->ID << std::endl;
    std::cout << "Model Meshes: " << sahurModel->meshes.size() << std::endl;

    sahurEntity->addComponent(renderer);
    sahurEntity->position = glm::vec3(0.0f, -1.0f, 0.0f);
    sahurEntity->scale = glm::vec3(0.01f);
    sahurEntity->rotation = glm::vec3(90.0f, 180.0f, 90.0f);

    root_entity->addChild(sahurEntity);

}

void Game::processInput(GLFWwindow* window) {

// Camera WASD Movement
    
    static bool escPressedLastFrame = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (!escPressedLastFrame) {
            mouseCaptured = !mouseCaptured;
            if (mouseCaptured) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                firstMouse = true;
            } else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
            escPressedLastFrame = true;
        }
    } else {
        escPressedLastFrame = false;
    }

    if (!mouseCaptured) return;

    float velocity = camera.movementSpeed * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.position += camera.front * velocity;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.position -= camera.front * velocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.position -= camera.right * velocity;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.position += camera.right * velocity;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.position += camera.worldUp  * velocity;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        camera.position -= camera.worldUp * velocity;

}

void Game::update() {
    // Update all game objects
    for (auto& rootNode : entities) {
        rootNode->update(deltaTime);
        rootNode->updateSelfAndChild();
    }
}

// 2. Traverse the Scene Graph and pass the data down
void Game::render() {
    renderer.clear();
    
    // Setup scene global data
    // Setup scene global data
    glm::vec3 lightPos(2.0f, 4.0f, 3.0f); 
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
    renderer.beginScene(camera, lightPos, lightColor);

    // Lambda for recursive drawing
    // We can't use a local lambda easily with recursion in C++ without std::function overhead or auto deduction tricks
    // So let's just make a member helper or a free function helper that takes the renderer
    
    // Let's use a standard implementation
    for (auto& rootNode : entities) {
        renderer.drawNode(rootNode);
    }

    renderer.endScene();
}

void Game::handleMouse(double xposIn, double yposIn) {
    if (!mouseCaptured) return;

    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // Reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.processMouseMovement(xoffset, yoffset);
}