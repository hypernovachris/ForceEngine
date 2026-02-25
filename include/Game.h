#ifndef GAME_H
#define GAME_H

#include <vector>
#include <memory>
#include "Entity.h"
#include "Renderer.h"
#include "CameraComponent.h"
#include "Shader.h"
#include "ResourceManager.h"
#include <GLFW/glfw3.h>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

class Game {
public:
    Renderer renderer;
    unsigned int VAO, VBO;
    CameraComponent* activeCamera = nullptr;
    std::vector<std::shared_ptr<Entity>> entities;
    std::shared_ptr<Entity> visualEntity;
    
    // Debug state
    bool debugMode = false;
    std::shared_ptr<Model> debugCubeModel;
    
    // Time tracking
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    Game();
    ~Game();

    void init(GLFWwindow* window);
    void processInput(GLFWwindow* window);
    void update();
    void render();
};



#endif