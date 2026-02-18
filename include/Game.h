#ifndef GAME_H
#define GAME_H

#include <vector>
#include <memory>
#include "Entity.h"
#include "Renderer.h"
#include "Camera.h"
#include "Shader.h"
#include "ResourceManager.h"
#include <GLFW/glfw3.h>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

class Game {
public:
    Renderer renderer;
    unsigned int VAO, VBO;
    Camera camera;
    std::vector<std::shared_ptr<Entity>> entities;
    
    // Time tracking
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    // Mouse state
    bool mouseCaptured = true;
    bool firstMouse = true;
    float lastX = 400.0f;
    float lastY = 300.0f;

    Game();
    ~Game();

    void init();
    void processInput(GLFWwindow* window);
    void handleMouse(double xpos, double ypos);
    void update();
    void render();
};



#endif