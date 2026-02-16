#ifndef GAME_H
#define GAME_H

#include <vector>
#include "Entity.h"
#include "Camera.h"
#include "Shader.h"
#include <GLFW/glfw3.h>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

class Game {
public:
    unsigned int VAO, VBO;
    Shader* shader;
    Camera camera;
    std::vector<Entity*> entities;

    Entity* centerCube;
    float shootCooldown = 0.0f;
    
    // Time tracking
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    Game();
    ~Game();

    void init();
    void processInput(GLFWwindow* window);
    void update();
    void render();
};



#endif