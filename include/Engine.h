#ifndef ENGINE_H
#define ENGINE_H

#include <vector>
#include <memory>
#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Entity.h"
#include "Renderer.h"
#include "CameraComponent.h"
#include "ResourceManager.h"

class Engine {
public:
    Engine();
    ~Engine();

    bool init(int width, int height, const char* title);
    void run();
    void shutdown();

    // The root of the scene graph
    std::shared_ptr<Entity> rootEntity;

    // Rendering subsystem
    Renderer renderer;
    CameraComponent* activeCamera = nullptr;
    
    // Debug state (toggled by engine for now)
    bool debugMode = false;
    std::shared_ptr<Model> debugCubeModel;

    // Time tracking
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    // Expose the window if the specific game apps need it
    GLFWwindow* getWindow() const { return window; }

protected:
    void processInput();
    void update();
    void render();

private:
    GLFWwindow* window = nullptr;
};

#endif
