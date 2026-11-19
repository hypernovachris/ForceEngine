#ifndef ENGINE_H
#define ENGINE_H

#include <vector>
#include <memory>
#include <string>
#include <SDL3/SDL.h>
#include "RHI/RHI_Device.h"
#include "Entity.h"
#include "Renderer.h"
#include "CameraComponent.h"
#include "ResourceManager.h"
#include "PhysicsSystem.h"
#include <functional>

class Engine {
public:
    Engine();
    ~Engine();

    bool init(int width, int height, const char* title);
    bool initWithNativeView(void* nativeView, int width, int height);
    void stepFrame(float dt);
    void resize(int width, int height);
    void run();
    void shutdown();

    // Application callbacks (allow editor or game to override loop)
    std::function<void(const SDL_Event&)> onEvent = nullptr;
    std::function<void(float)> onUpdate = nullptr;
    std::function<void()> onRender = nullptr;

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
    SDL_Window* getWindow() const { return window; }

protected:
    void processInput();
    void update();
    void render();

private:
    SDL_Window* window = nullptr;
    bool isRunning = false;
};

#endif
