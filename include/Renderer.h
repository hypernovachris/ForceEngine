#ifndef RENDERER_H
#define RENDERER_H

#include <glm/glm/glm.hpp>
#include <memory>
#include "Shader.h"
#include "Model.h"
#include "Material.h"
#include "CameraComponent.h"

class Entity;

struct PointLightData {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
};

struct DrawCommand {
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Material> material;
    glm::mat4 transform;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize OpenGL state
    void init();

    // Clear frame buffers
    void clear();

    // Prepare the scene for rendering (set global uniforms)
    void beginScene(CameraComponent* camera);

    // Submit an Entity for drawing
    void submitNode(std::shared_ptr<Entity> node);

    // Draw a mesh with a material and model matrix
    void draw(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> material, const glm::mat4& modelMatrix);

    // End the scene rendering
    void endScene();

    // Render debug wireframes for lights and colliders
    void renderDebug(std::shared_ptr<Model> cubeModel);

private:
    std::vector<PointLightData> activeLights;
    std::vector<DrawCommand> renderQueue;

    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;
    glm::vec3 m_viewPos;
};

#endif
