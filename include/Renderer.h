#ifndef RENDERER_H
#define RENDERER_H

#include <glm/glm/glm.hpp>
#include <memory>
#include "Shader.h"
#include "Model.h"
#include "Material.h"
#include "Camera.h"

class Entity;

class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize OpenGL state
    void init();

    // Clear frame buffers
    void clear();

    // Prepare the scene for rendering (set global uniforms)
    void beginScene(const Camera& camera, const glm::vec3& lightPos, const glm::vec3& lightColor);

    // Submit an Entity for drawing
    void drawNode(std::shared_ptr<Entity> node);

    // Draw a mesh with a material and model matrix
    void draw(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> material, const glm::mat4& modelMatrix);

    // End the scene rendering
    void endScene();

private:
    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;
    glm::vec3 m_viewPos;
    glm::vec3 m_lightPos;
    glm::vec3 m_lightColor;
};

#endif
