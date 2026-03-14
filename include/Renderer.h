#ifndef RENDERER_H
#define RENDERER_H

#include "glm/glm/glm.hpp"
#include <memory>
#include <vector>
#include "Shader.h"
#include "Model.h"
#include "Material.h"
#include "CameraComponent.h"
#include "RHI/RHI_Device.h"

class Entity;

// 16-byte aligned for GPU uniform buffers
struct PointLightData {
    glm::vec4 position; // w is unused or for padding
    glm::vec4 color;    // w is intensity
    float intensity;
    float pad[3];
};

struct DrawCommand {
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Material> material;
    glm::mat4 transform;
};

// Global uniforms sent once per frame to binding 1
struct GlobalUniforms {
    glm::mat4 model; // Updated per draw though
    glm::mat4 view;
    glm::mat4 projection;
    glm::vec2 textureScale;
    float pad[2];
};

struct FragmentUniforms {
    glm::vec4 viewPos;
    int numLights;
    int pad[3];
    PointLightData lights[16];
    float hasDiffuse;
    float hasSpecular;
    float hasNormalMap;
    float pad2;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize SDL3_GPU pipelines
    void init();

    // Prepare the scene for rendering (reset queues)
    void beginScene(CameraComponent* camera);

    // Submit an Entity for drawing
    void submitNode(std::shared_ptr<Entity> node);

    // Bake and record graphics commands
    void endScene();

    // Render debug wireframes for lights and colliders
    void renderDebug(std::shared_ptr<Model> cubeModel);

private:
    std::vector<PointLightData> activeLights;
    std::vector<DrawCommand> renderQueue;
    
    // Per frame data
    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;
    glm::vec3 m_viewPos;

    // RHI specific caching
    std::shared_ptr<RHI_Pipeline> getOrCreatePipeline(std::shared_ptr<Shader> shader, bool enableDepthTest = true, bool debugLines = false);

    // Uniform Buffers (we will use push constants or transfer buffers per frame for simplicity)
};

#endif
