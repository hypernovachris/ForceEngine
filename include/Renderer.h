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

// 16-byte aligned for GPU uniform buffers (128 bytes per light)
struct PointLightData {
    glm::mat4 lightSpaceMatrix; // 64 bytes
    glm::vec4 position;         // 16 bytes
    glm::vec4 color;            // 16 bytes
    glm::vec4 shadowTile;       // 16 bytes: uOffset, vOffset, uScale, vScale in shadow atlas
    float intensity;            // 4 bytes
    float castsShadow;          // 4 bytes: 1.0f if true, 0.0f otherwise
    float pad[2];               // 8 bytes padding
};

struct DrawCommand {
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Material> material;
    glm::mat4 transform;
};

// Global uniforms sent once per frame to binding 0
struct GlobalUniforms {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
    glm::vec2 textureScale;
    float pad[2];
};

struct FragmentUniforms {
    glm::vec4 viewPos;
    int numLights;
    int pad[3];
    PointLightData lights[8]; // Up to 8 lights
    float hasDiffuse;
    float hasSpecular;
    float hasNormalMap;
    float pad2;
};

struct ShadowConstants {
    glm::mat4 lightSpaceMatrix;
    glm::mat4 model;
};

struct InternalLight {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
    bool castsShadow;
    float fov;
    float nearPlane;
    float farPlane;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize SDL3_GPU pipelines and shadow resources
    void init();

    // Prepare the scene for rendering (reset queues)
    void beginScene(CameraComponent* camera);

    // Submit an Entity for drawing
    void submitNode(std::shared_ptr<Entity> node);

    // Bake and record graphics commands (Shadow Atlas pass + Forward Color pass)
    void endScene();

    // Render debug wireframes for lights and colliders
    void renderDebug(std::shared_ptr<Model> cubeModel);

private:
    std::vector<InternalLight> activeLights;
    std::vector<DrawCommand> renderQueue;
    
    // Per frame camera data
    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;
    glm::vec3 m_viewPos;

    // Shadow Atlas Resources (4096 x 2048, 8 tiles of 1024 x 1024)
    SDL_GPUTexture* shadowAtlasTexture = nullptr;
    SDL_GPUSampler* shadowSampler = nullptr;
    std::shared_ptr<Shader> shadowShader;
    std::shared_ptr<RHI_Pipeline> shadowPipeline;

    // RHI specific caching
    std::shared_ptr<RHI_Pipeline> getOrCreatePipeline(std::shared_ptr<Shader> shader, bool enableDepthTest = true, bool debugLines = false);
    std::shared_ptr<RHI_Pipeline> getOrCreateShadowPipeline();
};

#endif // RENDERER_H
