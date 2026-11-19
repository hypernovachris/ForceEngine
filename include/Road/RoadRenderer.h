#ifndef ROAD_RENDERER_H
#define ROAD_RENDERER_H

#include <memory>
#include <glm/glm/glm.hpp>
#include <SDL3/SDL_gpu.h>
#include "Shader.h"
#include "RHI/RHI_Pipeline.h"
#include "Mesh.h"

struct RoadUniforms {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
    glm::vec4 viewPos;
    glm::vec4 roadParams;        // [mode, wireframe, polygonOffset, extra]
    glm::vec4 chunkCenterRelEye; // xyz: (O_chunk - C_ECEF), w: isQuadsphere (1.0 or 0.0)
    glm::vec4 sunDir;            // Directional sun light vector
};

class RoadRenderer {
public:
    RoadRenderer();
    ~RoadRenderer();

    // Initialize the road graphics pipeline with given swapchain and depth formats
    bool init(SDL_GPUTextureFormat colorFormat,
              SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT);

    // Draw arbitrary road mesh (Legacy Cartesian)
    void drawMesh(SDL_GPURenderPass* pass,
                  SDL_GPUCommandBuffer* cmdBuf,
                  const glm::mat4& view,
                  const glm::mat4& projection,
                  const glm::vec3& cameraPos,
                  const Mesh& mesh,
                  const glm::vec4& roadParams = glm::vec4(0.0f),
                  const glm::mat4& model = glm::mat4(1.0f),
                  bool occludedSilhouette = false);

    // Draw road mesh on Quadsphere using Relative-to-Eye (RTE)
    void drawMeshQuadsphere(SDL_GPURenderPass* pass,
                           SDL_GPUCommandBuffer* cmdBuf,
                           const glm::mat4& viewRot,
                           const glm::mat4& projection,
                           const glm::dvec3& camPosECEF,
                           const glm::dvec3& chunkCenterECEF,
                           const Mesh& mesh,
                           const glm::vec4& roadParams = glm::vec4(0.0f),
                           const glm::vec3& sunDir = glm::vec3(0.0f, 1.0f, 0.0f),
                           bool occludedSilhouette = false);

    // Draw with raw GPU buffer bindings
    void drawBuffers(SDL_GPURenderPass* pass,
                     SDL_GPUCommandBuffer* cmdBuf,
                     const glm::mat4& view,
                     const glm::mat4& projection,
                     const glm::vec3& cameraPos,
                     const SDL_GPUBufferBinding& vertexBinding,
                     const SDL_GPUBufferBinding& indexBinding,
                     uint32_t indexCount,
                     const glm::vec4& roadParams = glm::vec4(0.0f),
                     const glm::mat4& model = glm::mat4(1.0f),
                     bool occludedSilhouette = false);

    std::shared_ptr<RHI_Pipeline> getPipeline() const { return m_pipeline; }
    std::shared_ptr<RHI_Pipeline> getOccludedPipeline() const { return m_occludedPipeline; }
    bool isInitialized() const { return m_pipeline != nullptr; }

private:
    std::shared_ptr<Shader> m_shader;
    std::shared_ptr<RHI_Pipeline> m_pipeline;
    std::shared_ptr<RHI_Pipeline> m_occludedPipeline;
};

#endif // ROAD_RENDERER_H

