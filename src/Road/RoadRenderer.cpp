#include "Road/RoadRenderer.h"
#include "RHI/RHI_Device.h"
#include <iostream>

RoadRenderer::RoadRenderer() = default;
RoadRenderer::~RoadRenderer() = default;

bool RoadRenderer::init(SDL_GPUTextureFormat colorFormat, SDL_GPUTextureFormat depthFormat) {
    m_shader = std::make_shared<Shader>("assets/shaders/road.hlsl", "assets/shaders/road.hlsl");
    if (!m_shader || !m_shader->vertexShader || !m_shader->fragmentShader) {
        std::cerr << "RoadRenderer: Failed to load road shaders from assets/shaders/road.hlsl" << std::endl;
        return false;
    }

    SDL_GPUVertexAttribute roadAttributes[4] = {};
    roadAttributes[0].location = 0;
    roadAttributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    roadAttributes[0].offset = offsetof(Vertex, position);

    roadAttributes[1].location = 1;
    roadAttributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    roadAttributes[1].offset = offsetof(Vertex, normal);

    roadAttributes[2].location = 2;
    roadAttributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    roadAttributes[2].offset = offsetof(Vertex, texCoords);

    roadAttributes[3].location = 3;
    roadAttributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    roadAttributes[3].offset = offsetof(Vertex, tangent);

    SDL_GPUVertexBufferDescription roadBinding[1] = {};
    roadBinding[0].slot = 0;
    roadBinding[0].pitch = sizeof(Vertex);
    roadBinding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

    m_pipeline = std::make_shared<RHI_Pipeline>();
    if (!m_pipeline->create(m_shader->vertexShader.get(),
                            m_shader->fragmentShader.get(),
                            roadAttributes, 4,
                            roadBinding, 1,
                            colorFormat,
                            depthFormat,
                            true /* depthTest */)) {
        std::cerr << "RoadRenderer: Failed to create road graphics pipeline" << std::endl;
        m_pipeline.reset();
        return false;
    }

    // Create occluded silhouette pipeline (X-Ray: renders occluded parts darker)
    m_occludedPipeline = std::make_shared<RHI_Pipeline>();
    if (!m_occludedPipeline->create(m_shader->vertexShader.get(),
                                    m_shader->fragmentShader.get(),
                                    roadAttributes, 4,
                                    roadBinding, 1,
                                    colorFormat,
                                    depthFormat,
                                    true /* enableDepthTest */,
                                    true /* enableBlend */,
                                    false /* enableDepthWrite */,
                                    SDL_GPU_COMPAREOP_GREATER)) {
        std::cerr << "RoadRenderer: Failed to create occluded road graphics pipeline" << std::endl;
        m_occludedPipeline.reset();
    }

    return true;
}

void RoadRenderer::drawMesh(SDL_GPURenderPass* pass,
                           SDL_GPUCommandBuffer* cmdBuf,
                           const glm::mat4& view,
                           const glm::mat4& projection,
                           const glm::vec3& cameraPos,
                           const Mesh& mesh,
                           const glm::vec4& roadParams,
                           const glm::mat4& model,
                           bool occludedSilhouette) {
    if (!m_pipeline || mesh.indexCount == 0) return;

    SDL_GPUBufferBinding vb = { mesh.vertexBuffer.getBuffer(), 0 };
    SDL_GPUBufferBinding ib = { mesh.indexBuffer.getBuffer(), 0 };
    drawBuffers(pass, cmdBuf, view, projection, cameraPos, vb, ib, mesh.indexCount, roadParams, model, occludedSilhouette);
}

void RoadRenderer::drawMeshQuadsphere(SDL_GPURenderPass* pass,
                                     SDL_GPUCommandBuffer* cmdBuf,
                                     const glm::mat4& viewRot,
                                     const glm::mat4& projection,
                                     const glm::dvec3& camPosECEF,
                                     const glm::dvec3& chunkCenterECEF,
                                     const Mesh& mesh,
                                     const glm::vec4& roadParams,
                                     const glm::vec3& sunDir,
                                     bool occludedSilhouette) {
    if (!m_pipeline || !pass || !cmdBuf || mesh.indexCount == 0) return;

    RoadUniforms ru = {};
    ru.model = glm::mat4(1.0f);
    ru.view = viewRot;
    ru.projection = projection;
    ru.viewPos = glm::vec4(0.0f); // In RTE, eye is at origin
    ru.roadParams = roadParams;

    glm::vec3 relCenter = glm::vec3(chunkCenterECEF - camPosECEF);
    ru.chunkCenterRelEye = glm::vec4(relCenter, 1.0f);
    ru.sunDir = glm::vec4(sunDir, 0.0f);

    SDL_GPUBufferBinding vb = { mesh.vertexBuffer.getBuffer(), 0 };
    SDL_GPUBufferBinding ib = { mesh.indexBuffer.getBuffer(), 0 };

    // 1. Pass 1: Occluded silhouette (Mario behind objects effect)
    if (occludedSilhouette && m_occludedPipeline) {
        SDL_BindGPUGraphicsPipeline(pass, m_occludedPipeline->getPipeline());
        RoadUniforms ruOcc = ru;
        ruOcc.roadParams.z = 1.0f; // Flag occluded pass in shader
        SDL_PushGPUVertexUniformData(cmdBuf, 0, &ruOcc, sizeof(ruOcc));
        SDL_PushGPUFragmentUniformData(cmdBuf, 0, &ruOcc, sizeof(ruOcc));
        SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
        SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);
        SDL_DrawGPUIndexedPrimitives(pass, mesh.indexCount, 1, 0, 0, 0);
    }

    // 2. Pass 2: Visible normal road pass
    SDL_BindGPUGraphicsPipeline(pass, m_pipeline->getPipeline());
    ru.roadParams.z = 0.0f;
    SDL_PushGPUVertexUniformData(cmdBuf, 0, &ru, sizeof(ru));
    SDL_PushGPUFragmentUniformData(cmdBuf, 0, &ru, sizeof(ru));
    SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
    SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_DrawGPUIndexedPrimitives(pass, mesh.indexCount, 1, 0, 0, 0);
}

void RoadRenderer::drawBuffers(SDL_GPURenderPass* pass,
                              SDL_GPUCommandBuffer* cmdBuf,
                              const glm::mat4& view,
                              const glm::mat4& projection,
                              const glm::vec3& cameraPos,
                              const SDL_GPUBufferBinding& vertexBinding,
                              const SDL_GPUBufferBinding& indexBinding,
                              uint32_t indexCount,
                              const glm::vec4& roadParams,
                              const glm::mat4& model,
                              bool occludedSilhouette) {
    if (!m_pipeline || !pass || !cmdBuf || indexCount == 0) return;

    RoadUniforms ru = {};
    ru.model = model;
    ru.view = view;
    ru.projection = projection;
    ru.viewPos = glm::vec4(cameraPos, 1.0f);
    ru.roadParams = roadParams;
    ru.chunkCenterRelEye = glm::vec4(0.0f);
    ru.sunDir = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);

    // 1. Pass 1: Occluded silhouette (Mario behind objects effect)
    if (occludedSilhouette && m_occludedPipeline) {
        SDL_BindGPUGraphicsPipeline(pass, m_occludedPipeline->getPipeline());
        RoadUniforms ruOcc = ru;
        ruOcc.roadParams.z = 1.0f; // Flag occluded pass in shader
        SDL_PushGPUVertexUniformData(cmdBuf, 0, &ruOcc, sizeof(ruOcc));
        SDL_PushGPUFragmentUniformData(cmdBuf, 0, &ruOcc, sizeof(ruOcc));
        SDL_BindGPUVertexBuffers(pass, 0, &vertexBinding, 1);
        SDL_BindGPUIndexBuffer(pass, &indexBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);
        SDL_DrawGPUIndexedPrimitives(pass, indexCount, 1, 0, 0, 0);
    }

    // 2. Pass 2: Visible normal road pass
    SDL_BindGPUGraphicsPipeline(pass, m_pipeline->getPipeline());
    ru.roadParams.z = 0.0f;
    SDL_PushGPUVertexUniformData(cmdBuf, 0, &ru, sizeof(ru));
    SDL_PushGPUFragmentUniformData(cmdBuf, 0, &ru, sizeof(ru));
    SDL_BindGPUVertexBuffers(pass, 0, &vertexBinding, 1);
    SDL_BindGPUIndexBuffer(pass, &indexBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_DrawGPUIndexedPrimitives(pass, indexCount, 1, 0, 0, 0);
}

