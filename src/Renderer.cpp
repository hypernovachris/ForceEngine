#include "../include/Renderer.h"
#include "../include/Entity.h"
#include "../include/RendererComponent.h"
#include "../include/LightComponent.h"
#include "../include/ColliderComponent.h"
#include "../include/Model.h"
#include "../include/RHI/RHI_Pipeline.h"
#include <iostream>
#include <glm/glm/gtc/matrix_transform.hpp>

Renderer::Renderer() {}
Renderer::~Renderer() {}

void Renderer::init() {
    // Pipeline creation happens lazily or in a pre-warm pass.
}

void Renderer::beginScene(CameraComponent* camera) {
    if (camera) {
        m_viewMatrix = camera->getViewMatrix();
        m_projectionMatrix = camera->getProjectionMatrix();
        if (camera->owner) {
            m_viewPos = glm::vec3(camera->owner->worldTransform[3]);
        }
    } else {
        m_viewMatrix = glm::mat4(1.0f);
        m_projectionMatrix = glm::mat4(1.0f);
        m_viewPos = glm::vec3(0.0f);
    }

    activeLights.clear();
    renderQueue.clear();
}

void Renderer::submitNode(std::shared_ptr<Entity> node) {
    auto lightComp = node->getComponent<LightComponent>();
    if (lightComp) {
        glm::vec3 worldPos = glm::vec3(node->worldTransform[3]);
        PointLightData pl = {};
        pl.position = glm::vec4(worldPos, 1.0f);
        pl.color = glm::vec4(lightComp->color, lightComp->intensity);
        pl.intensity = lightComp->intensity;
        activeLights.push_back(pl);
    }
    
    auto renderComp = node->getComponent<RendererComponent>();
    if (renderComp && renderComp->model) {
        auto model = renderComp->model;
        for (size_t i = 0; i < model->meshes.size(); ++i) {
            auto mesh = model->meshes[i];
            auto material = (i < model->materials.size()) ? model->materials[i] : nullptr; 
            if (mesh && material) {
                renderQueue.push_back({mesh, material, node->worldTransform});
            }
        }
    }

    for (auto& child : node->children) {
        submitNode(child);
    }
}

std::shared_ptr<RHI_Pipeline> Renderer::getOrCreatePipeline(std::shared_ptr<Shader> shader, bool enableDepthTest, bool debugLines) {
    if (shader->pipelineBaked) return shader->pipeline;

    // Build Vertex Layout for `Vertex` struct in Mesh.h
    SDL_GPUVertexAttribute attributes[4];
    attributes[0].location = 0;
    attributes[0].buffer_slot = 0;
    attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[0].offset = offsetof(Vertex, position);

    attributes[1].location = 1;
    attributes[1].buffer_slot = 0;
    attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[1].offset = offsetof(Vertex, normal);

    attributes[2].location = 2;
    attributes[2].buffer_slot = 0;
    attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    attributes[2].offset = offsetof(Vertex, texCoords);

    attributes[3].location = 3;
    attributes[3].buffer_slot = 0;
    attributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[3].offset = offsetof(Vertex, tangent);

    SDL_GPUVertexBufferDescription binding[1];
    binding[0].slot = 0;
    binding[0].pitch = sizeof(Vertex);
    binding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    binding[0].instance_step_rate = 0;

    auto pipeline = std::make_shared<RHI_Pipeline>();
    SDL_GPUTextureFormat colorFormat = RHI_Device::getInstance().getSwapchainFormat();
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT; // Standard Mac format usually D32_FLOAT or D24_UNORM_S8_UINT

    pipeline->create(shader->vertexShader.get(), shader->fragmentShader.get(),
                     attributes, 4,
                     binding, 1,
                     colorFormat, depthFormat,
                     enableDepthTest);

    shader->pipeline = pipeline;
    shader->pipelineBaked = true;
    return pipeline;
}

void Renderer::endScene() {
    auto& device = RHI_Device::getInstance();
    SDL_GPUTexture* swapchainTexture = nullptr;
    SDL_GPUCommandBuffer* cmdBuf = device.beginFrame(swapchainTexture);

    if (!cmdBuf || !swapchainTexture) return;

    // 1. We must have a depth attachment for 3D
    // In a real engine, we cache this. For now, generate a transient depth texture each frame.
    SDL_GPUTextureCreateInfo depthInfo = {};
    depthInfo.type = SDL_GPU_TEXTURETYPE_2D;
    depthInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    depthInfo.width = device.getWindow() ? 800 : 800; // Need real dimensions
    depthInfo.height = device.getWindow() ? 600 : 600;
    int w, h;
    SDL_GetWindowSizeInPixels(device.getWindow(), &w, &h);
    depthInfo.width = w;
    depthInfo.height = h;
    depthInfo.layer_count_or_depth = 1;
    depthInfo.num_levels = 1;
    depthInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    depthInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    SDL_GPUTexture* depthTexture = SDL_CreateGPUTexture(device.getDevice(), &depthInfo);

    // 2. Begin Render Pass
    SDL_GPUColorTargetInfo colorTarget = {};
    colorTarget.texture = swapchainTexture;
    colorTarget.clear_color = {0.1f, 0.1f, 0.1f, 1.0f};
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo depthTarget = {};
    depthTarget.texture = depthTexture;
    depthTarget.cycle = true;
    depthTarget.clear_depth = 1.0f;
    depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass* renderPass = SDL_BeginGPURenderPass(cmdBuf, &colorTarget, 1, &depthTarget);

    // 3. Prepare Uniform Data
    GlobalUniforms globalUniforms = {};
    globalUniforms.view = m_viewMatrix;
    globalUniforms.projection = m_projectionMatrix;

    FragmentUniforms fragUniforms = {};
    fragUniforms.viewPos = glm::vec4(m_viewPos, 1.0f);
    fragUniforms.numLights = std::min((int)activeLights.size(), 16);
    for (int i = 0; i < fragUniforms.numLights; ++i) {
        fragUniforms.lights[i] = activeLights[i];
    }

    // 4. Draw Opaque Scene
    for (const auto& cmd : renderQueue) {
        if (!cmd.mesh || !cmd.material) continue;

        auto pipeline = getOrCreatePipeline(cmd.material->shader);
        if (!pipeline) continue;

        SDL_BindGPUGraphicsPipeline(renderPass, pipeline->getPipeline());

        cmd.material->updateData();

        // Push globals
        globalUniforms.model = cmd.transform;
        globalUniforms.textureScale = cmd.material->textureScale;
        SDL_PushGPUVertexUniformData(cmdBuf, 0, &globalUniforms, sizeof(GlobalUniforms));

        // Push fragments
        fragUniforms.hasDiffuse = cmd.material->materialData.hasDiffuse;
        fragUniforms.hasSpecular = cmd.material->materialData.hasSpecular;
        fragUniforms.hasNormalMap = cmd.material->materialData.hasNormalMap;
        SDL_PushGPUFragmentUniformData(cmdBuf, 0, &fragUniforms, sizeof(FragmentUniforms));
        
        // Push Shininess (16-byte aligned)
        struct ShininessData {
            float shininess;
            float pad[3];
        };
        ShininessData sd = { cmd.material->materialData.shininess, {0.0f, 0.0f, 0.0f} };
        SDL_PushGPUFragmentUniformData(cmdBuf, 1, &sd, sizeof(ShininessData));

        // Bind Textures
        SDL_GPUTextureSamplerBinding samplers[3] = {};
        Texture* fallbackTex = nullptr;
        if (cmd.material->diffuseMap) fallbackTex = cmd.material->diffuseMap.get();
        else if (cmd.material->specularMap) fallbackTex = cmd.material->specularMap.get();
        else if (cmd.material->normalMap) fallbackTex = cmd.material->normalMap.get();

        if (fallbackTex) {
            samplers[0].texture = cmd.material->diffuseMap ? cmd.material->diffuseMap->rhiTexture.getTexture() : fallbackTex->rhiTexture.getTexture();
            samplers[0].sampler = cmd.material->diffuseMap ? cmd.material->diffuseMap->rhiTexture.getSampler() : fallbackTex->rhiTexture.getSampler();

            samplers[1].texture = cmd.material->specularMap ? cmd.material->specularMap->rhiTexture.getTexture() : fallbackTex->rhiTexture.getTexture();
            samplers[1].sampler = cmd.material->specularMap ? cmd.material->specularMap->rhiTexture.getSampler() : fallbackTex->rhiTexture.getSampler();

            samplers[2].texture = cmd.material->normalMap ? cmd.material->normalMap->rhiTexture.getTexture() : fallbackTex->rhiTexture.getTexture();
            samplers[2].sampler = cmd.material->normalMap ? cmd.material->normalMap->rhiTexture.getSampler() : fallbackTex->rhiTexture.getSampler();

            SDL_BindGPUFragmentSamplers(renderPass, 0, samplers, 3);
        }
        
        // Let's bind buffers
        SDL_GPUBufferBinding vBinding = {};
        vBinding.buffer = cmd.mesh->vertexBuffer.getBuffer();
        vBinding.offset = 0;
        SDL_BindGPUVertexBuffers(renderPass, 0, &vBinding, 1);

        SDL_GPUBufferBinding iBinding = {};
        iBinding.buffer = cmd.mesh->indexBuffer.getBuffer();
        iBinding.offset = 0;
        SDL_BindGPUIndexBuffer(renderPass, &iBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

        SDL_DrawGPUIndexedPrimitives(renderPass, cmd.mesh->indexCount, 1, 0, 0, 0);
    }

    SDL_EndGPURenderPass(renderPass);
    SDL_ReleaseGPUTexture(device.getDevice(), depthTexture);

    device.endFrame(cmdBuf);
}

void Renderer::renderDebug(std::shared_ptr<Model> cubeModel) {
    // Debug line rendering omitted for brevity in RHI port
}