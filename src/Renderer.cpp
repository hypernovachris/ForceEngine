#include "../include/Renderer.h"
#include "../include/Entity.h"
#include "../include/RendererComponent.h"
#include "../include/LightComponent.h"
#include "../include/ColliderComponent.h"
#include "../include/Model.h"
#include "../include/RHI/RHI_Pipeline.h"
#include <iostream>
#include <algorithm>
#include <glm/glm/gtc/matrix_transform.hpp>

Renderer::Renderer() {}

Renderer::~Renderer() {
    auto device = RHI_Device::getInstance().getDevice();
    if (device) {
        if (shadowAtlasTexture) {
            SDL_ReleaseGPUTexture(device, shadowAtlasTexture);
            shadowAtlasTexture = nullptr;
        }
        if (shadowSampler) {
            SDL_ReleaseGPUSampler(device, shadowSampler);
            shadowSampler = nullptr;
        }
    }
}

void Renderer::init() {
    auto& device = RHI_Device::getInstance();
    if (!device.getDevice()) return;

    // 1. Create Shadow Atlas Texture (4096 x 2048, D32_FLOAT, 8 tiles of 1024x1024)
    if (!shadowAtlasTexture) {
        SDL_GPUTextureCreateInfo shadowAtlasInfo = {};
        shadowAtlasInfo.type = SDL_GPU_TEXTURETYPE_2D;
        shadowAtlasInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
        shadowAtlasInfo.width = 4096;
        shadowAtlasInfo.height = 2048;
        shadowAtlasInfo.layer_count_or_depth = 1;
        shadowAtlasInfo.num_levels = 1;
        shadowAtlasInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
        shadowAtlasInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;

        shadowAtlasTexture = SDL_CreateGPUTexture(device.getDevice(), &shadowAtlasInfo);
        if (!shadowAtlasTexture) {
            std::cerr << "Renderer: Failed to create Shadow Atlas texture: " << SDL_GetError() << std::endl;
        }
    }

    // 2. Create Shadow Sampler (Nearest filtering for depth texture, Clamp to edge)
    if (!shadowSampler) {
        SDL_GPUSamplerCreateInfo samplerInfo = {};
        samplerInfo.min_filter = SDL_GPU_FILTER_NEAREST;
        samplerInfo.mag_filter = SDL_GPU_FILTER_NEAREST;
        samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
        samplerInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        samplerInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;

        shadowSampler = SDL_CreateGPUSampler(device.getDevice(), &samplerInfo);
        if (!shadowSampler) {
            std::cerr << "Renderer: Failed to create Shadow Sampler: " << SDL_GetError() << std::endl;
        }
    }

    // 3. Compile Shadow Shader
    if (!shadowShader) {
        shadowShader = std::make_shared<Shader>("assets/shaders/shadow.hlsl", "assets/shaders/shadow.hlsl");
    }
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
    if (lightComp && activeLights.size() < 8) {
        glm::vec3 worldPos = glm::vec3(node->worldTransform[3]);
        InternalLight il = {};
        il.position = worldPos;
        il.color = lightComp->color;
        il.intensity = lightComp->intensity;
        il.castsShadow = lightComp->castsShadow;
        il.fov = lightComp->fov;
        il.nearPlane = lightComp->nearPlane;
        il.farPlane = lightComp->farPlane;
        activeLights.push_back(il);
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
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;

    pipeline->create(shader->vertexShader.get(), shader->fragmentShader.get(),
                     attributes, 4,
                     binding, 1,
                     colorFormat, depthFormat,
                     enableDepthTest);

    shader->pipeline = pipeline;
    shader->pipelineBaked = true;
    return pipeline;
}

std::shared_ptr<RHI_Pipeline> Renderer::getOrCreateShadowPipeline() {
    if (shadowPipeline) return shadowPipeline;
    if (!shadowShader || !shadowShader->vertexShader || !shadowShader->fragmentShader) return nullptr;

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
    SDL_GPUTextureFormat colorFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;

    if (!pipeline->create(shadowShader->vertexShader.get(), shadowShader->fragmentShader.get(),
                          attributes, 4,
                          binding, 1,
                          colorFormat, depthFormat,
                          true)) {
        std::cerr << "Renderer: Failed to create shadow graphics pipeline: " << SDL_GetError() << std::endl;
        return nullptr;
    }

    shadowPipeline = pipeline;
    return shadowPipeline;
}

void Renderer::endScene() {
    auto& device = RHI_Device::getInstance();
    SDL_GPUTexture* swapchainTexture = nullptr;
    SDL_GPUCommandBuffer* cmdBuf = device.beginFrame(swapchainTexture);

    if (!cmdBuf || !swapchainTexture) return;

    // Safety initialization of shadow resources if not already done
    if (!shadowAtlasTexture) {
        init();
    }

    // 1. Prepare Uniform Data
    GlobalUniforms globalUniforms = {};
    globalUniforms.view = m_viewMatrix;
    globalUniforms.projection = m_projectionMatrix;

    FragmentUniforms fragUniforms = {};
    fragUniforms.viewPos = glm::vec4(m_viewPos, 1.0f);
    fragUniforms.numLights = std::min((int)activeLights.size(), 8);

    for (int i = 0; i < fragUniforms.numLights; ++i) {
        const auto& light = activeLights[i];

        // Downward-facing frustum:
        // Eye: light position
        // Target: light position looking straight down (-Y)
        // Up vector: -Z (mutually orthogonal with -Y, zero gimbal lock)
        glm::mat4 lightView = glm::lookAt(light.position, light.position + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        glm::mat4 lightProj = glm::perspectiveRH_ZO(glm::radians(light.fov), 1.0f, light.nearPlane, light.farPlane);
        glm::mat4 lightSpaceMatrix = lightProj * lightView;

        int col = i % 4;
        int row = i / 4;
        glm::vec4 shadowTile = glm::vec4(col * 0.25f, row * 0.5f, 0.25f, 0.5f);

        fragUniforms.lights[i].lightSpaceMatrix = lightSpaceMatrix;
        fragUniforms.lights[i].position = glm::vec4(light.position, 1.0f);
        fragUniforms.lights[i].color = glm::vec4(light.color, 1.0f);
        fragUniforms.lights[i].shadowTile = shadowTile;
        fragUniforms.lights[i].intensity = light.intensity;
        fragUniforms.lights[i].castsShadow = light.castsShadow ? 1.0f : 0.0f;
    }

    // 2. PASS 1: Multi-Light Shadow Atlas Pass (Depth-only)
    if (shadowAtlasTexture) {
        SDL_GPUDepthStencilTargetInfo shadowDepthTarget = {};
        shadowDepthTarget.texture = shadowAtlasTexture;
        shadowDepthTarget.cycle = false;
        shadowDepthTarget.clear_depth = 1.0f;
        shadowDepthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
        shadowDepthTarget.store_op = SDL_GPU_STOREOP_STORE;
        shadowDepthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        shadowDepthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

        SDL_GPURenderPass* shadowPass = SDL_BeginGPURenderPass(cmdBuf, nullptr, 0, &shadowDepthTarget);
        if (shadowPass) {
            auto shadowPipe = getOrCreateShadowPipeline();
            if (shadowPipe && shadowPipe->getPipeline()) {
                SDL_BindGPUGraphicsPipeline(shadowPass, shadowPipe->getPipeline());

                for (int i = 0; i < fragUniforms.numLights; ++i) {
                    if (!activeLights[i].castsShadow) continue;

                    int col = i % 4;
                    int row = i / 4;

                    SDL_GPUViewport vp = {};
                    vp.x = (float)(col * 1024);
                    vp.y = (float)(row * 1024);
                    vp.w = 1024.0f;
                    vp.h = 1024.0f;
                    vp.min_depth = 0.0f;
                    vp.max_depth = 1.0f;
                    SDL_SetGPUViewport(shadowPass, &vp);

                    SDL_Rect sc = {};
                    sc.x = col * 1024;
                    sc.y = row * 1024;
                    sc.w = 1024;
                    sc.h = 1024;
                    SDL_SetGPUScissor(shadowPass, &sc);

                    glm::mat4 lsm = fragUniforms.lights[i].lightSpaceMatrix;

                    for (const auto& cmd : renderQueue) {
                        if (!cmd.mesh) continue;
                        if (cmd.material && !cmd.material->castsShadow) continue;

                        ShadowConstants scConstants;
                        scConstants.lightSpaceMatrix = lsm;
                        scConstants.model = cmd.transform;
                        SDL_PushGPUVertexUniformData(cmdBuf, 0, &scConstants, sizeof(ShadowConstants));

                        SDL_GPUBufferBinding vBinding = {};
                        vBinding.buffer = cmd.mesh->vertexBuffer.getBuffer();
                        vBinding.offset = 0;
                        SDL_BindGPUVertexBuffers(shadowPass, 0, &vBinding, 1);

                        SDL_GPUBufferBinding iBinding = {};
                        iBinding.buffer = cmd.mesh->indexBuffer.getBuffer();
                        iBinding.offset = 0;
                        SDL_BindGPUIndexBuffer(shadowPass, &iBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

                        SDL_DrawGPUIndexedPrimitives(shadowPass, cmd.mesh->indexCount, 1, 0, 0, 0);
                    }
                }
            }
            SDL_EndGPURenderPass(shadowPass);
        }
    }

    // 3. PASS 2: Forward Color Pass
    // Depth attachment for main 3D scene
    SDL_GPUTextureCreateInfo depthInfo = {};
    depthInfo.type = SDL_GPU_TEXTURETYPE_2D;
    depthInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    int w = 800, h = 600;
    if (device.getWindow()) {
        SDL_GetWindowSizeInPixels(device.getWindow(), &w, &h);
    }
    depthInfo.width = w;
    depthInfo.height = h;
    depthInfo.layer_count_or_depth = 1;
    depthInfo.num_levels = 1;
    depthInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    depthInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    SDL_GPUTexture* depthTexture = SDL_CreateGPUTexture(device.getDevice(), &depthInfo);

    SDL_GPUColorTargetInfo colorTarget = {};
    colorTarget.texture = swapchainTexture;
    colorTarget.clear_color = {0.05f, 0.05f, 0.07f, 1.0f};
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

    // 4. Draw Opaque Scene
    for (const auto& cmd : renderQueue) {
        if (!cmd.mesh || !cmd.material) continue;

        auto pipeline = getOrCreatePipeline(cmd.material->shader);
        if (!pipeline) continue;

        SDL_BindGPUGraphicsPipeline(renderPass, pipeline->getPipeline());

        cmd.material->updateData();

        // Push globals (vertex uniforms)
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

        // Bind Textures:
        // Slot 0: Diffuse Map
        // Slot 1: Specular Map
        // Slot 2: Normal Map
        // Slot 3: Shadow Atlas Texture
        SDL_GPUTextureSamplerBinding samplers[4] = {};
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

            samplers[3].texture = shadowAtlasTexture;
            samplers[3].sampler = shadowSampler;

            SDL_BindGPUFragmentSamplers(renderPass, 0, samplers, 4);
        }
        
        // Bind vertex and index buffers
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