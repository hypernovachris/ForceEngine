#include "RHI/RHI_Pipeline.h"
#include <iostream>

RHI_Pipeline::~RHI_Pipeline() {
    if (pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(RHI_Device::getInstance().getDevice(), pipeline);
        pipeline = nullptr;
    }
}

bool RHI_Pipeline::create(RHI_Shader* vertexShader, RHI_Shader* fragmentShader, 
                          const SDL_GPUVertexAttribute* attributes, Uint32 numAttributes,
                          const SDL_GPUVertexBufferDescription* bindings, Uint32 numBindings,
                          SDL_GPUTextureFormat colorFormat, SDL_GPUTextureFormat depthFormat,
                          bool enableDepthTest) {
    if (pipeline) return false;
    if (!vertexShader || !fragmentShader) return false;

    SDL_GPUGraphicsPipelineCreateInfo pipelineInfo = {};
    
    // Shaders
    pipelineInfo.vertex_shader = vertexShader->getShader();
    pipelineInfo.fragment_shader = fragmentShader->getShader();

    // Vertex State
    pipelineInfo.vertex_input_state.vertex_attributes = attributes;
    pipelineInfo.vertex_input_state.num_vertex_attributes = numAttributes;
    pipelineInfo.vertex_input_state.vertex_buffer_descriptions = bindings;
    pipelineInfo.vertex_input_state.num_vertex_buffers = numBindings;

    // Output State (Formats)
    SDL_GPUColorTargetDescription colorTargetDesc = {};
    colorTargetDesc.format = colorFormat;
    
    pipelineInfo.target_info.color_target_descriptions = &colorTargetDesc;
    pipelineInfo.target_info.num_color_targets = 1;
    pipelineInfo.target_info.depth_stencil_format = depthFormat;
    pipelineInfo.target_info.has_depth_stencil_target = depthFormat != SDL_GPU_TEXTUREFORMAT_INVALID;

    // Rasterization (Triangles, CCW, Front face cull for basic behavior or None)
    pipelineInfo.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE; // Can change later
    pipelineInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    pipelineInfo.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;

    // Depth Stencil State
    pipelineInfo.depth_stencil_state.enable_depth_test = enableDepthTest;
    pipelineInfo.depth_stencil_state.enable_depth_write = enableDepthTest;
    pipelineInfo.depth_stencil_state.compare_op = enableDepthTest ? SDL_GPU_COMPAREOP_LESS_OR_EQUAL : SDL_GPU_COMPAREOP_ALWAYS;

    // Primitive Type
    pipelineInfo.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;

    pipeline = SDL_CreateGPUGraphicsPipeline(RHI_Device::getInstance().getDevice(), &pipelineInfo);
    if (!pipeline) {
        std::cerr << "Failed to create GPU Graphics Pipeline: " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}
