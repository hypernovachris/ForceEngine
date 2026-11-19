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
                          bool enableDepthTest, bool enableBlend,
                          bool enableDepthWrite, SDL_GPUCompareOp compareOp) {
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
    if (enableBlend) {
        colorTargetDesc.blend_state.enable_blend = true;
        colorTargetDesc.blend_state.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        colorTargetDesc.blend_state.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        colorTargetDesc.blend_state.color_blend_op = SDL_GPU_BLENDOP_ADD;
        colorTargetDesc.blend_state.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        colorTargetDesc.blend_state.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        colorTargetDesc.blend_state.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
    }
    
    pipelineInfo.target_info.color_target_descriptions = (colorFormat != SDL_GPU_TEXTUREFORMAT_INVALID) ? &colorTargetDesc : nullptr;
    pipelineInfo.target_info.num_color_targets = (colorFormat != SDL_GPU_TEXTUREFORMAT_INVALID) ? 1 : 0;
    pipelineInfo.target_info.depth_stencil_format = depthFormat;
    pipelineInfo.target_info.has_depth_stencil_target = depthFormat != SDL_GPU_TEXTUREFORMAT_INVALID;

    // Rasterization (Triangles, CCW, Front face cull for basic behavior or None)
    pipelineInfo.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE; // Can change later
    pipelineInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    pipelineInfo.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;

    // Depth Stencil State
    pipelineInfo.depth_stencil_state.enable_depth_test = enableDepthTest;
    pipelineInfo.depth_stencil_state.enable_depth_write = enableDepthTest && enableDepthWrite;
    pipelineInfo.depth_stencil_state.compare_op = enableDepthTest ? compareOp : SDL_GPU_COMPAREOP_ALWAYS;

    // Primitive Type
    pipelineInfo.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;

    pipeline = SDL_CreateGPUGraphicsPipeline(RHI_Device::getInstance().getDevice(), &pipelineInfo);
    if (!pipeline) {
        std::cerr << "Failed to create GPU Graphics Pipeline: " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}
