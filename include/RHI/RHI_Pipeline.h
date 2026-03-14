#ifndef RHI_PIPELINE_H
#define RHI_PIPELINE_H

#include "RHI_Device.h"
#include "RHI_Shader.h"
#include <SDL3/SDL_gpu.h>

class RHI_Pipeline {
public:
    RHI_Pipeline() = default;
    ~RHI_Pipeline();

    bool create(RHI_Shader* vertexShader, RHI_Shader* fragmentShader, 
                const SDL_GPUVertexAttribute* attributes, Uint32 numAttributes,
                const SDL_GPUVertexBufferDescription* bindings, Uint32 numBindings,
                SDL_GPUTextureFormat colorFormat, SDL_GPUTextureFormat depthFormat,
                bool enableDepthTest = true);

    SDL_GPUGraphicsPipeline* getPipeline() const { return pipeline; }

private:
    SDL_GPUGraphicsPipeline* pipeline = nullptr;
};

#endif
