#ifndef RHI_SHADER_H
#define RHI_SHADER_H

#include "RHI_Device.h"
#include <SDL3_shadercross/SDL_shadercross.h>
#include <string>

class RHI_Shader {
public:
    RHI_Shader() = default;
    ~RHI_Shader();

    bool loadFromHLSL(const std::string& path, const std::string& entryPoint, SDL_ShaderCross_ShaderStage stage);

    SDL_GPUShader* getShader() const { return shader; }

private:
    SDL_GPUShader* shader = nullptr;
};

#endif
