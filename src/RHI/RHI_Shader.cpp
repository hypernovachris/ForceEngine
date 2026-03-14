#include "RHI/RHI_Shader.h"
#include <iostream>
#include <fstream>
#include <vector>

RHI_Shader::~RHI_Shader() {
    if (shader) {
        SDL_ReleaseGPUShader(RHI_Device::getInstance().getDevice(), shader);
        shader = nullptr;
    }
}

bool RHI_Shader::loadFromHLSL(const std::string& path, const std::string& entryPoint, SDL_ShaderCross_ShaderStage stage) {
    if (shader) return false;

    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << path << std::endl;
        return false;
    }

    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize + 1);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    buffer[fileSize] = '\0';
    file.close();

    SDL_ShaderCross_HLSL_Info hlslInfo = {};
    hlslInfo.source = buffer.data();
    hlslInfo.entrypoint = entryPoint.c_str();
    hlslInfo.include_dir = nullptr;
    hlslInfo.defines = nullptr;
    hlslInfo.shader_stage = stage;
    hlslInfo.props = 0;

    size_t spirvSize = 0;
    void* spirvBytecode = SDL_ShaderCross_CompileSPIRVFromHLSL(&hlslInfo, &spirvSize);
    if (!spirvBytecode) {
        std::cerr << "Failed to compile HLSL to SPIRV: " << path << "\nSDL Error: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_ShaderCross_GraphicsShaderMetadata* metadata = SDL_ShaderCross_ReflectGraphicsSPIRV((const Uint8*)spirvBytecode, spirvSize, 0);
    if (!metadata) {
        std::cerr << "Failed to reflect SPIRV: " << path << "\nSDL Error: " << SDL_GetError() << std::endl;
        SDL_free(spirvBytecode);
        return false;
    }

    SDL_ShaderCross_SPIRV_Info spirvInfo = {};
    spirvInfo.bytecode = (const Uint8*)spirvBytecode;
    spirvInfo.bytecode_size = spirvSize;
    spirvInfo.entrypoint = entryPoint.c_str();
    spirvInfo.shader_stage = stage;
    spirvInfo.props = 0;

    shader = SDL_ShaderCross_CompileGraphicsShaderFromSPIRV(
        RHI_Device::getInstance().getDevice(),
        &spirvInfo,
        &metadata->resource_info,
        0
    );

    SDL_free(metadata);
    SDL_free(spirvBytecode);

    if (!shader) {
        std::cerr << "Failed to create shader from SPIRV " << path << ": " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}
