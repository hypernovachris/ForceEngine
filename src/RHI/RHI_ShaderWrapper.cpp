#include "Shader.h"
#include <iostream>

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
    vertexShader = std::make_shared<RHI_Shader>();
    fragmentShader = std::make_shared<RHI_Shader>();

    std::string hlslVertexPath = vertexPath;
    std::string hlslFragmentPath = fragmentPath;
    
    // Replace .vs and .metal extension with .hlsl if present
    size_t extPosV = hlslVertexPath.find(".vs");
    if (extPosV != std::string::npos) hlslVertexPath.replace(extPosV, 3, ".hlsl");
    extPosV = hlslVertexPath.find(".metal");
    if (extPosV != std::string::npos) hlslVertexPath.replace(extPosV, 6, ".hlsl");
    
    // Replace .fs and .metal extension with .hlsl if present
    size_t extPosF = hlslFragmentPath.find(".fs");
    if (extPosF != std::string::npos) hlslFragmentPath.replace(extPosF, 3, ".hlsl");
    extPosF = hlslFragmentPath.find(".metal");
    if (extPosF != std::string::npos) hlslFragmentPath.replace(extPosF, 6, ".hlsl");

    if (!vertexShader->loadFromHLSL(hlslVertexPath, "vs_main", SDL_SHADERCROSS_SHADERSTAGE_VERTEX)) {
        std::cerr << "Failed to load vertex shader: " << hlslVertexPath << std::endl;
    }

    if (!fragmentShader->loadFromHLSL(hlslFragmentPath, "fs_main", SDL_SHADERCROSS_SHADERSTAGE_FRAGMENT)) {
        std::cerr << "Failed to load fragment shader: " << hlslFragmentPath << std::endl;
    }
}
