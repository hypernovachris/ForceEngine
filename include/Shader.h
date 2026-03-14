#ifndef SHADER_H
#define SHADER_H

#include "RHI/RHI_Shader.h"
#include "RHI/RHI_Pipeline.h"
#include <SDL3/SDL.h>
#include <string>
#include <memory>

class Shader {
public:
    std::shared_ptr<RHI_Shader> vertexShader;
    std::shared_ptr<RHI_Shader> fragmentShader;
    std::shared_ptr<RHI_Pipeline> pipeline;
    bool pipelineBaked = false;

    Shader() = default;
    
    // In our new architecture, compiling a shader means loading the .metallib files
    Shader(const char* vertexPath, const char* fragmentPath);
    
    // We will bake the pipeline in the Renderer since it requires knowing the render target formats
    // and vertex layouts.
};

#endif