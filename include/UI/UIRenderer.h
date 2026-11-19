#ifndef UI_RENDERER_H
#define UI_RENDERER_H

#include <vector>
#include <string>
#include <memory>
#include <glm/glm/glm.hpp>
#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include "RHI/RHI_Pipeline.h"
#include "RHI/RHI_Shader.h"
#include "RHI/RHI_Buffer.h"

struct UIVertex {
    float x, y;
    float u, v;
    float r, g, b, a;
};

class UI {
public:
    static bool init();
    static void shutdown();

    static void beginFrame(float screenW, float screenH,
                           float mouseX, float mouseY,
                           bool mouseDown, bool mouseClicked,
                           float wheelDelta = 0.0f);
    static void endFrame();
    static void render(SDL_GPUCommandBuffer* cmdBuf, SDL_GPUTexture* swapchainTexture);

    static bool wantsMouseInput();

    // Widget API
    static void panel(const std::string& title, float x, float y, float w, float h,
                      glm::vec4 bgColor = glm::vec4(0.12f, 0.13f, 0.16f, 0.94f));
    static bool button(const std::string& text, float x, float y, float w, float h, bool active = false);
    static bool toggle(const std::string& text, bool& value, float x, float y, float w = 220.0f, float h = 24.0f);
    static bool sliderFloat(const std::string& label, float& value, float minVal, float maxVal,
                            float x, float y, float w, float h);
    static void label(const std::string& text, float x, float y, float scale = 1.0f,
                      glm::vec4 color = glm::vec4(0.92f, 0.92f, 0.94f, 1.0f));
    static void separator(float x, float y, float w);

    // Primitives
    static void drawRect(float x, float y, float w, float h, glm::vec4 color);
    static void drawRectOutline(float x, float y, float w, float h, float thickness, glm::vec4 color);
    static void drawText(const std::string& text, float x, float y, float scale = 1.0f,
                         glm::vec4 color = glm::vec4(1.0f));

private:
    static float s_screenW;
    static float s_screenH;
    static float s_mouseX;
    static float s_mouseY;
    static bool  s_mouseDown;
    static bool  s_mouseClicked;
    static bool  s_hoveredAny;
    static uint64_t s_activeSliderId;

    static std::vector<UIVertex> s_vertices;
    static std::vector<uint32_t> s_indices;

    static SDL_GPUTexture* s_fontTexture;
    static SDL_GPUSampler* s_fontSampler;
    static std::shared_ptr<RHI_Shader> s_vs;
    static std::shared_ptr<RHI_Shader> s_fs;
    static std::shared_ptr<RHI_Pipeline> s_pipeline;

    static RHI_Buffer s_vertexBuffer;
    static RHI_Buffer s_indexBuffer;

    static void createFontAtlas();
};

#endif // UI_RENDERER_H

