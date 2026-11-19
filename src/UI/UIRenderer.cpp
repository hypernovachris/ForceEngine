#include "UI/UIRenderer.h"
#include "RHI/RHI_Device.h"
#include <iostream>
#include <algorithm>
#include <cstring>
#include <cmath>

float UI::s_screenW = 1280.0f;
float UI::s_screenH = 800.0f;
float UI::s_mouseX = 0.0f;
float UI::s_mouseY = 0.0f;
bool  UI::s_mouseDown = false;
bool  UI::s_mouseClicked = false;
bool  UI::s_hoveredAny = false;
uint64_t UI::s_activeSliderId = 0;

std::vector<UIVertex> UI::s_vertices;
std::vector<uint32_t> UI::s_indices;

SDL_GPUTexture* UI::s_fontTexture = nullptr;
SDL_GPUSampler* UI::s_fontSampler = nullptr;
std::shared_ptr<RHI_Shader> UI::s_vs = nullptr;
std::shared_ptr<RHI_Shader> UI::s_fs = nullptr;
std::shared_ptr<RHI_Pipeline> UI::s_pipeline = nullptr;

RHI_Buffer UI::s_vertexBuffer;
RHI_Buffer UI::s_indexBuffer;

// 95 printable ASCII glyphs (ASCII 32 ' ' to 126 '~'), 8 bytes per glyph (8x8 bits)
static const uint8_t s_fontData[95 * 8] = {
    // 32 ' '
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 33 '!'
    0x18, 0x3c, 0x3c, 0x18, 0x18, 0x00, 0x18, 0x00,
    // 34 '"'
    0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 35 '#'
    0x36, 0x36, 0x7f, 0x36, 0x7f, 0x36, 0x36, 0x00,
    // 36 '$'
    0x0c, 0x3e, 0x03, 0x1e, 0x30, 0x1f, 0x0c, 0x00,
    // 37 '%'
    0x00, 0x63, 0x33, 0x18, 0x0c, 0x66, 0x63, 0x00,
    // 38 '&'
    0x1c, 0x36, 0x1c, 0x6e, 0x3b, 0x33, 0x6e, 0x00,
    // 39 '\''
    0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 40 '('
    0x0c, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0c, 0x00,
    // 41 ')'
    0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x18, 0x30, 0x00,
    // 42 '*'
    0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00,
    // 43 '+'
    0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00,
    // 44 ','
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30,
    // 45 '-'
    0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00,
    // 46 '.'
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00,
    // 47 '/'
    0x02, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x40, 0x00,
    // 48 '0'
    0x3c, 0x66, 0x6e, 0x76, 0x66, 0x66, 0x3c, 0x00,
    // 49 '1'
    0x18, 0x18, 0x38, 0x18, 0x18, 0x18, 0x7e, 0x00,
    // 50 '2'
    0x3c, 0x66, 0x06, 0x0c, 0x18, 0x30, 0x7e, 0x00,
    // 51 '3'
    0x3c, 0x66, 0x06, 0x1c, 0x06, 0x66, 0x3c, 0x00,
    // 52 '4'
    0x06, 0x0e, 0x1e, 0x36, 0x66, 0x7f, 0x06, 0x00,
    // 53 '5'
    0x7e, 0x60, 0x7c, 0x06, 0x06, 0x66, 0x3c, 0x00,
    // 54 '6'
    0x3c, 0x66, 0x60, 0x7c, 0x66, 0x66, 0x3c, 0x00,
    // 55 '7'
    0x7e, 0x66, 0x0c, 0x18, 0x18, 0x18, 0x18, 0x00,
    // 56 '8'
    0x3c, 0x66, 0x66, 0x3c, 0x66, 0x66, 0x3c, 0x00,
    // 57 '9'
    0x3c, 0x66, 0x66, 0x3e, 0x06, 0x66, 0x3c, 0x00,
    // 58 ':'
    0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00,
    // 59 ';'
    0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x30, 0x00,
    // 60 '<'
    0x06, 0x0c, 0x18, 0x30, 0x18, 0x0c, 0x06, 0x00,
    // 61 '='
    0x00, 0x00, 0x7e, 0x00, 0x7e, 0x00, 0x00, 0x00,
    // 62 '>'
    0x60, 0x30, 0x18, 0x0c, 0x18, 0x30, 0x60, 0x00,
    // 63 '?'
    0x3c, 0x66, 0x06, 0x0c, 0x18, 0x00, 0x18, 0x00,
    // 64 '@'
    0x3c, 0x66, 0x6e, 0x6e, 0x60, 0x62, 0x3c, 0x00,
    // 65 'A'
    0x18, 0x3c, 0x66, 0x66, 0x7e, 0x66, 0x66, 0x00,
    // 66 'B'
    0x7c, 0x66, 0x66, 0x7c, 0x66, 0x66, 0x7c, 0x00,
    // 67 'C'
    0x3c, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3c, 0x00,
    // 68 'D'
    0x78, 0x6c, 0x66, 0x66, 0x66, 0x6c, 0x78, 0x00,
    // 69 'E'
    0x7e, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x7e, 0x00,
    // 70 'F'
    0x7e, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x60, 0x00,
    // 71 'G'
    0x3c, 0x66, 0x60, 0x6e, 0x66, 0x66, 0x3c, 0x00,
    // 72 'H'
    0x66, 0x66, 0x66, 0x7e, 0x66, 0x66, 0x66, 0x00,
    // 73 'I'
    0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x00,
    // 74 'J'
    0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3c, 0x00,
    // 75 'K'
    0x66, 0x6c, 0x78, 0x70, 0x78, 0x6c, 0x66, 0x00,
    // 76 'L'
    0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7e, 0x00,
    // 77 'M'
    0x63, 0x77, 0x7f, 0x6b, 0x63, 0x63, 0x63, 0x00,
    // 78 'N'
    0x66, 0x76, 0x7e, 0x7e, 0x6e, 0x66, 0x66, 0x00,
    // 79 'O'
    0x3c, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x00,
    // 80 'P'
    0x7c, 0x66, 0x66, 0x7c, 0x60, 0x60, 0x60, 0x00,
    // 81 'Q'
    0x3c, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x0e, 0x00,
    // 82 'R'
    0x7c, 0x66, 0x66, 0x7c, 0x78, 0x6c, 0x66, 0x00,
    // 83 'S'
    0x3c, 0x66, 0x60, 0x3c, 0x06, 0x66, 0x3c, 0x00,
    // 84 'T'
    0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00,
    // 85 'U'
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x00,
    // 86 'V'
    0x66, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x00,
    // 87 'W'
    0x63, 0x63, 0x63, 0x6b, 0x7f, 0x77, 0x63, 0x00,
    // 88 'X'
    0x66, 0x66, 0x3c, 0x18, 0x3c, 0x66, 0x66, 0x00,
    // 89 'Y'
    0x66, 0x66, 0x66, 0x3c, 0x18, 0x18, 0x18, 0x00,
    // 90 'Z'
    0x7e, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x7e, 0x00,
    // 91 '['
    0x3c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3c, 0x00,
    // 92 '\\'
    0x40, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x02, 0x00,
    // 93 ']'
    0x3c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x3c, 0x00,
    // 94 '^'
    0x18, 0x3c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 95 '_'
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00,
    // 96 '`'
    0x30, 0x18, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 97 'a'
    0x00, 0x00, 0x3c, 0x06, 0x3e, 0x66, 0x3e, 0x00,
    // 98 'b'
    0x60, 0x60, 0x7c, 0x66, 0x66, 0x66, 0x7c, 0x00,
    // 99 'c'
    0x00, 0x00, 0x3c, 0x60, 0x60, 0x66, 0x3c, 0x00,
    // 100 'd'
    0x06, 0x06, 0x3e, 0x66, 0x66, 0x66, 0x3e, 0x00,
    // 101 'e'
    0x00, 0x00, 0x3c, 0x66, 0x7e, 0x60, 0x3c, 0x00,
    // 102 'f'
    0x0e, 0x18, 0x7c, 0x18, 0x18, 0x18, 0x18, 0x00,
    // 103 'g'
    0x00, 0x00, 0x3e, 0x66, 0x66, 0x3e, 0x06, 0x3c,
    // 104 'h'
    0x60, 0x60, 0x7c, 0x66, 0x66, 0x66, 0x66, 0x00,
    // 105 'i'
    0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3c, 0x00,
    // 106 'j'
    0x06, 0x00, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3c,
    // 107 'k'
    0x60, 0x60, 0x66, 0x6c, 0x78, 0x6c, 0x66, 0x00,
    // 108 'l'
    0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00,
    // 109 'm'
    0x00, 0x00, 0x66, 0x7f, 0x7f, 0x6b, 0x63, 0x00,
    // 110 'n'
    0x00, 0x00, 0x7c, 0x66, 0x66, 0x66, 0x66, 0x00,
    // 111 'o'
    0x00, 0x00, 0x3c, 0x66, 0x66, 0x66, 0x3c, 0x00,
    // 112 'p'
    0x00, 0x00, 0x7c, 0x66, 0x66, 0x7c, 0x60, 0x60,
    // 113 'q'
    0x00, 0x00, 0x3e, 0x66, 0x66, 0x3e, 0x06, 0x06,
    // 114 'r'
    0x00, 0x00, 0x7c, 0x66, 0x60, 0x60, 0x60, 0x00,
    // 115 's'
    0x00, 0x00, 0x3e, 0x60, 0x3c, 0x06, 0x7c, 0x00,
    // 116 't'
    0x18, 0x18, 0x7e, 0x18, 0x18, 0x18, 0x0e, 0x00,
    // 117 'u'
    0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3e, 0x00,
    // 118 'v'
    0x00, 0x00, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x00,
    // 119 'w'
    0x00, 0x00, 0x63, 0x6b, 0x7f, 0x7f, 0x36, 0x00,
    // 120 'x'
    0x00, 0x00, 0x66, 0x3c, 0x18, 0x3c, 0x66, 0x00,
    // 121 'y'
    0x00, 0x00, 0x66, 0x66, 0x66, 0x3e, 0x06, 0x3c,
    // 122 'z'
    0x00, 0x00, 0x7e, 0x0c, 0x18, 0x30, 0x7e, 0x00,
    // 123 '{'
    0x0e, 0x18, 0x18, 0x70, 0x18, 0x18, 0x0e, 0x00,
    // 124 '|'
    0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00,
    // 125 '}'
    0x70, 0x18, 0x18, 0x0e, 0x18, 0x18, 0x70, 0x00,
    // 126 '~'
    0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

bool UI::init() {
    auto& device = RHI_Device::getInstance();
    if (!device.getDevice()) return false;

    createFontAtlas();

    s_vs = std::make_shared<RHI_Shader>();
    s_fs = std::make_shared<RHI_Shader>();

    if (!s_vs->loadFromHLSL("assets/shaders/ui.hlsl", "vs_main", SDL_SHADERCROSS_SHADERSTAGE_VERTEX) ||
        !s_fs->loadFromHLSL("assets/shaders/ui.hlsl", "fs_main", SDL_SHADERCROSS_SHADERSTAGE_FRAGMENT)) {
        std::cerr << "UI: Failed to compile ui.hlsl!" << std::endl;
        return false;
    }

    SDL_GPUVertexAttribute attributes[3] = {};
    attributes[0].location = 0;
    attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    attributes[0].offset = offsetof(UIVertex, x);

    attributes[1].location = 1;
    attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    attributes[1].offset = offsetof(UIVertex, u);

    attributes[2].location = 2;
    attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    attributes[2].offset = offsetof(UIVertex, r);

    SDL_GPUVertexBufferDescription binding[1] = {};
    binding[0].slot = 0;
    binding[0].pitch = sizeof(UIVertex);
    binding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

    s_pipeline = std::make_shared<RHI_Pipeline>();
    SDL_GPUTextureFormat colorFormat = device.getSwapchainFormat();
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;

    if (!s_pipeline->create(s_vs.get(), s_fs.get(),
                            attributes, 3,
                            binding, 1,
                            colorFormat, depthFormat,
                            false, /* enableDepthTest = false */
                            true   /* enableBlend = true */)) {
        std::cerr << "UI: Failed to create UI graphics pipeline!" << std::endl;
        return false;
    }

    // Allocate initial dynamic vertex/index buffers
    s_vertexBuffer.create(SDL_GPU_BUFFERUSAGE_VERTEX, 20000 * sizeof(UIVertex));
    s_indexBuffer.create(SDL_GPU_BUFFERUSAGE_INDEX, 60000 * sizeof(uint32_t));

    std::cout << "UI: Native Immediate-Mode UI initialized successfully!" << std::endl;
    return true;
}

void UI::shutdown() {
    auto device = RHI_Device::getInstance().getDevice();
    if (device) {
        if (s_fontTexture) {
            SDL_ReleaseGPUTexture(device, s_fontTexture);
            s_fontTexture = nullptr;
        }
        if (s_fontSampler) {
            SDL_ReleaseGPUSampler(device, s_fontSampler);
            s_fontSampler = nullptr;
        }
    }
    s_pipeline.reset();
    s_vs.reset();
    s_fs.reset();
}

void UI::createFontAtlas() {
    auto device = RHI_Device::getInstance().getDevice();

    const int atlasW = 128;
    const int atlasH = 128;
    std::vector<uint8_t> pixels(atlasW * atlasH, 0);

    // Pixel (0,0)..(7,7) is solid white (for solid colored rectangles)
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            pixels[y * atlasW + x] = 255;
        }
    }

    // Characters 32..126
    for (int c = 32; c <= 126; ++c) {
        int g = c - 32;
        int cellIndex = g + 16; // Skip row 0
        int cx = (cellIndex % 16) * 8;
        int cy = (cellIndex / 16) * 8;

        for (int py = 0; py < 8; ++py) {
            uint8_t row = s_fontData[g * 8 + py];
            for (int px = 0; px < 8; ++px) {
                bool bit = (row & (0x80 >> px)) != 0;
                pixels[(cy + py) * atlasW + (cx + px)] = bit ? 255 : 0;
            }
        }
    }

    SDL_GPUTextureCreateInfo texInfo = {};
    texInfo.type = SDL_GPU_TEXTURETYPE_2D;
    texInfo.format = SDL_GPU_TEXTUREFORMAT_R8_UNORM;
    texInfo.width = atlasW;
    texInfo.height = atlasH;
    texInfo.layer_count_or_depth = 1;
    texInfo.num_levels = 1;
    texInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    texInfo.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;

    s_fontTexture = SDL_CreateGPUTexture(device, &texInfo);

    // Upload via transfer buffer
    SDL_GPUTransferBufferCreateInfo tbInfo = {};
    tbInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tbInfo.size = atlasW * atlasH;
    SDL_GPUTransferBuffer* tbuf = SDL_CreateGPUTransferBuffer(device, &tbInfo);
    if (tbuf) {
        void* map = SDL_MapGPUTransferBuffer(device, tbuf, false);
        memcpy(map, pixels.data(), pixels.size());
        SDL_UnmapGPUTransferBuffer(device, tbuf);

        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(device);
        SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);

        SDL_GPUTextureTransferInfo src = {};
        src.transfer_buffer = tbuf;
        src.offset = 0;
        src.pixels_per_row = atlasW;
        src.rows_per_layer = atlasH;

        SDL_GPUTextureRegion dst = {};
        dst.texture = s_fontTexture;
        dst.w = atlasW;
        dst.h = atlasH;
        dst.d = 1;

        SDL_UploadToGPUTexture(copy, &src, &dst, false);
        SDL_EndGPUCopyPass(copy);
        SDL_SubmitGPUCommandBuffer(cmd);
        SDL_ReleaseGPUTransferBuffer(device, tbuf);
    }

    // Sampler (Nearest filtering for pixel-perfect font edges)
    SDL_GPUSamplerCreateInfo samplerInfo = {};
    samplerInfo.min_filter = SDL_GPU_FILTER_NEAREST;
    samplerInfo.mag_filter = SDL_GPU_FILTER_NEAREST;
    samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
    samplerInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    samplerInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    s_fontSampler = SDL_CreateGPUSampler(device, &samplerInfo);
}

void UI::beginFrame(float screenW, float screenH, float mouseX, float mouseY,
                    bool mouseDown, bool mouseClicked, float wheelDelta) {
    s_screenW = screenW;
    s_screenH = screenH;
    s_mouseX = mouseX;
    s_mouseY = mouseY;
    s_mouseDown = mouseDown;
    s_mouseClicked = mouseClicked;
    s_hoveredAny = false;

    if (!s_mouseDown) {
        s_activeSliderId = 0;
    }

    s_vertices.clear();
    s_indices.clear();
}

void UI::endFrame() {
    // Ready for render
}

bool UI::wantsMouseInput() {
    return s_hoveredAny || (s_activeSliderId != 0);
}

void UI::drawRect(float x, float y, float w, float h, glm::vec4 color) {
    if (w <= 0.0f || h <= 0.0f) return;

    uint32_t baseIdx = (uint32_t)s_vertices.size();
    float u = 4.0f / 128.0f;
    float v = 4.0f / 128.0f;

    s_vertices.push_back({x,     y,     u, v, color.r, color.g, color.b, color.a});
    s_vertices.push_back({x + w, y,     u, v, color.r, color.g, color.b, color.a});
    s_vertices.push_back({x + w, y + h, u, v, color.r, color.g, color.b, color.a});
    s_vertices.push_back({x,     y + h, u, v, color.r, color.g, color.b, color.a});

    s_indices.push_back(baseIdx + 0);
    s_indices.push_back(baseIdx + 1);
    s_indices.push_back(baseIdx + 2);
    s_indices.push_back(baseIdx + 2);
    s_indices.push_back(baseIdx + 3);
    s_indices.push_back(baseIdx + 0);
}

void UI::drawRectOutline(float x, float y, float w, float h, float thickness, glm::vec4 color) {
    drawRect(x, y, w, thickness, color); // Top
    drawRect(x, y + h - thickness, w, thickness, color); // Bottom
    drawRect(x, y + thickness, thickness, h - 2 * thickness, color); // Left
    drawRect(x + w - thickness, y + thickness, thickness, h - 2 * thickness, color); // Right
}

void UI::drawText(const std::string& text, float x, float y, float scale, glm::vec4 color) {
    float curX = x;
    float glyphW = 8.0f * scale;
    float glyphH = 8.0f * scale;

    for (char c : text) {
        if (c == '\n') {
            y += glyphH + 4.0f;
            curX = x;
            continue;
        }
        if (c < 32 || c > 126) c = '?';

        int g = (int)c - 32;
        int cellIndex = g + 16;
        int cx = (cellIndex % 16);
        int cy = (cellIndex / 16);

        float u0 = (cx * 8.0f) / 128.0f;
        float v0 = (cy * 8.0f) / 128.0f;
        float u1 = ((cx + 1) * 8.0f) / 128.0f;
        float v1 = ((cy + 1) * 8.0f) / 128.0f;

        uint32_t baseIdx = (uint32_t)s_vertices.size();
        s_vertices.push_back({curX,          y,          u0, v0, color.r, color.g, color.b, color.a});
        s_vertices.push_back({curX + glyphW, y,          u1, v0, color.r, color.g, color.b, color.a});
        s_vertices.push_back({curX + glyphW, y + glyphH, u1, v1, color.r, color.g, color.b, color.a});
        s_vertices.push_back({curX,          y + glyphH, u0, v1, color.r, color.g, color.b, color.a});

        s_indices.push_back(baseIdx + 0);
        s_indices.push_back(baseIdx + 1);
        s_indices.push_back(baseIdx + 2);
        s_indices.push_back(baseIdx + 2);
        s_indices.push_back(baseIdx + 3);
        s_indices.push_back(baseIdx + 0);

        curX += glyphW + 1.0f * scale;
    }
}

void UI::panel(const std::string& title, float x, float y, float w, float h, glm::vec4 bgColor) {
    bool hovered = (s_mouseX >= x && s_mouseX <= x + w && s_mouseY >= y && s_mouseY <= y + h);
    if (hovered) s_hoveredAny = true;

    // Drop shadow
    drawRect(x + 4, y + 4, w, h, glm::vec4(0.0f, 0.0f, 0.0f, 0.35f));

    // Panel body
    drawRect(x, y, w, h, bgColor);
    drawRectOutline(x, y, w, h, 1.0f, glm::vec4(0.28f, 0.30f, 0.35f, 0.9f));

    // Header
    if (!title.empty()) {
        drawRect(x, y, w, 26.0f, glm::vec4(0.18f, 0.20f, 0.24f, 0.98f));
        drawRect(x, y + 25.0f, w, 1.0f, glm::vec4(0.32f, 0.35f, 0.40f, 0.8f));
        drawText(title, x + 10.0f, y + 8.0f, 1.25f, glm::vec4(0.95f, 0.95f, 0.98f, 1.0f));
    }
}

bool UI::button(const std::string& text, float x, float y, float w, float h, bool active) {
    bool hovered = (s_mouseX >= x && s_mouseX <= x + w && s_mouseY >= y && s_mouseY <= y + h);
    if (hovered) s_hoveredAny = true;

    glm::vec4 bgCol = glm::vec4(0.20f, 0.22f, 0.26f, 0.95f);
    glm::vec4 borderCol = glm::vec4(0.35f, 0.38f, 0.44f, 0.9f);
    glm::vec4 textCol = glm::vec4(0.90f, 0.92f, 0.95f, 1.0f);

    if (active) {
        bgCol = glm::vec4(0.15f, 0.45f, 0.85f, 1.0f);
        borderCol = glm::vec4(0.4f, 0.7f, 1.0f, 1.0f);
    } else if (hovered) {
        if (s_mouseDown) {
            bgCol = glm::vec4(0.12f, 0.38f, 0.72f, 1.0f);
        } else {
            bgCol = glm::vec4(0.28f, 0.31f, 0.38f, 1.0f);
            borderCol = glm::vec4(0.50f, 0.55f, 0.65f, 1.0f);
        }
    }

    drawRect(x, y, w, h, bgCol);
    drawRectOutline(x, y, w, h, 1.0f, borderCol);

    float textW = (float)text.length() * 9.0f;
    float textX = x + std::max((w - textW) * 0.5f, 6.0f);
    float textY = y + (h - 8.0f) * 0.5f;
    drawText(text, textX, textY, 1.0f, textCol);

    return hovered && s_mouseClicked;
}

bool UI::toggle(const std::string& text, bool& value, float x, float y, float w, float h) {
    bool hovered = (s_mouseX >= x && s_mouseX <= x + w && s_mouseY >= y && s_mouseY <= y + h);
    if (hovered) s_hoveredAny = true;

    float boxSize = 16.0f;
    float boxY = y + (h - boxSize) * 0.5f;

    glm::vec4 boxBg = value ? glm::vec4(0.18f, 0.55f, 0.95f, 1.0f) : glm::vec4(0.16f, 0.17f, 0.20f, 1.0f);
    glm::vec4 borderCol = hovered ? glm::vec4(0.5f, 0.7f, 1.0f, 1.0f) : glm::vec4(0.35f, 0.38f, 0.42f, 1.0f);

    drawRect(x, boxY, boxSize, boxSize, boxBg);
    drawRectOutline(x, boxY, boxSize, boxSize, 1.0f, borderCol);

    if (value) {
        drawText("X", x + 4.0f, boxY + 4.0f, 1.0f, glm::vec4(1.0f));
    }

    drawText(text, x + boxSize + 8.0f, y + (h - 8.0f) * 0.5f, 1.0f, glm::vec4(0.88f, 0.90f, 0.94f, 1.0f));

    if (hovered && s_mouseClicked) {
        value = !value;
        return true;
    }
    return false;
}

bool UI::sliderFloat(const std::string& label, float& value, float minVal, float maxVal,
                     float x, float y, float w, float h) {
    bool hovered = (s_mouseX >= x && s_mouseX <= x + w && s_mouseY >= y && s_mouseY <= y + h);
    if (hovered) s_hoveredAny = true;

    // Track area
    float trackY = y + 16.0f;
    float trackH = 8.0f;
    float trackW = w;

    uint64_t sliderId = std::hash<std::string>{}(label) ^ (static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&value)));
    if (sliderId == 0) sliderId = 1;
    bool changed = false;

    // Only activate this slider if no other slider is currently captured
    if (s_activeSliderId == 0 && hovered && s_mouseDown) {
        s_activeSliderId = sliderId;
    }

    if (s_activeSliderId == sliderId && s_mouseDown) {
        s_hoveredAny = true;
        float frac = std::clamp((s_mouseX - x) / trackW, 0.0f, 1.0f);
        float newVal = minVal + frac * (maxVal - minVal);
        if (std::abs(newVal - value) > 1e-4f) {
            value = newVal;
            changed = true;
        }
    }

    // Label and current value string
    char valBuf[32];
    std::snprintf(valBuf, sizeof(valBuf), "%.2f", value);
    std::string text;
    if (label.rfind("##", 0) == 0) {
        text = valBuf;
    } else {
        text = label + ": " + valBuf;
    }
    drawText(text, x, y, 1.0f, glm::vec4(0.85f, 0.88f, 0.92f, 1.0f));

    // Track
    drawRect(x, trackY, trackW, trackH, glm::vec4(0.18f, 0.20f, 0.24f, 1.0f));
    drawRectOutline(x, trackY, trackW, trackH, 1.0f, glm::vec4(0.30f, 0.33f, 0.38f, 1.0f));

    // Active fill
    float frac = std::clamp((value - minVal) / (maxVal - minVal), 0.0f, 1.0f);
    drawRect(x, trackY, trackW * frac, trackH, glm::vec4(0.18f, 0.52f, 0.90f, 1.0f));

    // Knob
    float knobX = x + trackW * frac - 4.0f;
    float knobY = trackY - 3.0f;
    drawRect(knobX, knobY, 8.0f, 14.0f, glm::vec4(0.95f, 0.95f, 0.98f, 1.0f));
    drawRectOutline(knobX, knobY, 8.0f, 14.0f, 1.0f, glm::vec4(0.1f, 0.1f, 0.1f, 0.8f));

    return changed;
}

void UI::label(const std::string& text, float x, float y, float scale, glm::vec4 color) {
    drawText(text, x, y, scale, color);
}

void UI::separator(float x, float y, float w) {
    drawRect(x, y, w, 1.0f, glm::vec4(0.28f, 0.30f, 0.35f, 0.7f));
}

void UI::render(SDL_GPUCommandBuffer* cmdBuf, SDL_GPUTexture* swapchainTexture) {
    if (s_vertices.empty() || s_indices.empty()) return;

    auto device = RHI_Device::getInstance().getDevice();
    if (!device || !cmdBuf || !swapchainTexture || !s_pipeline) return;

    // Upload vertices and indices
    Uint32 vBytes = (Uint32)(s_vertices.size() * sizeof(UIVertex));
    Uint32 iBytes = (Uint32)(s_indices.size() * sizeof(uint32_t));

    s_vertexBuffer.uploadData(s_vertices.data(), vBytes);
    s_indexBuffer.uploadData(s_indices.data(), iBytes);

    // Begin UI render pass (Load swapchain content, do NOT clear!)
    SDL_GPUColorTargetInfo colorTarget = {};
    colorTarget.texture = swapchainTexture;
    colorTarget.load_op = SDL_GPU_LOADOP_LOAD;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;

    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmdBuf, &colorTarget, 1, nullptr);
    if (!pass) return;

    SDL_BindGPUGraphicsPipeline(pass, s_pipeline->getPipeline());

    // Push screen size uniform
    struct UIGlobals {
        float screenSize[2];
        float pad[2];
    } globals = { { s_screenW, s_screenH }, { 0.0f, 0.0f } };
    SDL_PushGPUVertexUniformData(cmdBuf, 0, &globals, sizeof(globals));

    // Bind font texture & sampler
    SDL_GPUTextureSamplerBinding fontBinding = {};
    fontBinding.texture = s_fontTexture;
    fontBinding.sampler = s_fontSampler;
    SDL_BindGPUFragmentSamplers(pass, 0, &fontBinding, 1);

    // Bind vertex and index buffers
    SDL_GPUBufferBinding vBinding = {};
    vBinding.buffer = s_vertexBuffer.getBuffer();
    vBinding.offset = 0;
    SDL_BindGPUVertexBuffers(pass, 0, &vBinding, 1);

    SDL_GPUBufferBinding iBinding = {};
    iBinding.buffer = s_indexBuffer.getBuffer();
    iBinding.offset = 0;
    SDL_BindGPUIndexBuffer(pass, &iBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

    SDL_DrawGPUIndexedPrimitives(pass, (Uint32)s_indices.size(), 1, 0, 0, 0);

    SDL_EndGPURenderPass(pass);
}

