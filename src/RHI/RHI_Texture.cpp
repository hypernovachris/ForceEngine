#include "RHI/RHI_Texture.h"
#include "stb_image.h"
#include <iostream>

RHI_Texture::~RHI_Texture() {
    SDL_GPUDevice* device = RHI_Device::getInstance().getDevice();
    if (texture) {
        SDL_ReleaseGPUTexture(device, texture);
        texture = nullptr;
    }
    if (sampler) {
        SDL_ReleaseGPUSampler(device, sampler);
        sampler = nullptr;
    }
}

bool RHI_Texture::loadFromFile(const std::string& path) {
    if (texture) return false;

    stbi_set_flip_vertically_on_load(false); // MSL/Metal uses top-left origin usually, wait... actually Metal uses Top-Left origin (0,0) is top-left.
    // OpenGL used bottom-left. We will keep false for Metal
    int w, h, channels;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 4); // force 4 channels
    if (!data) {
        std::cerr << "Failed to load texture: " << path << std::endl;
        return false;
    }

    width = w;
    height = h;
    Uint32 dataSize = width * height * 4;

    SDL_GPUDevice* device = RHI_Device::getInstance().getDevice();

    // Create the GPU Texture
    SDL_GPUTextureCreateInfo texInfo = {};
    texInfo.type = SDL_GPU_TEXTURETYPE_2D;
    texInfo.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    texInfo.width = width;
    texInfo.height = height;
    texInfo.layer_count_or_depth = 1;
    texInfo.num_levels = 1;
    texInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    texInfo.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;

    texture = SDL_CreateGPUTexture(device, &texInfo);
    if (!texture) {
        std::cerr << "Failed to create GPU Texture: " << SDL_GetError() << std::endl;
        stbi_image_free(data);
        return false;
    }

    // Upload Data via Transfer Buffer
    SDL_GPUTransferBufferCreateInfo tbufInfo = {};
    tbufInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tbufInfo.size = dataSize;
    SDL_GPUTransferBuffer* tbuf = SDL_CreateGPUTransferBuffer(device, &tbufInfo);

    void* map = SDL_MapGPUTransferBuffer(device, tbuf, false);
    memcpy(map, data, dataSize);
    SDL_UnmapGPUTransferBuffer(device, tbuf);

    SDL_GPUCommandBuffer* cmdBuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(cmdBuf);

    SDL_GPUTextureTransferInfo srcInfo = {};
    srcInfo.transfer_buffer = tbuf;
    srcInfo.offset = 0;

    SDL_GPUTextureRegion dstRegion = {};
    dstRegion.texture = texture;
    dstRegion.w = width;
    dstRegion.h = height;
    dstRegion.d = 1;

    SDL_UploadToGPUTexture(copyPass, &srcInfo, &dstRegion, false);
    SDL_EndGPUCopyPass(copyPass);

    SDL_SubmitGPUCommandBuffer(cmdBuf);
    SDL_ReleaseGPUTransferBuffer(device, tbuf);
    
    stbi_image_free(data);

    // Create Sampler
    SDL_GPUSamplerCreateInfo samplerInfo = {};
    samplerInfo.min_filter = SDL_GPU_FILTER_LINEAR;
    samplerInfo.mag_filter = SDL_GPU_FILTER_LINEAR;
    samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
    samplerInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    samplerInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    
    sampler = SDL_CreateGPUSampler(device, &samplerInfo);

    return true;
}
