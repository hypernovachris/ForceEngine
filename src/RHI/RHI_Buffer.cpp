#include "RHI/RHI_Buffer.h"
#include <iostream>

RHI_Buffer::~RHI_Buffer() {
    if (buffer) {
        SDL_ReleaseGPUBuffer(RHI_Device::getInstance().getDevice(), buffer);
        buffer = nullptr;
    }
}

bool RHI_Buffer::create(SDL_GPUBufferUsageFlags usage, Uint32 size) {
    if (buffer) return false;

    SDL_GPUBufferCreateInfo info = {};
    info.usage = usage;
    info.size = size;

    buffer = SDL_CreateGPUBuffer(RHI_Device::getInstance().getDevice(), &info);
    if (!buffer) {
        std::cerr << "Failed to create GPU Buffer: " << SDL_GetError() << std::endl;
        return false;
    }

    bufferSize = size;
    return true;
}

bool RHI_Buffer::uploadData(const void* data, Uint32 size) {
    if (!buffer || !data || size > bufferSize) return false;

    SDL_GPUDevice* device = RHI_Device::getInstance().getDevice();

    // 1. Create a Transfer Buffer
    SDL_GPUTransferBufferCreateInfo tbufInfo = {};
    tbufInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tbufInfo.size = size;
    SDL_GPUTransferBuffer* tbuf = SDL_CreateGPUTransferBuffer(device, &tbufInfo);
    if (!tbuf) return false;

    // 2. Map & Copy
    void* map = SDL_MapGPUTransferBuffer(device, tbuf, false);
    if (!map) {
        SDL_ReleaseGPUTransferBuffer(device, tbuf);
        return false;
    }
    memcpy(map, data, size);
    SDL_UnmapGPUTransferBuffer(device, tbuf);

    // 3. Record Copy Command
    SDL_GPUCommandBuffer* cmdBuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(cmdBuf);
    
    SDL_GPUTransferBufferLocation srcLoc = {};
    srcLoc.transfer_buffer = tbuf;
    srcLoc.offset = 0;

    SDL_GPUBufferRegion dstRegion = {};
    dstRegion.buffer = buffer;
    dstRegion.offset = 0;
    dstRegion.size = size;

    SDL_UploadToGPUBuffer(copyPass, &srcLoc, &dstRegion, false);
    SDL_EndGPUCopyPass(copyPass);

    // 4. Submit and cleanup staging
    SDL_SubmitGPUCommandBuffer(cmdBuf);
    SDL_ReleaseGPUTransferBuffer(device, tbuf);

    return true;
}
