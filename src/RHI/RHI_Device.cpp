#include "RHI/RHI_Device.h"
#include <iostream>

bool RHI_Device::init(SDL_Window* inWindow) {
    if (!inWindow) return false;
    window = inWindow;

    // Create the GPU Device (Metal on Mac)
    device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_METALLIB, true, nullptr);
    if (!device) {
        std::cerr << "Failed to create SDL GPU Device: " << SDL_GetError() << std::endl;
        return false;
    }

    // Claim the window for the swapchain
    if (!SDL_ClaimWindowForGPUDevice(device, window)) {
        std::cerr << "Failed to claim window for GPU Device: " << SDL_GetError() << std::endl;
        return false;
    }

    swapchainFormat = SDL_GetGPUSwapchainTextureFormat(device, window);
    return true;
}

void RHI_Device::shutdown() {
    if (device) {
        if (window) {
            SDL_ReleaseWindowFromGPUDevice(device, window);
            window = nullptr;
        }
        SDL_DestroyGPUDevice(device);
        device = nullptr;
    }
}

SDL_GPUCommandBuffer* RHI_Device::beginFrame(SDL_GPUTexture*& outSwapchainTexture) {
    if (!device || !window) return nullptr;

    SDL_GPUCommandBuffer* cmdBuf = SDL_AcquireGPUCommandBuffer(device);
    if (!cmdBuf) {
        std::cerr << "Failed to acquire command buffer: " << SDL_GetError() << std::endl;
        return nullptr;
    }

    if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmdBuf, window, &outSwapchainTexture, nullptr, nullptr)) {
        std::cerr << "Failed to acquire swapchain texture: " << SDL_GetError() << std::endl;
        SDL_SubmitGPUCommandBuffer(cmdBuf); // submit empty
        return nullptr;
    }

    return cmdBuf;
}

void RHI_Device::endFrame(SDL_GPUCommandBuffer* cmdBuf) {
    if (cmdBuf) {
        SDL_SubmitGPUCommandBuffer(cmdBuf);
    }
}
