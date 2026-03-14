#ifndef RHI_DEVICE_H
#define RHI_DEVICE_H

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

class RHI_Device {
public:
    static RHI_Device& getInstance() {
        static RHI_Device instance;
        return instance;
    }

    bool init(SDL_Window* window);
    void shutdown();

    SDL_GPUDevice* getDevice() const { return device; }
    SDL_Window* getWindow() const { return window; }
    SDL_GPUTextureFormat getSwapchainFormat() const { return swapchainFormat; }

    // Begin a command buffer and acquire a swapchain texture
    SDL_GPUCommandBuffer* beginFrame(SDL_GPUTexture*& outSwapchainTexture);

    // Submit the command buffer
    void endFrame(SDL_GPUCommandBuffer* cmdBuf);

private:
    RHI_Device() = default;
    ~RHI_Device() { shutdown(); }
    RHI_Device(const RHI_Device&) = delete;
    RHI_Device& operator=(const RHI_Device&) = delete;

    SDL_GPUDevice* device = nullptr;
    SDL_Window* window = nullptr;
    SDL_GPUTextureFormat swapchainFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
};

#endif
