#ifndef RHI_TEXTURE_H
#define RHI_TEXTURE_H

#include "RHI_Device.h"
#include <SDL3/SDL_gpu.h>
#include <string>

class RHI_Texture {
public:
    RHI_Texture() = default;
    ~RHI_Texture();

    bool loadFromFile(const std::string& path);

    SDL_GPUTexture* getTexture() const { return texture; }
    SDL_GPUSampler* getSampler() const { return sampler; }

private:
    SDL_GPUTexture* texture = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    Uint32 width = 0;
    Uint32 height = 0;
};

#endif
