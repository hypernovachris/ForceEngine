#ifndef RHI_BUFFER_H
#define RHI_BUFFER_H

#include "RHI_Device.h"
#include <SDL3/SDL_gpu.h>
#include <vector>

class RHI_Buffer {
public:
    RHI_Buffer() = default;
    ~RHI_Buffer();

    // Create a GPU Buffer 
    bool create(SDL_GPUBufferUsageFlags usage, Uint32 size);
    
    // Upload data using a staging buffer
    bool uploadData(const void* data, Uint32 size);

    SDL_GPUBuffer* getBuffer() const { return buffer; }
    Uint32 getSize() const { return bufferSize; }

private:
    SDL_GPUBuffer* buffer = nullptr;
    Uint32 bufferSize = 0;
};

#endif
