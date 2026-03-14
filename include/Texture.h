#ifndef TEXTURE_H
#define TEXTURE_H

#include "RHI/RHI_Texture.h"
#include <string>

class Texture {
public:
    RHI_Texture rhiTexture;

    Texture() = default;

    Texture(const char* imagePath) {
        rhiTexture.loadFromFile(imagePath);
    }
};

#endif