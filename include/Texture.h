#ifndef TEXTURE_H
#define TEXTURE_H

#include <glad/glad.h>
#include "../include/stb_image.h"
#include <iostream>
#include <string>

class Texture {
public:
    unsigned int ID;

    Texture(const char* imagePath) {
        glGenTextures(1, &ID);
        glBindTexture(GL_TEXTURE_2D, ID);

        // Set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        
        // Set texture filtering parameters (Linear filtering looks smooth)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // OpenGL expects the 0.0 coordinate on the Y-axis to be on the bottom, 
        // but images usually have 0.0 at the top. We must flip it.
        stbi_set_flip_vertically_on_load(true);

        // Load image, create texture and generate mipmaps
        int width, height, nrChannels;
        unsigned char *data = stbi_load(imagePath, &width, &height, &nrChannels, 0);
        
        if (data) {
            // Check if the image has an alpha channel (PNG) or just RGB (JPG)
            GLenum format;
            if (nrChannels == 1)
                format = GL_RED;
            else if (nrChannels == 3)
                format = GL_RGB;
            else if (nrChannels == 4)
                format = GL_RGBA;
            else
                format = GL_RGB; // Default fallback (might still be risky but better than nothing)

            glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);

            if (nrChannels == 1) {
                // If the texture is grayscale (1 channel), we want it to appear as white/gray
                // (R, R, R, 1.0) when sampled in the shader.
                GLint swizzleMask[] = {GL_RED, GL_RED, GL_RED, GL_ONE};
                glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
            }
            glGenerateMipmap(GL_TEXTURE_2D);
        } else {
            std::cout << "Failed to load texture at path: " << imagePath << std::endl;
        }
        
        // Free the image memory from the CPU, it's on the GPU now
        stbi_image_free(data);
    }

    void bind(unsigned int slot = 0) const {
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(GL_TEXTURE_2D, ID);
    }
};

#endif