#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h> // Include glad to get all the required OpenGL headers
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

class Shader {
public:
    // The program ID
    unsigned int ID;

    // Constructor reads the files and builds the shader
    Shader(const char* vertexPath, const char* fragmentPath);
    
    // Use/activate the shader program
    void use();
    
    // Utility functions to pass data to the GPU later
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec2(const std::string &name, const glm::vec2 &value) const;
    void setVec2(const std::string &name, float x, float y) const;
    void setFloat(const std::string &name, float x, float y, float z) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;
};

#endif