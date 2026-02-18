#include "../include/ResourceManager.h"

// Define static members
std::map<std::string, std::shared_ptr<Shader>> ResourceManager::Shaders;
std::map<std::string, std::shared_ptr<Texture>> ResourceManager::Textures;
std::map<std::string, std::shared_ptr<Model>> ResourceManager::Models;
