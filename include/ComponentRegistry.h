#ifndef COMPONENT_REGISTRY_H
#define COMPONENT_REGISTRY_H

#include "Component.h"
#include <functional>
#include <map>
#include <string>
#include <memory>
#include <sstream>


// The signature for our creation functions
using ComponentFactoryFunc = std::function<std::shared_ptr<Component>(std::istringstream&, SDL_Window*)>;

class ComponentRegistry {
public:
    static std::map<std::string, ComponentFactoryFunc> map;

    static void registerComponent(const std::string& name, ComponentFactoryFunc func) {
        map[name] = func;
    }

    static std::shared_ptr<Component> create(const std::string& name, std::istringstream& iss, SDL_Window* window) {
        if (map.find(name) != map.end()) {
            return map[name](iss, window); // Call the specific component's static function
        }
        return nullptr;
    }
};

// Define the static map
inline std::map<std::string, ComponentFactoryFunc> ComponentRegistry::map;

#endif