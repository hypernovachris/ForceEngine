#define GLFW_KEY_ESCAPE 0

// Include real GLM
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

// Mock other classes that Entity.h forward declares or uses as pointers
class Mesh {
public:
    void draw() {}
};
class Material {
public:
    class Shader* shader;
    void apply() {}
};
class Shader {
public:
    void use() {}
    void setFloat(const char*, float, float, float) {}
    void setMat4(const char*, const glm::mat4&) {}
};

// Include the ACTUAL header
#include "../include/Entity.h"

#include <iostream>

// Create a test entity that sets a flag when updated
class TestEntity : public Entity {
public:
    bool updated = false;
    void update(float deltaTime) override {
        updated = true;
    }
};

int main() {
    std::cout << "Starting Real Prop Update Test (with real GLM)..." << std::endl;

    Prop* parentProp = new Prop();
    TestEntity* childEntity = new TestEntity();
    
    parentProp->addChild(childEntity);

    // Initial state
    if (childEntity->updated) {
        std::cerr << "FAIL: Child shouldn't be updated yet." << std::endl;
        return 1;
    }

    // Run update
    std::cout << "Calling update on parent Prop..." << std::endl;
    parentProp->update(0.16f);

    // Check result
    if (childEntity->updated) {
        std::cout << "SUCCESS: Child was updated!" << std::endl;
        delete parentProp; // Clean up
        return 0;
    } else {
        std::cout << "FAIL: Child was NOT updated!" << std::endl;
        delete parentProp; // Clean up
        return 1;
    }
}
