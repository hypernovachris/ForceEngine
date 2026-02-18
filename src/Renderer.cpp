#include "../include/Renderer.h"
#include "../include/Entity.h"
#include "../include/RendererComponent.h"
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <glm/glm/gtc/matrix_transform.hpp>

Renderer::Renderer() {
}

Renderer::~Renderer() {
}

void Renderer::init() {
    glEnable(GL_DEPTH_TEST);
}

void Renderer::clear() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::beginScene(const Camera& camera, const glm::vec3& lightPos, const glm::vec3& lightColor) {
    m_viewMatrix = camera.getViewMatrix();
    // Assuming aspect ratio is fixed for now, can be passed in later
    m_projectionMatrix = glm::perspective(glm::radians(camera.zoom), 800.0f / 600.0f, 0.1f, 100.0f);
    m_viewPos = camera.position;
    m_lightPos = lightPos;
    m_lightColor = lightColor;
}

void Renderer::drawNode(std::shared_ptr<Entity> node) {
    auto renderComp = node->getComponent<RendererComponent>();
    
    if (renderComp && renderComp->model) {
        auto model = renderComp->model;
        
        // Loop through the corresponding meshes and materials
        for (size_t i = 0; i < model->meshes.size(); ++i) {
            auto mesh = model->meshes[i];
            
            // Safety check in case a material is missing
            auto material = (i < model->materials.size()) ? model->materials[i] : nullptr; 
            
            // Override if the component has a specific material
            if (renderComp->material) {
                material = renderComp->material;
            }

            // Pass the single mesh and material to the GPU
            if (mesh && material) {
                this->draw(mesh, material, node->worldTransform);
            }
        }
    }

    // Recurse through all children
    for (auto& child : node->children) {
        drawNode(child);
    }
}

void Renderer::draw(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> material, const glm::mat4& modelMatrix) {
    if (!mesh || !material || !material->shader) return;

    auto shader = material->shader;
    shader->use();

    // Set Global Uniforms
    shader->setMat4("view", m_viewMatrix);
    shader->setMat4("projection", m_projectionMatrix);
    shader->setFloat("viewPos", m_viewPos.x, m_viewPos.y, m_viewPos.z);
    shader->setFloat("lightPos", m_lightPos.x, m_lightPos.y, m_lightPos.z);
    shader->setFloat("lightColor", m_lightColor.x, m_lightColor.y, m_lightColor.z);

    // Apply Material Properties
    material->apply();

    // Set Model Matrix
    shader->setMat4("model", modelMatrix);

    // Draw the specific mesh!
    mesh->draw(); 
}

void Renderer::endScene() {
    // Nothing for now
}
