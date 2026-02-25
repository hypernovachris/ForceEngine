#include "../include/Renderer.h"
#include "../include/Entity.h"
#include "../include/RendererComponent.h"
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "../include/LightComponent.h"
#include "../include/ColliderComponent.h"

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

void Renderer::beginScene(CameraComponent* camera) {
    if (camera) {
        m_viewMatrix = camera->getViewMatrix();
        m_projectionMatrix = camera->getProjectionMatrix();
        if (camera->owner) {
            m_viewPos = glm::vec3(camera->owner->worldTransform[3]);
        }
    } else {
        m_viewMatrix = glm::mat4(1.0f);
        m_projectionMatrix = glm::mat4(1.0f);
        m_viewPos = glm::vec3(0.0f);
    }

    activeLights.clear();
    renderQueue.clear();
}

void Renderer::submitNode(std::shared_ptr<Entity> node) {
    // 1. Extract light data
    auto lightComp = node->getComponent<LightComponent>();
    if (lightComp) {
        glm::vec3 worldPos = glm::vec3(node->worldTransform[3]);
        activeLights.push_back({worldPos, lightComp->color, lightComp->intensity});
    }
    
    // 2. Extract render data
    auto renderComp = node->getComponent<RendererComponent>();
    
    if (renderComp && renderComp->model) {
        auto model = renderComp->model;
        
        // Loop through the corresponding meshes and materials
        for (size_t i = 0; i < model->meshes.size(); ++i) {
            auto mesh = model->meshes[i];
            
            // Safety check in case a material is missing
            auto material = (i < model->materials.size()) ? model->materials[i] : nullptr; 

            // Pass the single mesh and material to the GPU
            if (mesh && material) {
                renderQueue.push_back({mesh, material, node->worldTransform});
            }
        }
    }

    // Recurse through all children
    for (auto& child : node->children) {
        submitNode(child);
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

    // Send light data to shader
    shader->setInt("numLights", activeLights.size());

    // Loop through the vector and set the uniforms for each light
    for (size_t i = 0; i < activeLights.size(); ++i) {
        std::string number = std::to_string(i);
        shader->setFloat(("lights[" + number + "].position").c_str(), activeLights[i].position.x, activeLights[i].position.y, activeLights[i].position.z);
        shader->setFloat(("lights[" + number + "].color").c_str(), activeLights[i].color.x, activeLights[i].color.y, activeLights[i].color.z);
        shader->setFloat(("lights[" + number + "].intensity").c_str(), activeLights[i].intensity);
    }
    // Apply Material Properties
    material->apply();

    // Set Model Matrix
    shader->setMat4("model", modelMatrix);

    // Draw the specific mesh!
    mesh->draw(); 
}

void Renderer::endScene() {
    // We now have a complete list of lights and a complete list of meshes!
    
    for (const auto& cmd : renderQueue) {
        // We pass the activeLights array to your low-level draw function
        this->draw(cmd.mesh, cmd.material, cmd.transform);
    }
}

void Renderer::renderDebug(std::shared_ptr<Model> cubeModel) {
    if (!cubeModel || cubeModel->meshes.empty() || cubeModel->materials.empty()) return;
    auto mesh = cubeModel->meshes[0];
    auto material = cubeModel->materials[0];

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Draw all active lights (fixed size 0.5 box)
    for (const auto& light : activeLights) {
        glm::mat4 modelMat = glm::translate(glm::mat4(1.0f), light.position);
        modelMat = glm::scale(modelMat, glm::vec3(0.5f));
        this->draw(mesh, material, modelMat);
    }

    // Draw all colliders
    for (auto* collider : ColliderComponent::allColliders) {
        if (!collider || !collider->owner) continue;
        glm::mat4 modelMat = collider->owner->worldTransform;
        modelMat = glm::scale(modelMat, collider->size);
        this->draw(mesh, material, modelMat);
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}