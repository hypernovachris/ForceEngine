#ifndef MODEL_H
#define MODEL_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Mesh.h"
#include "Material.h"
#include <vector>
#include <string>
#include <memory>
#include <iostream>

class Model {
public:
    std::vector<std::shared_ptr<Mesh>> meshes;
    std::vector<std::shared_ptr<Material>> materials;
    std::string directory;

    Model(const std::string& path) {
        loadModel(path);
    }

private:
    void loadModel(const std::string& path) {
        Assimp::Importer importer;
        // aiProcess_CalcTangentSpace tells Assimp to do the hard math for us!
        // aiProcess_Triangulate ensures all polygons are converted to triangles.
        // aiProcess_GenSmoothNormals creates normals if they are missing
        const aiScene* scene = importer.ReadFile(path, 
            aiProcess_Triangulate | 
            aiProcess_FlipUVs | 
            aiProcess_CalcTangentSpace |
            aiProcess_GenSmoothNormals |
            aiProcess_GenUVCoords |
            aiProcess_JoinIdenticalVertices);

        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
            return;
        }
        
        // Save the directory path so we can load textures relative to the model later
        directory = path.substr(0, path.find_last_of('/'));

        processNode(scene->mRootNode, scene);
    }

    void processNode(aiNode *node, const aiScene *scene) {
        // Process all the meshes at the current node
        for(unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh *mesh = scene->mMeshes[node->mMeshes[i]]; 
            std::cout << "Mesh: " << (mesh->mName.C_Str() ? mesh->mName.C_Str() : "Unnamed") << "\n";
            std::cout << "  Diff Texture Count: " << mesh->mMaterialIndex << "\n"; // Just an index
            std::cout << "  Vertices: " << mesh->mNumVertices << "\n";
            std::cout << "  Has Normals: " << (mesh->HasNormals() ? "Yes" : "No") << "\n";
            std::cout << "  Has UVs: " << (mesh->HasTextureCoords(0) ? "Yes" : "No") << "\n";
            std::cout << "  Has Tangents: " << (mesh->HasTangentsAndBitangents() ? "Yes" : "No") << "\n";
            
            meshes.push_back(processMesh(mesh, scene));
        }
        // Recursively process child nodes
        for(unsigned int i = 0; i < node->mNumChildren; i++) {
            processNode(node->mChildren[i], scene);
        }
    }

    std::shared_ptr<Mesh> processMesh(aiMesh *mesh, const aiScene *scene) {
        std::vector<float> vertices;
        std::vector<unsigned int> indices;

        for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
            // 1. Positions
            vertices.push_back(mesh->mVertices[i].x);
            vertices.push_back(mesh->mVertices[i].y);
            vertices.push_back(mesh->mVertices[i].z);

            // 2. Normals
            if (mesh->HasNormals()) {
                vertices.push_back(mesh->mNormals[i].x);
                vertices.push_back(mesh->mNormals[i].y);
                vertices.push_back(mesh->mNormals[i].z);
            }

            // 3. Texture Coordinates
            if(mesh->mTextureCoords[0]) {
                vertices.push_back(mesh->mTextureCoords[0][i].x);
                vertices.push_back(mesh->mTextureCoords[0][i].y);
            }

            // 4. Tangents
            if (mesh->HasTangentsAndBitangents()) {
                vertices.push_back(mesh->mTangents[i].x);
                vertices.push_back(mesh->mTangents[i].y);
                vertices.push_back(mesh->mTangents[i].z);
            }
        }

        // 5. Indices
        for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            for(unsigned int j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }

        // Return a shared pointer to the newly constructed Mesh
        return std::make_shared<Mesh>(vertices, indices, mesh->HasNormals(), mesh->mTextureCoords[0] != nullptr);
    }
};

#endif