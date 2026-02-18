#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 3) in vec3 aTangent; // The new Tangent attribute!

out vec3 FragPos;
out vec2 TexCoords;
out mat3 TBN; // Pass the TBN matrix to the fragment shader

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec2 textureScale;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    TexCoords = aTexCoords * textureScale;
    
    // Calculate the TBN Matrix
    vec3 T = normalize(vec3(model * vec4(aTangent, 0.0)));
    vec3 N = normalize(vec3(model * vec4(aNormal, 0.0)));
    
    // Re-orthogonalize T with respect to N (Gram-Schmidt process)
    T = normalize(T - dot(T, N) * N);
    
    // Calculate Bitangent
    vec3 B = cross(N, T);
    
    TBN = mat3(T, B, N);
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
}