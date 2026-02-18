#version 330 core
out vec4 FragColor;

in vec3 FragPos;  
in vec2 TexCoords;
in mat3 TBN;
  
uniform vec3 lightPos; 
uniform vec3 lightColor;
uniform vec3 viewPos; // Camera position for specular math

// Material Data Structure
struct Material {
    sampler2D diffuse;
    sampler2D specular;
    sampler2D normal;
    float shininess;
}; 

uniform Material material;
uniform float hasDiffuse;
uniform float hasSpecular;
uniform float hasNormalMap;

void main() {
    // --- 1. NORMAL MAPPING ---
    vec3 norm;
    if (hasNormalMap > 0.5) {
        // Obtain normal from normal map in range [0,1]
        norm = texture(material.normal, TexCoords).rgb;
        // Transform normal vector to range [-1,1]
        norm = normalize(norm * 2.0 - 1.0);  
        // Multiply by TBN to move from Tangent Space to World Space
        norm = normalize(TBN * norm); 
    } else {
        // Fallback to vertex normal (which is the Z column of the TBN matrix)
        norm = normalize(TBN[2]); 
    }

    // Base Color
    vec3 diffuseColor = (hasDiffuse > 0.5) ? texture(material.diffuse, TexCoords).rgb : vec3(1.0);

    // --- 2. AMBIENT ---
    vec3 ambient = 0.1 * lightColor * diffuseColor;
  	
    // --- 3. DIFFUSE ---
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = lightColor * diff * diffuseColor;
            
    // --- 4. SPECULAR ---
    vec3 viewDir = normalize(viewPos - FragPos);
    
    // Blinn-Phong calculation (Uses halfway vector for better realism)
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    float spec = pow(max(dot(norm, halfwayDir), 0.0), material.shininess);
    
    vec3 specularMapColor = (hasSpecular > 0.5) ? texture(material.specular, TexCoords).rgb : vec3(0.0f); // Default to no specularity
    vec3 specular = lightColor * spec * specularMapColor;

    // --- FINAL COMPOSITION ---
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}