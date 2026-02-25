#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm/gtx/euler_angles.hpp>
int main() { glm::mat4 m; float x,y,z; glm::extractEulerAngleXYZ(m, x, y, z); return 0; }
