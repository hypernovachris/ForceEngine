#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "../include/Game.h"

int main() {
    // 1. Initialize GLFW and Window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "Force", NULL, NULL);
    glfwMakeContextCurrent(window);

    // Hide the cursor and capture it for the 3D camera
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // 2. Load GLAD
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glEnable(GL_DEPTH_TEST); // Enable 3D depth testing

    // 3. Instantiate and Initialize our Game
    Game myGame;
    myGame.init(window);

    // Store the Game pointer inside the window for our mouse callback
    glfwSetWindowUserPointer(window, &myGame);

    // 4. The Master Game Loop
    while (!glfwWindowShouldClose(window)) {
        // Calculate deltaTime
        float currentFrame = static_cast<float>(glfwGetTime());
        myGame.deltaTime = currentFrame - myGame.lastFrame;
        myGame.lastFrame = currentFrame;

        // Execute Engine Stages
        myGame.processInput(window);
        myGame.update();
        myGame.render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}