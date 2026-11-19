#include <iostream>
#include "../include/Engine.h"
#include "../include/Road/RoadEditor.h"

int main() {
    Engine engine;
    
    // Launch ForceEngine in 1280x800 for the Road Editor
    if (engine.init(1280, 800, "ForceEngine - Jughandle Road Editor")) {
        RoadEditor roadEditor;
        if (!roadEditor.init(engine)) {
            std::cerr << "Failed to initialize Road Editor!" << std::endl;
            return 1;
        }

        // Hook Road Editor into engine loop
        engine.onEvent = [&](const SDL_Event& event) {
            roadEditor.handleEvent(event);
        };

        engine.onUpdate = [&](float dt) {
            roadEditor.update(dt);
        };

        engine.onRender = [&]() {
            roadEditor.render(engine);
        };

        // Run master engine loop
        engine.run();
    }
    
    return 0;
}
