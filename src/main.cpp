#include <iostream>
#include "../include/Engine.h"
#include "../include/Game.h"

int main() {
    Engine engine;
    
    if (engine.init(800, 600, "Force Engine")) {
        // Initialize the specific Game application on top of the engine
        Game flappyGame;
        flappyGame.init(engine);
        
        // Hand over control to the engine's master loop
        engine.run();
    }
    
    return 0;
}