#ifndef GAME_H
#define GAME_H

#include <vector>
#include <memory>
#include "Entity.h"
#include "Engine.h"

// The Game class now represents a specific application running on the Engine.
class Game {
public:
    Game();
    ~Game();

    // Initialize game-specific components and load the initial scene using the engine
    void init(Engine& engine);

private:
    std::shared_ptr<Entity> visualEntity;
};

#endif