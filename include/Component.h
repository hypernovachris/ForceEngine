#ifndef COMPONENT_H
#define COMPONENT_H

// Forward declaration to avoid circular includes
class Entity; 

class Component {
public:
    Entity* owner = nullptr;

    virtual ~Component() = default;

    // Called once when the component is attached to the entity
    virtual void awake() {} 

    // Called every frame
    virtual void update(float deltaTime) {} 
};

#endif