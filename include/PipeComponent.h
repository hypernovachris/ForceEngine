#ifndef PIPE_COMPONENT_H
#define PIPE_COMPONENT_H

#include "Component.h"
#include "Entity.h"
#include "ColliderComponent.h"

class PipeComponent : public Component {
public:
    ColliderComponent* myCollider;
    ColliderComponent* leftBoundary;

    PipeComponent(ColliderComponent* myCol, ColliderComponent* lb) 
        : myCollider(myCol), leftBoundary(lb) {}

    void update(float deltaTime) override {
        if (!owner) return;
        
        // If pipe touches the left boundary, mark for deletion
        if (myCollider && leftBoundary && myCollider->isCollidingWith(leftBoundary)) {
            owner->pendingDestroy = true;
        }
    }

    
};

#endif
