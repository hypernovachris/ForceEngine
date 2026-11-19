#ifndef TRIGGER_SYSTEM_H
#define TRIGGER_SYSTEM_H

#include <vector>
#include <functional>
#include <glm/glm/glm.hpp>
#include "MapBinary.h"

struct ActiveTrigger {
    FMapTrigger data;
    bool wasInside = false;
};

using TriggerCallback = std::function<void(const FMapTrigger& trigger, bool isEnter)>;

class TriggerSystem {
public:
    static TriggerSystem& getInstance() {
        static TriggerSystem instance;
        return instance;
    }

    void clear() {
        m_triggers.clear();
    }

    void registerTrigger(const FMapTrigger& trigger) {
        ActiveTrigger at = {};
        at.data = trigger;
        at.wasInside = false;
        m_triggers.push_back(at);
    }

    void setCallback(TriggerCallback cb) {
        m_callback = cb;
    }

    void update(const glm::vec3& position) {
        for (auto& trigger : m_triggers) {
            glm::vec3 bMin = trigger.data.center - trigger.data.halfExtents;
            glm::vec3 bMax = trigger.data.center + trigger.data.halfExtents;

            bool isInside = (position.x >= bMin.x && position.x <= bMax.x &&
                             position.y >= bMin.y && position.y <= bMax.y &&
                             position.z >= bMin.z && position.z <= bMax.z);

            if (isInside && !trigger.wasInside) {
                trigger.wasInside = true;
                if (m_callback) {
                    m_callback(trigger.data, true);
                }
            } else if (!isInside && trigger.wasInside) {
                trigger.wasInside = false;
                if (m_callback) {
                    m_callback(trigger.data, false);
                }
            }
        }
    }

    const std::vector<ActiveTrigger>& getTriggers() const {
        return m_triggers;
    }

private:
    TriggerSystem() = default;
    std::vector<ActiveTrigger> m_triggers;
    TriggerCallback m_callback = nullptr;
};

#endif // TRIGGER_SYSTEM_H
