#include "Road/Qt/RoadViewportWidget.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <iostream>

RoadViewportWidget::RoadViewportWidget(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_NativeWindow, true);
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    connect(&m_renderTimer, &QTimer::timeout, this, &RoadViewportWidget::onRenderTimer);
}

RoadViewportWidget::~RoadViewportWidget() {
    m_renderTimer.stop();
    m_engine.shutdown();
}

void RoadViewportWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);

    if (!m_initialized && winId()) {
        int w = std::max(640, width());
        int h = std::max(480, height());

        if (m_engine.initWithNativeView(reinterpret_cast<void*>(winId()), w, h)) {
            m_roadEditor.m_enableInternalUI = false; // Run in pure 3D viewport mode without 2D UI panels
            if (m_roadEditor.init(m_engine)) {
                // Hook Road Editor callbacks into engine
                m_engine.onEvent = [this](const SDL_Event& ev) {
                    m_roadEditor.handleEvent(ev);
                };
                m_engine.onUpdate = [this](float dt) {
                    m_roadEditor.update(dt);
                };
                m_engine.onRender = [this]() {
                    m_roadEditor.render(m_engine);
                };

                m_initialized = true;
                m_frameTimer.start();
                m_renderTimer.start(16); // ~60 FPS
                std::cout << "RoadViewportWidget: ForceEngine 3D Viewport initialized successfully." << std::endl;
            } else {
                std::cerr << "RoadViewportWidget: Failed to initialize RoadEditor." << std::endl;
            }
        } else {
            std::cerr << "RoadViewportWidget: Failed to initialize Engine with native view." << std::endl;
        }
    }
}

void RoadViewportWidget::paintEvent(QPaintEvent* /*event*/) {
    // Metal rendering happens via onRenderTimer
}

void RoadViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (m_initialized) {
        m_engine.resize(event->size().width(), event->size().height());
    }
}

void RoadViewportWidget::onRenderTimer() {
    if (!m_initialized) return;

    float dt = static_cast<float>(m_frameTimer.restart()) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f; // clamp delta time spikes

    m_engine.stepFrame(dt);

    // Frame metrics
    m_fpsAccumTime += dt;
    m_fpsFrameCount++;
    if (m_fpsAccumTime >= 0.5f) {
        float fps = static_cast<float>(m_fpsFrameCount) / m_fpsAccumTime;
        float frameMs = (m_fpsAccumTime / static_cast<float>(m_fpsFrameCount)) * 1000.0f;
        emit frameRendered(fps, frameMs);
        m_fpsAccumTime = 0.0f;
        m_fpsFrameCount = 0;
    }

    // Detect selection change in RoadEditor
    int curSeg = m_roadEditor.getSelectedSegmentIdx();
    int curJunc = m_roadEditor.getSelectedJunctionIdx();
    int curCp = m_roadEditor.getSelectedControlPointIdx();

    if (curSeg != m_lastSegmentIdx || curJunc != m_lastJunctionIdx || curCp != m_lastControlPointIdx) {
        m_lastSegmentIdx = curSeg;
        m_lastJunctionIdx = curJunc;
        m_lastControlPointIdx = curCp;
        emit selectionChanged(curSeg, curJunc, curCp);
    }

    const std::string& curStatus = m_roadEditor.getStatusMessage();
    if (curStatus != m_lastStatusMsg) {
        m_lastStatusMsg = curStatus;
        emit statusChanged(QString::fromStdString(curStatus));
    }
}

void RoadViewportWidget::mousePressEvent(QMouseEvent* event) {
    setFocus();
    m_lastMousePos = event->position();
    m_hasLastMousePos = true;

    SDL_Event ev = {};
    ev.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
    ev.button.x = static_cast<float>(event->position().x());
    ev.button.y = static_cast<float>(event->position().y());

    if (event->button() == Qt::LeftButton) {
        ev.button.button = SDL_BUTTON_LEFT;
    } else if (event->button() == Qt::MiddleButton) {
        ev.button.button = SDL_BUTTON_MIDDLE;
    } else if (event->button() == Qt::RightButton) {
        ev.button.button = SDL_BUTTON_RIGHT;
    }

    if (m_roadEditor.getActiveTool() == EditorTool::Pan && (event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
        setCursor(Qt::ClosedHandCursor);
    }

    m_roadEditor.handleEvent(ev);
}

void RoadViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    SDL_Event ev = {};
    ev.type = SDL_EVENT_MOUSE_BUTTON_UP;
    ev.button.x = static_cast<float>(event->position().x());
    ev.button.y = static_cast<float>(event->position().y());

    if (event->button() == Qt::LeftButton) {
        ev.button.button = SDL_BUTTON_LEFT;
    } else if (event->button() == Qt::MiddleButton) {
        ev.button.button = SDL_BUTTON_MIDDLE;
    } else if (event->button() == Qt::RightButton) {
        ev.button.button = SDL_BUTTON_RIGHT;
    }

    if (m_roadEditor.getActiveTool() == EditorTool::Pan) {
        setCursor(Qt::OpenHandCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }

    m_roadEditor.handleEvent(ev);
}

void RoadViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    QPointF currentPos = event->position();
    float dx = 0.0f;
    float dy = 0.0f;
    if (m_hasLastMousePos) {
        dx = static_cast<float>(currentPos.x() - m_lastMousePos.x());
        dy = static_cast<float>(currentPos.y() - m_lastMousePos.y());
    }
    m_lastMousePos = currentPos;
    m_hasLastMousePos = true;

    SDL_Event ev = {};
    ev.type = SDL_EVENT_MOUSE_MOTION;
    ev.motion.x = static_cast<float>(currentPos.x());
    ev.motion.y = static_cast<float>(currentPos.y());
    ev.motion.xrel = dx;
    ev.motion.yrel = dy;

    m_roadEditor.handleEvent(ev);
}

void RoadViewportWidget::wheelEvent(QWheelEvent* event) {
    SDL_Event ev = {};
    ev.type = SDL_EVENT_MOUSE_WHEEL;
    ev.wheel.y = static_cast<float>(event->angleDelta().y()) / 120.0f;
    ev.wheel.x = static_cast<float>(event->angleDelta().x()) / 120.0f;

    m_roadEditor.handleEvent(ev);
}

namespace {
bool mapQtKeyToSDL(int qtKey, SDL_Scancode& outScancode, SDL_Keycode& outKeycode) {
    switch (qtKey) {
        case Qt::Key_W:
            outScancode = SDL_SCANCODE_W;
            outKeycode = SDLK_W;
            return true;
        case Qt::Key_A:
            outScancode = SDL_SCANCODE_A;
            outKeycode = SDLK_A;
            return true;
        case Qt::Key_S:
            outScancode = SDL_SCANCODE_S;
            outKeycode = SDLK_S;
            return true;
        case Qt::Key_D:
            outScancode = SDL_SCANCODE_D;
            outKeycode = SDLK_D;
            return true;
        case Qt::Key_Up:
            outScancode = SDL_SCANCODE_UP;
            outKeycode = SDLK_UP;
            return true;
        case Qt::Key_Down:
            outScancode = SDL_SCANCODE_DOWN;
            outKeycode = SDLK_DOWN;
            return true;
        case Qt::Key_Left:
            outScancode = SDL_SCANCODE_LEFT;
            outKeycode = SDLK_LEFT;
            return true;
        case Qt::Key_Right:
            outScancode = SDL_SCANCODE_RIGHT;
            outKeycode = SDLK_RIGHT;
            return true;
        case Qt::Key_Space:
            outScancode = SDL_SCANCODE_SPACE;
            outKeycode = SDLK_SPACE;
            return true;
        case Qt::Key_Return:
        case Qt::Key_Enter:
            outScancode = SDL_SCANCODE_RETURN;
            outKeycode = SDLK_RETURN;
            return true;
        case Qt::Key_Escape:
            outScancode = SDL_SCANCODE_ESCAPE;
            outKeycode = SDLK_ESCAPE;
            return true;
        case Qt::Key_Tab:
            outScancode = SDL_SCANCODE_TAB;
            outKeycode = SDLK_TAB;
            return true;
        case Qt::Key_Delete:
            outScancode = SDL_SCANCODE_DELETE;
            outKeycode = SDLK_DELETE;
            return true;
        case Qt::Key_Backspace:
            outScancode = SDL_SCANCODE_BACKSPACE;
            outKeycode = SDLK_BACKSPACE;
            return true;
        case Qt::Key_Z:
            outScancode = SDL_SCANCODE_Z;
            outKeycode = SDLK_Z;
            return true;
        case Qt::Key_X:
            outScancode = SDL_SCANCODE_X;
            outKeycode = SDLK_X;
            return true;
        case Qt::Key_C:
            outScancode = SDL_SCANCODE_C;
            outKeycode = SDLK_C;
            return true;
        case Qt::Key_F:
            outScancode = SDL_SCANCODE_F;
            outKeycode = SDLK_F;
            return true;
        default:
            return false;
    }
}
}

void RoadViewportWidget::keyPressEvent(QKeyEvent* event) {
    SDL_Scancode scancode = SDL_SCANCODE_UNKNOWN;
    SDL_Keycode keycode = SDLK_UNKNOWN;
    if (mapQtKeyToSDL(event->key(), scancode, keycode)) {
        SDL_Event ev = {};
        ev.type = SDL_EVENT_KEY_DOWN;
        ev.key.scancode = scancode;
        ev.key.key = keycode;
        ev.key.down = true;
        ev.key.repeat = event->isAutoRepeat();
        m_roadEditor.handleEvent(ev);
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

void RoadViewportWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->isAutoRepeat()) {
        event->accept();
        return;
    }
    SDL_Scancode scancode = SDL_SCANCODE_UNKNOWN;
    SDL_Keycode keycode = SDLK_UNKNOWN;
    if (mapQtKeyToSDL(event->key(), scancode, keycode)) {
        SDL_Event ev = {};
        ev.type = SDL_EVENT_KEY_UP;
        ev.key.scancode = scancode;
        ev.key.key = keycode;
        ev.key.down = false;
        ev.key.repeat = false;
        m_roadEditor.handleEvent(ev);
        event->accept();
        return;
    }
    QWidget::keyReleaseEvent(event);
}
