#ifndef ROAD_VIEWPORT_WIDGET_H
#define ROAD_VIEWPORT_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QElapsedTimer>
#include <memory>
#include "Engine.h"
#include "Road/RoadEditor.h"

class RoadViewportWidget : public QWidget {
    Q_OBJECT
public:
    explicit RoadViewportWidget(QWidget* parent = nullptr);
    ~RoadViewportWidget() override;

    RoadEditor& getRoadEditor() { return m_roadEditor; }
    Engine& getEngine() { return m_engine; }

    QPaintEngine* paintEngine() const override { return nullptr; }

signals:
    void selectionChanged(int segmentIdx, int junctionIdx, int controlPointIdx);
    void statusChanged(const QString& status);
    void frameRendered(float fps, float frameTimeMs);

protected:
    void showEvent(QShowEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onRenderTimer();

private:
    bool m_initialized = false;
    Engine m_engine;
    RoadEditor m_roadEditor;
    QTimer m_renderTimer;
    QElapsedTimer m_frameTimer;

    // Mouse tracking for relative deltas
    QPointF m_lastMousePos;
    bool m_hasLastMousePos = false;

    // Cached selection states to emit signals on change
    int m_lastSegmentIdx = -1;
    int m_lastJunctionIdx = -1;
    int m_lastControlPointIdx = -1;
    std::string m_lastStatusMsg;

    float m_fpsAccumTime = 0.0f;
    int m_fpsFrameCount = 0;
};

#endif // ROAD_VIEWPORT_WIDGET_H
