#ifndef ROAD_EDITOR_MAIN_WINDOW_H
#define ROAD_EDITOR_MAIN_WINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QStackedWidget>
#include <QScrollArea>
#include <QToolBar>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QListWidget>
#include <QDialog>
#include <QKeyEvent>
#include "Road/Qt/RoadViewportWidget.h"

class RoadEditorMainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit RoadEditorMainWindow(QWidget* parent = nullptr);
    ~RoadEditorMainWindow() override = default;

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onSelectionChanged(int segmentIdx, int junctionIdx, int controlPointIdx);
    void onStatusChanged(const QString& status);
    void onFrameRendered(float fps, float frameTimeMs);

    // Actions & Menus
    void onOpenChunkDialog();
    void onReloadChunk();
    void onSaveChunk();
    void onDeleteSelected();
    void onApplyAashto();
    void onToggleAddJunction(bool checked);
    void onCreateNewSegment();
    void onTestDriveSelectedRoad();
    void onViewModeChanged(int index);
    void onRecenterOrbit();
    void onResetCamera();
    void onShowNetworkStatistics();

    // Inspector - Road Controls
    void onDeleteSelectedSegment();
    void onLanesChanged(int lanes);
    void onOneWayChanged(int state);
    void onLaneWidthChanged(double width);
    void onShoulderWidthChanged(double width);
    void onSampleStepChanged(double step);
    void onLayerChanged(int layer);
    void onBridgeTunnelChanged(int index);
    void onCpElevationChanged(double elev);
    void onInsertPointBefore();
    void onInsertPointAfter();
    void onSplitSegment();
    void onDeleteControlPoint();
    void onDisconnectStartJunction();
    void onInspectStartJunction();
    void onDisconnectEndJunction();
    void onInspectEndJunction();

    // Solver Panel Controls
    void onSolverTensionChanged(int value);
    void onSolverAdhesionChanged(int value);
    void onSolverRepulsionChanged(int value);
    void onSolverClearanceChanged(int value);
    void onSolverRelaxationChanged(int value);
    void onLiveAutoSolveToggled(bool checked);
    void onSolveElevationNow();
    void onResetSolverDefaults();

    // Inspector - Junction Controls
    void onRemoveSelectedJunction();
    void onDeselectJunction();
    void onJunctionListClicked(int row);

private:
    void createMenus();
    void createToolbar();
    void createInspectorDock();
    void createSolverDock();
    void createStatsDialog();
    void createStatusBar();

    void updateStatusBarInfo();
    void updateRoadInspector(int segIdx, int cpIdx);
    void updateJunctionInspector(int juncIdx);
    void updateSolverPanel();
    void updateStatsDialog();

    // 3D Viewport
    RoadViewportWidget* m_viewport = nullptr;

    // Right Docks
    QDockWidget* m_inspectorDock = nullptr;
    QDockWidget* m_solverDock = nullptr;
    QScrollArea* m_inspectorScrollArea = nullptr;

    // Contextual Inspector Stack
    QStackedWidget* m_inspectorStack = nullptr;
    QWidget* m_placeholderWidget = nullptr;
    QWidget* m_roadWidget = nullptr;
    QWidget* m_junctionWidget = nullptr;

    // Toolbar Controls
    QAction* m_actPan = nullptr;
    QAction* m_actSelect = nullptr;
    QAction* m_actAddJunction = nullptr;
    QAction* m_actAddSegment = nullptr;
    QAction* m_actTestDrive = nullptr;

    // Road Controls
    QLabel* m_lblRoadName = nullptr;
    QLabel* m_lblRoadClass = nullptr;
    QPushButton* m_btnDeleteSegment = nullptr;
    QSpinBox* m_spinLanes = nullptr;
    QCheckBox* m_chkOneWay = nullptr;
    QDoubleSpinBox* m_spinLaneWidth = nullptr;
    QDoubleSpinBox* m_spinShoulderWidth = nullptr;
    QDoubleSpinBox* m_spinSampleStep = nullptr;
    QSpinBox* m_spinLayer = nullptr;
    QComboBox* m_comboBridgeTunnel = nullptr;
    QLabel* m_lblControlPointInfo = nullptr;
    QDoubleSpinBox* m_spinCpElevation = nullptr;
    QPushButton* m_btnInsertBefore = nullptr;
    QPushButton* m_btnInsertAfter = nullptr;
    QPushButton* m_btnSplitSegment = nullptr;
    QPushButton* m_btnDeleteCp = nullptr;
    QLabel* m_lblStartJuncInfo = nullptr;
    QPushButton* m_btnDisconnectStartJunc = nullptr;
    QPushButton* m_btnInspectStartJunc = nullptr;
    QLabel* m_lblEndJuncInfo = nullptr;
    QPushButton* m_btnDisconnectEndJunc = nullptr;
    QPushButton* m_btnInspectEndJunc = nullptr;

    // Junction Controls
    QLabel* m_lblJunctionHeader = nullptr;
    QLabel* m_lblJunctionPos = nullptr;
    QPushButton* m_btnRemoveJunction = nullptr;
    QPushButton* m_btnDeselectJunction = nullptr;
    QTableWidget* m_tableConnectedRoads = nullptr;
    QListWidget* m_listAllJunctions = nullptr;
    QPushButton* m_btnAddJunctionInTab = nullptr;

    // Solver Panel Controls
    QSlider* m_sliderTension = nullptr;
    QLabel* m_lblTensionVal = nullptr;
    QSlider* m_sliderAdhesion = nullptr;
    QLabel* m_lblAdhesionVal = nullptr;
    QSlider* m_sliderRepulsion = nullptr;
    QLabel* m_lblRepulsionVal = nullptr;
    QSlider* m_sliderClearance = nullptr;
    QLabel* m_lblClearanceVal = nullptr;
    QSlider* m_sliderRelaxation = nullptr;
    QLabel* m_lblRelaxationVal = nullptr;
    QCheckBox* m_chkAutoSolve = nullptr;
    QLabel* m_lblSolverStats = nullptr;

    // Floating Network Statistics Window
    QDialog* m_statsDialog = nullptr;
    QLabel* m_lblStatTotalSegments = nullptr;
    QLabel* m_lblStatActiveSegments = nullptr;
    QLabel* m_lblStatTotalJunctions = nullptr;
    QLabel* m_lblStatVertices = nullptr;
    QLabel* m_lblStatTriangles = nullptr;

    // Status Bar Labels
    QLabel* m_statusMsgLabel = nullptr;
    QLabel* m_statusGpsLabel = nullptr;
    QLabel* m_statusChunkLabel = nullptr;
    QLabel* m_statusFpsLabel = nullptr;

    bool m_updatingUi = false;
};

#endif // ROAD_EDITOR_MAIN_WINDOW_H
