#include "Road/Qt/RoadEditorMainWindow.h"
#include "Road/Qt/ChunkInputDialog.h"
#include "Terrain/QuadsphereMath.h"
#include <QActionGroup>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QFile>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QScrollArea>
#include <QScrollBar>
#include <QStatusBar>
#include <QVBoxLayout>
#include <cmath>
#include <iomanip>
#include <sstream>

RoadEditorMainWindow::RoadEditorMainWindow(QWidget *parent)
    : QMainWindow(parent) {
  setWindowTitle("ForceEngine - Road Editor");
  resize(1360, 860);

  // 1. Direct Central ForceEngine 3D Viewport (no tabs above it)
  m_viewport = new RoadViewportWidget(this);
  setCentralWidget(m_viewport);

  connect(m_viewport, &RoadViewportWidget::selectionChanged, this,
          &RoadEditorMainWindow::onSelectionChanged);
  connect(m_viewport, &RoadViewportWidget::statusChanged, this,
          &RoadEditorMainWindow::onStatusChanged);
  connect(m_viewport, &RoadViewportWidget::frameRendered, this,
          &RoadEditorMainWindow::onFrameRendered);

  // 2. Build Right Docks (Inspector & Solver), Floating Stats, Menus, Toolbar,
  // and Status Bar
  createInspectorDock();
  createSolverDock();
  createStatsDialog();
  createMenus();
  createToolbar();
  createStatusBar();

  // Split right docks vertically: Inspector on top, Solver below
  splitDockWidget(m_inspectorDock, m_solverDock, Qt::Vertical);

  // Initial contextual state: nothing selected -> show placeholder
  m_inspectorStack->setCurrentWidget(m_placeholderWidget);
  if (m_inspectorScrollArea) {
    m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  }
  updateSolverPanel();
  updateStatusBarInfo();
}

void RoadEditorMainWindow::createMenus() {
  // 1. File Menu
  QMenu *fileMenu = menuBar()->addMenu("&File");
  fileMenu->addAction("Load Chunk", QKeySequence::Open, this,
                      &RoadEditorMainWindow::onOpenChunkDialog);
  fileMenu->addAction("Reload Chunk", QKeySequence::Refresh, this,
                      &RoadEditorMainWindow::onReloadChunk);
  fileMenu->addAction("Save .froad", QKeySequence::Save, this,
                      &RoadEditorMainWindow::onSaveChunk);
  fileMenu->addSeparator();
  fileMenu->addAction("Exit", QKeySequence::Quit, this, &QWidget::close);

  // 2. Edit Menu
  QMenu *editMenu = menuBar()->addMenu("&Edit");
  editMenu->addAction("Delete", QKeySequence::Delete, this,
                      &RoadEditorMainWindow::onDeleteSelected);
  editMenu->addAction("Split Segment at Control Point", this,
                      &RoadEditorMainWindow::onSplitSegment);

  // 3. View Menu
  QMenu *viewMenu = menuBar()->addMenu("&View");
  viewMenu->addAction("Top-Down (2D)", [this]() { onViewModeChanged(0); });
  viewMenu->addAction("3D Orbit", [this]() { onViewModeChanged(1); });
  viewMenu->addSeparator();
  viewMenu->addAction("Recenter on Terrain", QKeySequence("F"), this,
                      &RoadEditorMainWindow::onRecenterOrbit);
  viewMenu->addAction("Reset Camera Focus", QKeySequence("Home"), this,
                      &RoadEditorMainWindow::onResetCamera);

  // 4. Window Menu
  QMenu *windowMenu = menuBar()->addMenu("&Window");
  windowMenu->addAction(m_inspectorDock->toggleViewAction());
  windowMenu->addAction(m_solverDock->toggleViewAction());
  windowMenu->addSeparator();
  windowMenu->addAction("Network Statistics...", this,
                        &RoadEditorMainWindow::onShowNetworkStatistics);

  // 5. Help Menu
  QMenu *helpMenu = menuBar()->addMenu("&Help");
  helpMenu->addAction("About ForceEngine Road Editor", [this]() {
    QMessageBox::about(
        this, "About ForceEngine Road Editor",
        "<h3>ForceEngine Road Editor</h3>"
        "<p>Procedural road topology and physics editor powered by ForceEngine "
        "Metal 3D Viewport.</p>"
        "<p>Supports Quadsphere L15 Rotated Gnomonic Chunks, real-time "
        "elevation relaxation, and multi-way junction editing.</p>");
  });
}

void RoadEditorMainWindow::createToolbar() {
  QToolBar *toolbar = addToolBar("Tools");
  toolbar->setMovable(false);
  toolbar->setIconSize(QSize(24, 24));

  auto resolveIcon = [](const QString &relPath) -> QIcon {
    if (QFile::exists(relPath))
      return QIcon(relPath);
    QString appRel = QCoreApplication::applicationDirPath() + "/" + relPath;
    if (QFile::exists(appRel))
      return QIcon(appRel);
    QString parentRel =
        QCoreApplication::applicationDirPath() + "/../" + relPath;
    if (QFile::exists(parentRel))
      return QIcon(parentRel);
    QString srcRel =
        QString("/Users/chris/PersonalProjects/ForceEngine/") + relPath;
    if (QFile::exists(srcRel))
      return QIcon(srcRel);
    return QIcon(relPath);
  };

  // 1. Label "Tools" on the left of the top bar where the tools live
  QLabel *lblTools = new QLabel("Tools", toolbar);
  lblTools->setStyleSheet(
      "font-weight: bold; margin-left: 6px; margin-right: 6px;");
  toolbar->addWidget(lblTools);

  QActionGroup *toolGroup = new QActionGroup(this);
  toolGroup->setExclusive(true);

  // Pan Tool (hand icon)
  m_actPan = new QAction(resolveIcon("assets/icons/pan.svg"), "Pan", toolGroup);
  m_actPan->setCheckable(true);
  m_actPan->setChecked(false);
  m_actPan->setToolTip(
      "Pan Tool: Press either mouse button (left or right) to pan the view.");
  toolbar->addAction(m_actPan);

  // Select Tool (cursor icon)
  m_actSelect =
      new QAction(resolveIcon("assets/icons/select.svg"), "Select", toolGroup);
  m_actSelect->setCheckable(true);
  m_actSelect->setChecked(true);
  m_actSelect->setToolTip("Select Tool: Click to select roads, junctions, or "
                          "control points. Right-click drag to pan.");
  toolbar->addAction(m_actSelect);

  connect(toolGroup, &QActionGroup::triggered, this, [this](QAction *action) {
    if (action == m_actPan) {
      m_viewport->getRoadEditor().setActiveTool(EditorTool::Pan);
      m_viewport->setCursor(Qt::OpenHandCursor);
    } else {
      m_viewport->getRoadEditor().setActiveTool(EditorTool::Select);
      m_viewport->setCursor(Qt::ArrowCursor);
    }
  });

  // 2. Visual divider to the right of the "Select" tool
  toolbar->addSeparator();

  // 3. After the divider add a label "Actions"
  QLabel *lblActions = new QLabel("Actions", toolbar);
  lblActions->setStyleSheet(
      "font-weight: bold; margin-left: 6px; margin-right: 6px;");
  toolbar->addWidget(lblActions);

  // 4. Action button 1: Add Junction
  m_actAddJunction = new QAction(resolveIcon("assets/icons/add_junction.png"),
                                 "Create Junction", toolbar);
  m_actAddJunction->setCheckable(true);
  m_actAddJunction->setChecked(false);
  m_actAddJunction->setToolTip("Create Junction: Click road segment endpoints "
                               "in 3D to connect them into a junction.");
  connect(m_actAddJunction, &QAction::toggled, this,
          &RoadEditorMainWindow::onToggleAddJunction);
  toolbar->addAction(m_actAddJunction);

  // 4. Action button 2: Add Segment
  m_actAddSegment = new QAction(resolveIcon("assets/icons/add_segment.png"),
                                "New Road Segment", toolbar);
  m_actAddSegment->setToolTip("New Road Segment: Create a new procedural road "
                              "segment in the active chunk.");
  connect(m_actAddSegment, &QAction::triggered, this,
          &RoadEditorMainWindow::onCreateNewSegment);
  toolbar->addAction(m_actAddSegment);

  // 4. Action button 3: Test Drive Current Segment
  m_actTestDrive =
      new QAction(resolveIcon("assets/icons/test_drive_current_segment.svg"),
                  "Test Drive Selected Road", toolbar);
  m_actTestDrive->setToolTip(
      "Test Drive Selected Road: Drive vehicle along selected road segment "
      "(WASD to steer/throttle, ESC to exit).");
  connect(m_actTestDrive, &QAction::triggered, this,
          &RoadEditorMainWindow::onTestDriveSelectedRoad);
  toolbar->addAction(m_actTestDrive);
}

namespace {
class ContextualStackedWidget : public QStackedWidget {
public:
  using QStackedWidget::QStackedWidget;

  QSize sizeHint() const override {
    if (currentWidget()) {
      return currentWidget()->sizeHint();
    }
    return QStackedWidget::sizeHint();
  }

  QSize minimumSizeHint() const override {
    if (currentWidget()) {
      return currentWidget()->minimumSizeHint();
    }
    return QStackedWidget::minimumSizeHint();
  }
};
} // namespace

void RoadEditorMainWindow::createInspectorDock() {
  m_inspectorDock = new QDockWidget("Inspector", this);
  m_inspectorDock->setAllowedAreas(Qt::RightDockWidgetArea |
                                   Qt::LeftDockWidgetArea);

  m_inspectorScrollArea = new QScrollArea(m_inspectorDock);
  m_inspectorScrollArea->setWidgetResizable(true);
  m_inspectorScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QWidget *container = new QWidget(m_inspectorScrollArea);
  QVBoxLayout *mainLayout = new QVBoxLayout(container);
  mainLayout->setContentsMargins(6, 6, 6, 6);

  m_inspectorStack = new ContextualStackedWidget(container);

  // -------------------------------------------------------------
  // Page 0: Placeholder (Nothing Selected)
  // -------------------------------------------------------------
  m_placeholderWidget = new QWidget();
  m_placeholderWidget->setSizePolicy(QSizePolicy::Preferred,
                                     QSizePolicy::Preferred);
  QVBoxLayout *phLayout = new QVBoxLayout(m_placeholderWidget);
  phLayout->addStretch();
  QLabel *phLabel =
      new QLabel("Select a road segment or junction to show controls",
                 m_placeholderWidget);
  phLabel->setAlignment(Qt::AlignCenter);
  phLabel->setWordWrap(true);
  phLabel->setStyleSheet("color: gray; font-style: italic; padding: 20px;");
  phLayout->addWidget(phLabel);
  phLayout->addStretch();
  m_inspectorStack->addWidget(m_placeholderWidget);

  // -------------------------------------------------------------
  // Page 1: Road Segment Controls
  // -------------------------------------------------------------
  m_roadWidget = new QWidget();
  QVBoxLayout *roadLayout = new QVBoxLayout(m_roadWidget);
  roadLayout->setContentsMargins(0, 0, 0, 0);

  m_lblRoadName = new QLabel("Road Segment", m_roadWidget);
  QFont titleFont = m_lblRoadName->font();
  titleFont.setBold(true);
  titleFont.setPointSize(titleFont.pointSize() + 1);
  m_lblRoadName->setFont(titleFont);
  roadLayout->addWidget(m_lblRoadName);

  m_lblRoadClass = new QLabel("Classification: -", m_roadWidget);
  roadLayout->addWidget(m_lblRoadClass);

  m_btnDeleteSegment =
      new QPushButton("Delete Road Segment [Del]", m_roadWidget);
  connect(m_btnDeleteSegment, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onDeleteSelectedSegment);
  roadLayout->addWidget(m_btnDeleteSegment);

  QFormLayout *segForm = new QFormLayout();
  m_spinLanes = new QSpinBox(m_roadWidget);
  m_spinLanes->setRange(1, 8);
  m_spinLanes->setValue(2);
  connect(m_spinLanes, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &RoadEditorMainWindow::onLanesChanged);
  segForm->addRow("Lanes:", m_spinLanes);

  m_chkOneWay = new QCheckBox("One-Way", m_roadWidget);
  connect(m_chkOneWay, &QCheckBox::checkStateChanged, this,
          [this](Qt::CheckState state) {
            onOneWayChanged(state == Qt::Checked ? 1 : 0);
          });
  segForm->addRow("", m_chkOneWay);

  m_spinLaneWidth = new QDoubleSpinBox(m_roadWidget);
  m_spinLaneWidth->setRange(2.0, 6.0);
  m_spinLaneWidth->setSingleStep(0.25);
  m_spinLaneWidth->setValue(3.65);
  m_spinLaneWidth->setSuffix(" m");
  connect(m_spinLaneWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &RoadEditorMainWindow::onLaneWidthChanged);
  segForm->addRow("Lane Width:", m_spinLaneWidth);

  m_spinShoulderWidth = new QDoubleSpinBox(m_roadWidget);
  m_spinShoulderWidth->setRange(0.0, 4.0);
  m_spinShoulderWidth->setSingleStep(0.25);
  m_spinShoulderWidth->setValue(1.50);
  m_spinShoulderWidth->setSuffix(" m");
  connect(m_spinShoulderWidth,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &RoadEditorMainWindow::onShoulderWidthChanged);
  segForm->addRow("Shoulder Width:", m_spinShoulderWidth);

  m_spinSampleStep = new QDoubleSpinBox(m_roadWidget);
  m_spinSampleStep->setRange(1.0, 10.0);
  m_spinSampleStep->setSingleStep(0.5);
  m_spinSampleStep->setValue(2.50);
  m_spinSampleStep->setSuffix(" m");
  connect(m_spinSampleStep,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &RoadEditorMainWindow::onSampleStepChanged);
  segForm->addRow("Sample Step:", m_spinSampleStep);

  m_spinLayer = new QSpinBox(m_roadWidget);
  m_spinLayer->setRange(-3, 3);
  m_spinLayer->setValue(0);
  connect(m_spinLayer, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &RoadEditorMainWindow::onLayerChanged);
  segForm->addRow("Layer / Level:", m_spinLayer);

  m_comboBridgeTunnel = new QComboBox(m_roadWidget);
  m_comboBridgeTunnel->addItem("None (At-Grade)");
  m_comboBridgeTunnel->addItem("Bridge / Overpass");
  m_comboBridgeTunnel->addItem("Tunnel / Underpass");
  connect(m_comboBridgeTunnel,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &RoadEditorMainWindow::onBridgeTunnelChanged);
  segForm->addRow("Structure:", m_comboBridgeTunnel);
  roadLayout->addLayout(segForm);

  // Control Point Tools Group
  QGroupBox *cpGroup = new QGroupBox("Control Point Tools", m_roadWidget);
  QVBoxLayout *cpLayout = new QVBoxLayout(cpGroup);
  m_lblControlPointInfo =
      new QLabel("Click a control point in 3D to edit", cpGroup);
  cpLayout->addWidget(m_lblControlPointInfo);

  QHBoxLayout *elevRow = new QHBoxLayout();
  elevRow->addWidget(new QLabel("Elevation:", cpGroup));
  m_spinCpElevation = new QDoubleSpinBox(cpGroup);
  m_spinCpElevation->setRange(-500.0, 5000.0);
  m_spinCpElevation->setSingleStep(0.2);
  m_spinCpElevation->setSuffix(" m");
  connect(m_spinCpElevation,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &RoadEditorMainWindow::onCpElevationChanged);
  elevRow->addWidget(m_spinCpElevation);
  cpLayout->addLayout(elevRow);

  QHBoxLayout *cpBtnRow = new QHBoxLayout();
  m_btnInsertBefore = new QPushButton("Insert Point Before", cpGroup);
  m_btnInsertAfter = new QPushButton("Insert Point After", cpGroup);
  connect(m_btnInsertBefore, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onInsertPointBefore);
  connect(m_btnInsertAfter, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onInsertPointAfter);
  cpBtnRow->addWidget(m_btnInsertBefore);
  cpBtnRow->addWidget(m_btnInsertAfter);
  cpLayout->addLayout(cpBtnRow);

  m_btnSplitSegment = new QPushButton("Split Segment Here", cpGroup);
  connect(m_btnSplitSegment, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onSplitSegment);
  cpLayout->addWidget(m_btnSplitSegment);

  m_btnDeleteCp = new QPushButton("Delete Control Point", cpGroup);
  connect(m_btnDeleteCp, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onDeleteControlPoint);
  cpLayout->addWidget(m_btnDeleteCp);
  roadLayout->addWidget(cpGroup);

  // Junction Connections Group
  QGroupBox *jcGroup = new QGroupBox("Junction Connections", m_roadWidget);
  QVBoxLayout *jcLayout = new QVBoxLayout(jcGroup);

  m_lblStartJuncInfo = new QLabel("Start: Detached", jcGroup);
  jcLayout->addWidget(m_lblStartJuncInfo);
  QHBoxLayout *sjRow = new QHBoxLayout();
  m_btnDisconnectStartJunc = new QPushButton("Disconnect Start", jcGroup);
  m_btnInspectStartJunc = new QPushButton("Inspect Junc", jcGroup);
  connect(m_btnDisconnectStartJunc, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onDisconnectStartJunction);
  connect(m_btnInspectStartJunc, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onInspectStartJunction);
  sjRow->addWidget(m_btnDisconnectStartJunc);
  sjRow->addWidget(m_btnInspectStartJunc);
  jcLayout->addLayout(sjRow);

  m_lblEndJuncInfo = new QLabel("End: Detached", jcGroup);
  jcLayout->addWidget(m_lblEndJuncInfo);
  QHBoxLayout *ejRow = new QHBoxLayout();
  m_btnDisconnectEndJunc = new QPushButton("Disconnect End", jcGroup);
  m_btnInspectEndJunc = new QPushButton("Inspect Junc", jcGroup);
  connect(m_btnDisconnectEndJunc, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onDisconnectEndJunction);
  connect(m_btnInspectEndJunc, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onInspectEndJunction);
  ejRow->addWidget(m_btnDisconnectEndJunc);
  ejRow->addWidget(m_btnInspectEndJunc);
  jcLayout->addLayout(ejRow);
  roadLayout->addWidget(jcGroup);

  roadLayout->addStretch();
  m_inspectorStack->addWidget(m_roadWidget);

  // -------------------------------------------------------------
  // Page 2: Junction Controls
  // -------------------------------------------------------------
  m_junctionWidget = new QWidget();
  QVBoxLayout *juncLayout = new QVBoxLayout(m_junctionWidget);
  juncLayout->setContentsMargins(0, 0, 0, 0);

  m_lblJunctionHeader = new QLabel("Junction", m_junctionWidget);
  m_lblJunctionHeader->setFont(titleFont);
  juncLayout->addWidget(m_lblJunctionHeader);

  m_lblJunctionPos = new QLabel("Position: -", m_junctionWidget);
  juncLayout->addWidget(m_lblJunctionPos);

  QHBoxLayout *juncTopBtns = new QHBoxLayout();
  m_btnRemoveJunction = new QPushButton("Remove Junction [Del]", m_junctionWidget);
  connect(m_btnRemoveJunction, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onRemoveSelectedJunction);
  m_btnDeselectJunction = new QPushButton("Deselect", m_junctionWidget);
  connect(m_btnDeselectJunction, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onDeselectJunction);
  juncTopBtns->addWidget(m_btnRemoveJunction);
  juncTopBtns->addWidget(m_btnDeselectJunction);
  juncLayout->addLayout(juncTopBtns);

  juncLayout->addWidget(
      new QLabel("Connected Road Segments:", m_junctionWidget));
  m_tableConnectedRoads = new QTableWidget(0, 3, m_junctionWidget);
  m_tableConnectedRoads->setHorizontalHeaderLabels(
      {"Segment", "End", "Action"});
  m_tableConnectedRoads->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::Stretch);
  m_tableConnectedRoads->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::ResizeToContents);
  m_tableConnectedRoads->horizontalHeader()->setSectionResizeMode(
      2, QHeaderView::ResizeToContents);
  juncLayout->addWidget(m_tableConnectedRoads);

  juncLayout->addWidget(new QLabel("All Chunk Junctions:", m_junctionWidget));
  m_listAllJunctions = new QListWidget(m_junctionWidget);
  connect(m_listAllJunctions, &QListWidget::currentRowChanged, this,
          &RoadEditorMainWindow::onJunctionListClicked);
  juncLayout->addWidget(m_listAllJunctions);

  m_btnAddJunctionInTab =
      new QPushButton("+ Add Junction Tool", m_junctionWidget);
  m_btnAddJunctionInTab->setCheckable(true);
  connect(m_btnAddJunctionInTab, &QPushButton::toggled, this,
          &RoadEditorMainWindow::onToggleAddJunction);
  juncLayout->addWidget(m_btnAddJunctionInTab);

  juncLayout->addStretch();
  m_inspectorStack->addWidget(m_junctionWidget);

  mainLayout->addWidget(m_inspectorStack);
  m_inspectorScrollArea->setWidget(container);
  m_inspectorDock->setWidget(m_inspectorScrollArea);
  addDockWidget(Qt::RightDockWidgetArea, m_inspectorDock);
}

void RoadEditorMainWindow::createSolverDock() {
  m_solverDock = new QDockWidget("Elevation Solver", this);
  m_solverDock->setAllowedAreas(Qt::RightDockWidgetArea |
                                Qt::LeftDockWidgetArea);

  QScrollArea *scrollArea = new QScrollArea(m_solverDock);
  scrollArea->setWidgetResizable(true);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QWidget *container = new QWidget(scrollArea);
  QVBoxLayout *solverLayout = new QVBoxLayout(container);
  solverLayout->setContentsMargins(6, 6, 6, 6);

  QGroupBox *forcesGroup =
      new QGroupBox("Physical Forces & Constraints", container);
  QVBoxLayout *forcesLayout = new QVBoxLayout(forcesGroup);

  // Tension
  forcesLayout->addWidget(new QLabel("Tension (Stiffness):", forcesGroup));
  QHBoxLayout *rowTension = new QHBoxLayout();
  m_sliderTension = new QSlider(Qt::Horizontal, forcesGroup);
  m_sliderTension->setRange(0, 400);
  m_lblTensionVal = new QLabel("100.0", forcesGroup);
  m_lblTensionVal->setFixedWidth(40);
  connect(m_sliderTension, &QSlider::valueChanged, this,
          &RoadEditorMainWindow::onSolverTensionChanged);
  rowTension->addWidget(m_sliderTension);
  rowTension->addWidget(m_lblTensionVal);
  forcesLayout->addLayout(rowTension);

  // Adhesion
  forcesLayout->addWidget(new QLabel("Adhesion (Terrain Pull):", forcesGroup));
  QHBoxLayout *rowAdhesion = new QHBoxLayout();
  m_sliderAdhesion = new QSlider(Qt::Horizontal, forcesGroup);
  m_sliderAdhesion->setRange(0, 500);
  m_lblAdhesionVal = new QLabel("10.00", forcesGroup);
  m_lblAdhesionVal->setFixedWidth(40);
  connect(m_sliderAdhesion, &QSlider::valueChanged, this,
          &RoadEditorMainWindow::onSolverAdhesionChanged);
  rowAdhesion->addWidget(m_sliderAdhesion);
  rowAdhesion->addWidget(m_lblAdhesionVal);
  forcesLayout->addLayout(rowAdhesion);

  // Repulsion
  forcesLayout->addWidget(new QLabel("Repulsion (Clearance):", forcesGroup));
  QHBoxLayout *rowRepulsion = new QHBoxLayout();
  m_sliderRepulsion = new QSlider(Qt::Horizontal, forcesGroup);
  m_sliderRepulsion->setRange(0, 500);
  m_lblRepulsionVal = new QLabel("200.0", forcesGroup);
  m_lblRepulsionVal->setFixedWidth(40);
  connect(m_sliderRepulsion, &QSlider::valueChanged, this,
          &RoadEditorMainWindow::onSolverRepulsionChanged);
  rowRepulsion->addWidget(m_sliderRepulsion);
  rowRepulsion->addWidget(m_lblRepulsionVal);
  forcesLayout->addLayout(rowRepulsion);

  // Clearance Height
  forcesLayout->addWidget(new QLabel("Clearance Height:", forcesGroup));
  QHBoxLayout *rowClearance = new QHBoxLayout();
  m_sliderClearance = new QSlider(Qt::Horizontal, forcesGroup);
  m_sliderClearance->setRange(0, 150);
  m_lblClearanceVal = new QLabel("5.5m", forcesGroup);
  m_lblClearanceVal->setFixedWidth(40);
  connect(m_sliderClearance, &QSlider::valueChanged, this,
          &RoadEditorMainWindow::onSolverClearanceChanged);
  rowClearance->addWidget(m_sliderClearance);
  rowClearance->addWidget(m_lblClearanceVal);
  forcesLayout->addLayout(rowClearance);

  // Relaxation Sweeps
  forcesLayout->addWidget(new QLabel("Relaxation Sweeps:", forcesGroup));
  QHBoxLayout *rowRelax = new QHBoxLayout();
  m_sliderRelaxation = new QSlider(Qt::Horizontal, forcesGroup);
  m_sliderRelaxation->setRange(1, 500);
  m_lblRelaxationVal = new QLabel("200", forcesGroup);
  m_lblRelaxationVal->setFixedWidth(40);
  connect(m_sliderRelaxation, &QSlider::valueChanged, this,
          &RoadEditorMainWindow::onSolverRelaxationChanged);
  rowRelax->addWidget(m_sliderRelaxation);
  rowRelax->addWidget(m_lblRelaxationVal);
  forcesLayout->addLayout(rowRelax);

  solverLayout->addWidget(forcesGroup);

  m_chkAutoSolve = new QCheckBox("Live Auto-Solve on Edit", container);
  m_chkAutoSolve->setChecked(false); // Disabled by default as requested!
  connect(m_chkAutoSolve, &QCheckBox::toggled, this,
          &RoadEditorMainWindow::onLiveAutoSolveToggled);
  solverLayout->addWidget(m_chkAutoSolve);

  QHBoxLayout *solverBtnRow = new QHBoxLayout();
  QPushButton *btnSolveNow = new QPushButton("Solve Now", container);
  QPushButton *btnResetSolver = new QPushButton("Reset Defaults", container);
  connect(btnSolveNow, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onSolveElevationNow);
  connect(btnResetSolver, &QPushButton::clicked, this,
          &RoadEditorMainWindow::onResetSolverDefaults);
  solverBtnRow->addWidget(btnSolveNow);
  solverBtnRow->addWidget(btnResetSolver);
  solverLayout->addLayout(solverBtnRow);

  m_lblSolverStats = new QLabel("Ready", container);
  solverLayout->addWidget(m_lblSolverStats);

  solverLayout->addStretch();
  scrollArea->setWidget(container);
  m_solverDock->setWidget(scrollArea);
  addDockWidget(Qt::RightDockWidgetArea, m_solverDock);
}

void RoadEditorMainWindow::createStatsDialog() {
  m_statsDialog = new QDialog(this);
  m_statsDialog->setWindowTitle("Network Statistics");
  m_statsDialog->resize(320, 220);

  QVBoxLayout *layout = new QVBoxLayout(m_statsDialog);
  QFormLayout *form = new QFormLayout();

  m_lblStatTotalSegments = new QLabel("0", m_statsDialog);
  m_lblStatActiveSegments = new QLabel("0", m_statsDialog);
  m_lblStatTotalJunctions = new QLabel("0", m_statsDialog);
  m_lblStatVertices = new QLabel("0", m_statsDialog);
  m_lblStatTriangles = new QLabel("0", m_statsDialog);

  form->addRow("Total Road Segments:", m_lblStatTotalSegments);
  form->addRow("In Active Chunk:", m_lblStatActiveSegments);
  form->addRow("Total Junctions:", m_lblStatTotalJunctions);
  form->addRow("Road Mesh Vertices:", m_lblStatVertices);
  form->addRow("Road Mesh Triangles:", m_lblStatTriangles);
  layout->addLayout(form);

  layout->addStretch();
  QDialogButtonBox *btnBox =
      new QDialogButtonBox(QDialogButtonBox::Close, m_statsDialog);
  connect(btnBox, &QDialogButtonBox::rejected, m_statsDialog, &QDialog::close);
  layout->addWidget(btnBox);
}

void RoadEditorMainWindow::createStatusBar() {
  QStatusBar *bar = statusBar();

  m_statusMsgLabel = new QLabel("Ready", bar);
  m_statusGpsLabel = new QLabel("GPS: -", bar);
  m_statusChunkLabel = new QLabel("Chunk: -", bar);
  m_statusFpsLabel = new QLabel("60 FPS (16.6 ms)", bar);

  bar->addWidget(m_statusMsgLabel, 1);
  bar->addPermanentWidget(m_statusGpsLabel);
  bar->addPermanentWidget(m_statusChunkLabel);
  bar->addPermanentWidget(m_statusFpsLabel);
}

void RoadEditorMainWindow::updateStatusBarInfo() {
  auto &re = m_viewport->getRoadEditor();

  // Chunk label matching wireframe sketch: Chunk: [2, 16040, 20958]
  if (m_statusChunkLabel) {
    m_statusChunkLabel->setText(QString("Chunk: [%1, %2, %3]")
                                    .arg(re.getActiveChunkFace())
                                    .arg(re.getActiveChunkX())
                                    .arg(re.getActiveChunkZ()));
  }

  // GPS label matching wireframe sketch: GPS: 40.59001° N, 74.52346° W
  glm::dvec3 ecef = re.getChunkCenterECEF();
  if (glm::length(ecef) > 1000.0) {
    double lat = 0.0, lon = 0.0, dummyH = 0.0;
    Quadsphere::ecefToGeodetic(ecef, lat, lon, dummyH);
    if (m_statusGpsLabel) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(5) << "GPS: " << std::abs(lat)
         << (lat >= 0 ? "° N, " : "° S, ") << std::abs(lon)
         << (lon >= 0 ? "° E" : "° W");
      m_statusGpsLabel->setText(QString::fromStdString(ss.str()));
    }
  }
}

void RoadEditorMainWindow::updateStatsDialog() {
  if (!m_statsDialog)
    return;
  auto &re = m_viewport->getRoadEditor();
  if (m_lblStatTotalSegments)
    m_lblStatTotalSegments->setText(QString::number(re.getTotalSegments()));
  if (m_lblStatActiveSegments)
    m_lblStatActiveSegments->setText(QString::number(re.getTotalSegments()));
  if (m_lblStatTotalJunctions)
    m_lblStatTotalJunctions->setText(QString::number(re.getTotalJunctions()));
  if (m_lblStatVertices)
    m_lblStatVertices->setText(QString::number(re.getMeshVertexCount()));
  if (m_lblStatTriangles)
    m_lblStatTriangles->setText(QString::number(re.getMeshTriangleCount()));
}

void RoadEditorMainWindow::updateRoadInspector(int segIdx, int cpIdx) {
  auto &re = m_viewport->getRoadEditor();
  const auto *seg = re.getSelectedSegment();
  if (!seg)
    return;

  m_updatingUi = true;

  QString nameStr = seg->name.empty() ? QString("Segment #%1").arg(segIdx)
                                      : QString::fromStdString(seg->name);
  m_lblRoadName->setText(nameStr);
  m_lblRoadClass->setText(QString("Classification: %1 | %2 Lanes")
                              .arg(QString::fromStdString(seg->highway))
                              .arg(seg->lanes));

  m_spinLanes->setValue(seg->lanes);
  m_chkOneWay->setChecked(seg->oneway);
  m_spinLaneWidth->setValue(seg->laneWidth);
  m_spinShoulderWidth->setValue(seg->shoulderWidth);
  m_spinSampleStep->setValue(seg->sampleStep);
  m_spinLayer->setValue(seg->layer);

  if (seg->isBridge)
    m_comboBridgeTunnel->setCurrentIndex(1);
  else if (seg->isTunnel)
    m_comboBridgeTunnel->setCurrentIndex(2);
  else
    m_comboBridgeTunnel->setCurrentIndex(0);

  // Control point readout
  if (cpIdx >= 0 && cpIdx < static_cast<int>(seg->controlPoints.size())) {
    glm::vec3 cp = seg->controlPoints[cpIdx];
    m_lblControlPointInfo->setText(
        QString("Control Point #%1 of %2\n(%3, %4, %5)")
            .arg(cpIdx)
            .arg(seg->controlPoints.size())
            .arg(cp.x, 0, 'f', 1)
            .arg(cp.y, 0, 'f', 1)
            .arg(cp.z, 0, 'f', 1));
    m_spinCpElevation->setEnabled(true);
    m_spinCpElevation->setValue(cp.y);
    m_btnInsertBefore->setEnabled(true);
    m_btnInsertAfter->setEnabled(true);
    m_btnSplitSegment->setEnabled(
        cpIdx > 0 && cpIdx < static_cast<int>(seg->controlPoints.size()) - 1);
    m_btnDeleteCp->setEnabled(seg->controlPoints.size() > 2);
  } else {
    m_lblControlPointInfo->setText(
        QString("Road has %1 control points. Click one in 3D to edit.")
            .arg(seg->controlPoints.size()));
    m_spinCpElevation->setEnabled(false);
    m_btnInsertBefore->setEnabled(true);
    m_btnInsertAfter->setEnabled(true);
    m_btnSplitSegment->setEnabled(false);
    m_btnDeleteCp->setEnabled(false);
  }

  // Start Junction Card
  if (seg->startJunctionId >= 0 &&
      seg->startJunctionId < static_cast<int>(re.getJunctions().size())) {
    const auto &junc = re.getJunctions()[seg->startJunctionId];
    m_lblStartJuncInfo->setText(QString("Start Junc #%1 (%2-way)")
                                    .arg(seg->startJunctionId)
                                    .arg(junc.connections.size()));
    m_btnDisconnectStartJunc->setEnabled(true);
    m_btnInspectStartJunc->setEnabled(true);
  } else {
    m_lblStartJuncInfo->setText("Start: Detached / Dead-end");
    m_btnDisconnectStartJunc->setEnabled(false);
    m_btnInspectStartJunc->setEnabled(false);
  }

  // End Junction Card
  if (seg->endJunctionId >= 0 &&
      seg->endJunctionId < static_cast<int>(re.getJunctions().size())) {
    const auto &junc = re.getJunctions()[seg->endJunctionId];
    m_lblEndJuncInfo->setText(QString("End Junc #%1 (%2-way)")
                                  .arg(seg->endJunctionId)
                                  .arg(junc.connections.size()));
    m_btnDisconnectEndJunc->setEnabled(true);
    m_btnInspectEndJunc->setEnabled(true);
  } else {
    m_lblEndJuncInfo->setText("End: Detached / Dead-end");
    m_btnDisconnectEndJunc->setEnabled(false);
    m_btnInspectEndJunc->setEnabled(false);
  }

  m_updatingUi = false;
}

void RoadEditorMainWindow::updateJunctionInspector(int juncIdx) {
  auto &re = m_viewport->getRoadEditor();
  const auto *junc = re.getSelectedJunction();
  if (!junc)
    return;

  m_updatingUi = true;

  m_lblJunctionHeader->setText(QString("Junction #%1 (%2-way)")
                                   .arg(juncIdx)
                                   .arg(junc->connections.size()));
  m_lblJunctionPos->setText(QString("Position: (%1, %2, %3)")
                                .arg(junc->position.x, 0, 'f', 1)
                                .arg(junc->position.y, 0, 'f', 1)
                                .arg(junc->position.z, 0, 'f', 1));

  m_tableConnectedRoads->setRowCount(
      static_cast<int>(junc->connections.size()));
  for (int i = 0; i < static_cast<int>(junc->connections.size()); ++i) {
    const auto &conn = junc->connections[i];
    QString segName = QString("Seg #%1").arg(conn.segmentIdx);
    if (conn.segmentIdx >= 0 &&
        conn.segmentIdx < static_cast<int>(re.getSegments().size())) {
      const auto &s = re.getSegments()[conn.segmentIdx];
      if (!s.name.empty())
        segName = QString::fromStdString(s.name);
    }

    m_tableConnectedRoads->setItem(i, 0, new QTableWidgetItem(segName));
    m_tableConnectedRoads->setItem(
        i, 1, new QTableWidgetItem(conn.isStart ? "Start" : "End"));

    QPushButton *btnDis = new QPushButton("Disconnect", m_tableConnectedRoads);
    int targetSeg = conn.segmentIdx;
    bool targetIsStart = conn.isStart;
    connect(btnDis, &QPushButton::clicked, [this, targetSeg, targetIsStart]() {
      m_viewport->getRoadEditor().disconnectSegmentFromJunction(targetSeg,
                                                                targetIsStart);
      updateJunctionInspector(
          m_viewport->getRoadEditor().getSelectedJunctionIdx());
    });
    m_tableConnectedRoads->setCellWidget(i, 2, btnDis);
  }

  m_listAllJunctions->clear();
  const auto &allJuncs = re.getJunctions();
  for (size_t i = 0; i < allJuncs.size(); ++i) {
    m_listAllJunctions->addItem(QString("Junction #%1 (%2-way)")
                                    .arg(i)
                                    .arg(allJuncs[i].connections.size()));
  }

  m_updatingUi = false;
}

void RoadEditorMainWindow::updateSolverPanel() {
  auto &opt = m_viewport->getRoadEditor().getSolverOptions();
  m_updatingUi = true;
  m_sliderTension->setValue(static_cast<int>(opt.k_tension));
  m_lblTensionVal->setText(QString::number(opt.k_tension, 'f', 1));
  m_sliderAdhesion->setValue(static_cast<int>(opt.k_adhesion * 10.0f));
  m_lblAdhesionVal->setText(QString::number(opt.k_adhesion, 'f', 2));
  m_sliderRepulsion->setValue(static_cast<int>(opt.k_repulsion));
  m_lblRepulsionVal->setText(QString::number(opt.k_repulsion, 'f', 1));
  m_sliderClearance->setValue(static_cast<int>(opt.minClearance * 10.0f));
  m_lblClearanceVal->setText(QString("%1m").arg(opt.minClearance, 0, 'f', 1));
  m_sliderRelaxation->setValue(opt.maxSweeps);
  m_lblRelaxationVal->setText(QString::number(opt.maxSweeps));
  m_updatingUi = false;
}

void RoadEditorMainWindow::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
    if (m_viewport && m_viewport->getRoadEditor().isAddingJunction()) {
      m_viewport->getRoadEditor().createJunctionFromPending();
      event->accept();
      return;
    }
  } else if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
    onDeleteSelected();
    event->accept();
    return;
  }

  // Forward navigation & editor keys to viewport if focus is not in an editable field
  QWidget *fw = focusWidget();
  bool isEditing = fw && (fw->inherits("QLineEdit") || fw->inherits("QAbstractSpinBox") ||
                          fw->inherits("QTextEdit") || fw->inherits("QPlainTextEdit"));
  if (fw != m_viewport && !isEditing && m_viewport) {
    QCoreApplication::sendEvent(m_viewport, event);
    if (event->isAccepted()) {
      return;
    }
  }

  QMainWindow::keyPressEvent(event);
}

void RoadEditorMainWindow::keyReleaseEvent(QKeyEvent *event) {
  QWidget *fw = focusWidget();
  bool isEditing = fw && (fw->inherits("QLineEdit") || fw->inherits("QAbstractSpinBox") ||
                          fw->inherits("QTextEdit") || fw->inherits("QPlainTextEdit"));
  if (fw != m_viewport && !isEditing && m_viewport) {
    QCoreApplication::sendEvent(m_viewport, event);
    if (event->isAccepted()) {
      return;
    }
  }

  QMainWindow::keyReleaseEvent(event);
}

// -------------------------------------------------------------
// Slots
// -------------------------------------------------------------

void RoadEditorMainWindow::onSelectionChanged(int segmentIdx, int junctionIdx,
                                              int controlPointIdx) {
  if (m_updatingUi)
    return;

  if (segmentIdx >= 0) {
    if (m_inspectorScrollArea) {
      m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    }
    m_inspectorStack->setCurrentWidget(m_roadWidget);
    m_inspectorStack->updateGeometry();
    updateRoadInspector(segmentIdx, controlPointIdx);
  } else if (junctionIdx >= 0) {
    if (m_inspectorScrollArea) {
      m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    }
    m_inspectorStack->setCurrentWidget(m_junctionWidget);
    m_inspectorStack->updateGeometry();
    updateJunctionInspector(junctionIdx);
  } else {
    // Nothing selected -> show contextual placeholder without scrollbar
    if (m_inspectorScrollArea) {
      m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      m_inspectorScrollArea->verticalScrollBar()->setValue(0);
    }
    m_inspectorStack->setCurrentWidget(m_placeholderWidget);
    m_inspectorStack->updateGeometry();
  }
}

void RoadEditorMainWindow::onStatusChanged(const QString &status) {
  if (m_statusMsgLabel)
    m_statusMsgLabel->setText(status);
  bool adding = m_viewport->getRoadEditor().isAddingJunction();
  if (m_actAddJunction && m_actAddJunction->isChecked() != adding) {
    m_actAddJunction->setChecked(adding);
  }
  if (m_btnAddJunctionInTab && m_btnAddJunctionInTab->isChecked() != adding) {
    m_btnAddJunctionInTab->setChecked(adding);
  }
}

void RoadEditorMainWindow::onFrameRendered(float fps, float frameTimeMs) {
  if (m_statusFpsLabel) {
    m_statusFpsLabel->setText(QString("%1 FPS (%2 ms)")
                                  .arg(fps, 0, 'f', 0)
                                  .arg(frameTimeMs, 0, 'f', 1));
  }
}

void RoadEditorMainWindow::onOpenChunkDialog() {
  auto &re = m_viewport->getRoadEditor();
  ChunkInputDialog dlg(re.getActiveChunkFace(), re.getActiveChunkX(),
                       re.getActiveChunkZ(), re.isQuadsphereChunk(), this);
  if (dlg.exec() == QDialog::Accepted) {
    if (dlg.isQuadsphere()) {
      re.loadChunk(dlg.getFace(), dlg.getTileX(), dlg.getTileY());
    } else {
      re.loadChunk(dlg.getTileX(), dlg.getTileY());
    }
    updateStatusBarInfo();
    updateStatsDialog();
    m_inspectorStack->setCurrentWidget(m_placeholderWidget);
  }
}

void RoadEditorMainWindow::onReloadChunk() {
  m_viewport->getRoadEditor().reloadActiveChunk();
  updateStatusBarInfo();
  updateStatsDialog();
  m_inspectorStack->setCurrentWidget(m_placeholderWidget);
}

void RoadEditorMainWindow::onSaveChunk() {
  m_viewport->getRoadEditor().saveActiveChunk();
}

void RoadEditorMainWindow::onApplyAashto() {
  m_viewport->getRoadEditor().applyAashtoStandards();
  updateStatsDialog();
}

void RoadEditorMainWindow::onToggleAddJunction(bool checked) {
  m_viewport->getRoadEditor().setAddingJunction(checked);
  if (m_actAddJunction && m_actAddJunction->isChecked() != checked) {
    m_actAddJunction->setChecked(checked);
  }
  if (m_btnAddJunctionInTab && m_btnAddJunctionInTab->isChecked() != checked) {
    m_btnAddJunctionInTab->setChecked(checked);
  }
  if (m_statusMsgLabel)
    m_statusMsgLabel->setText(
        checked ? "Tool: Create Junction (Click road ends)" : "Ready");
}

void RoadEditorMainWindow::onCreateNewSegment() {
  m_viewport->getRoadEditor().createNewSegment();
  updateStatusBarInfo();
  updateStatsDialog();
}

void RoadEditorMainWindow::onTestDriveSelectedRoad() {
  m_viewport->getRoadEditor().testDriveSelectedRoad();
}

void RoadEditorMainWindow::onViewModeChanged(int index) {
  EditorViewMode mode = EditorViewMode::Orbit3D;
  if (index == 0)
    mode = EditorViewMode::TopDown2D;
  else if (index == 1)
    mode = EditorViewMode::Orbit3D;
  else if (index == 2)
    mode = EditorViewMode::DriveTest;

  m_viewport->getRoadEditor().setViewMode(mode);
}

void RoadEditorMainWindow::onRecenterOrbit() {
  m_viewport->getRoadEditor().recenterOrbitOnTerrain();
}

void RoadEditorMainWindow::onResetCamera() {
  m_viewport->getRoadEditor().resetCameraFocus();
}

void RoadEditorMainWindow::onShowNetworkStatistics() {
  updateStatsDialog();
  if (m_statsDialog)
    m_statsDialog->show();
}

void RoadEditorMainWindow::onDeleteSelected() {
  auto& re = m_viewport->getRoadEditor();
  int juncIdx = re.getSelectedJunctionIdx();
  int cpIdx = re.getSelectedControlPointIdx();
  int segIdx = re.getSelectedSegmentIdx();

  if (juncIdx >= 0) {
    onRemoveSelectedJunction();
  } else if (cpIdx >= 0 && segIdx >= 0) {
    onDeleteControlPoint();
  } else if (segIdx >= 0) {
    onDeleteSelectedSegment();
  }
}

void RoadEditorMainWindow::onDeleteSelectedSegment() {
  int idx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  if (idx >= 0) {
    m_viewport->getRoadEditor().deleteSegment(idx);
    m_inspectorStack->setCurrentWidget(m_placeholderWidget);
    if (m_inspectorScrollArea) {
      m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      m_inspectorScrollArea->verticalScrollBar()->setValue(0);
    }
    updateStatsDialog();
    updateStatusBarInfo();
  }
}

void RoadEditorMainWindow::onLanesChanged(int lanes) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->lanes = lanes;
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onOneWayChanged(int state) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->oneway = (state == Qt::Checked);
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onLaneWidthChanged(double width) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->laneWidth = static_cast<float>(width);
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onShoulderWidthChanged(double width) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->shoulderWidth = static_cast<float>(width);
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onSampleStepChanged(double step) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->sampleStep = static_cast<float>(step);
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onLayerChanged(int layer) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->layer = layer;
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onBridgeTunnelChanged(int index) {
  if (m_updatingUi)
    return;
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg) {
    seg->isBridge = (index == 1);
    seg->isTunnel = (index == 2);
    m_viewport->getRoadEditor().rebuildSegmentGeometry(
        m_viewport->getRoadEditor().getSelectedSegmentIdx());
  }
}

void RoadEditorMainWindow::onCpElevationChanged(double elev) {
  if (m_updatingUi)
    return;
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  int cpIdx = m_viewport->getRoadEditor().getSelectedControlPointIdx();
  if (segIdx >= 0 && cpIdx >= 0) {
    m_viewport->getRoadEditor().setControlPointElevation(
        segIdx, cpIdx, static_cast<float>(elev));
  }
}

void RoadEditorMainWindow::onInsertPointBefore() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  int cpIdx = m_viewport->getRoadEditor().getSelectedControlPointIdx();
  if (segIdx >= 0) {
    m_viewport->getRoadEditor().insertControlPointBefore(segIdx, cpIdx);
    updateRoadInspector(
        segIdx, m_viewport->getRoadEditor().getSelectedControlPointIdx());
  }
}

void RoadEditorMainWindow::onInsertPointAfter() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  int cpIdx = m_viewport->getRoadEditor().getSelectedControlPointIdx();
  if (segIdx >= 0) {
    m_viewport->getRoadEditor().insertControlPointAfter(segIdx, cpIdx);
    updateRoadInspector(
        segIdx, m_viewport->getRoadEditor().getSelectedControlPointIdx());
  }
}

void RoadEditorMainWindow::onSplitSegment() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  int cpIdx = m_viewport->getRoadEditor().getSelectedControlPointIdx();
  if (segIdx >= 0 && cpIdx > 0) {
    m_viewport->getRoadEditor().splitSegmentAtControlPoint(segIdx, cpIdx);
    updateStatsDialog();
    updateRoadInspector(m_viewport->getRoadEditor().getSelectedSegmentIdx(),
                        -1);
  }
}

void RoadEditorMainWindow::onDeleteControlPoint() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  int cpIdx = m_viewport->getRoadEditor().getSelectedControlPointIdx();
  if (segIdx >= 0 && cpIdx >= 0) {
    m_viewport->getRoadEditor().deleteControlPoint(segIdx, cpIdx);
    updateRoadInspector(segIdx, -1);
  }
}

void RoadEditorMainWindow::onDisconnectStartJunction() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  if (segIdx >= 0) {
    m_viewport->getRoadEditor().disconnectSegmentFromJunction(segIdx, true);
    updateRoadInspector(
        segIdx, m_viewport->getRoadEditor().getSelectedControlPointIdx());
  }
}

void RoadEditorMainWindow::onInspectStartJunction() {
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg && seg->startJunctionId >= 0) {
    m_viewport->getRoadEditor().selectJunction(seg->startJunctionId);
    m_inspectorStack->setCurrentWidget(m_junctionWidget);
    updateJunctionInspector(seg->startJunctionId);
  }
}

void RoadEditorMainWindow::onDisconnectEndJunction() {
  int segIdx = m_viewport->getRoadEditor().getSelectedSegmentIdx();
  if (segIdx >= 0) {
    m_viewport->getRoadEditor().disconnectSegmentFromJunction(segIdx, false);
    updateRoadInspector(
        segIdx, m_viewport->getRoadEditor().getSelectedControlPointIdx());
  }
}

void RoadEditorMainWindow::onInspectEndJunction() {
  auto *seg = m_viewport->getRoadEditor().getSelectedSegment();
  if (seg && seg->endJunctionId >= 0) {
    m_viewport->getRoadEditor().selectJunction(seg->endJunctionId);
    m_inspectorStack->setCurrentWidget(m_junctionWidget);
    updateJunctionInspector(seg->endJunctionId);
  }
}

// Solver Slots
void RoadEditorMainWindow::onSolverTensionChanged(int value) {
  if (m_updatingUi)
    return;
  float v = static_cast<float>(value);
  m_viewport->getRoadEditor().getSolverOptions().k_tension = v;
  m_lblTensionVal->setText(QString::number(v, 'f', 1));
  if (m_chkAutoSolve->isChecked())
    onSolveElevationNow();
}

void RoadEditorMainWindow::onSolverAdhesionChanged(int value) {
  if (m_updatingUi)
    return;
  float v = static_cast<float>(value) / 10.0f;
  m_viewport->getRoadEditor().getSolverOptions().k_adhesion = v;
  m_lblAdhesionVal->setText(QString::number(v, 'f', 2));
  if (m_chkAutoSolve->isChecked())
    onSolveElevationNow();
}

void RoadEditorMainWindow::onSolverRepulsionChanged(int value) {
  if (m_updatingUi)
    return;
  float v = static_cast<float>(value);
  m_viewport->getRoadEditor().getSolverOptions().k_repulsion = v;
  m_lblRepulsionVal->setText(QString::number(v, 'f', 1));
  if (m_chkAutoSolve->isChecked())
    onSolveElevationNow();
}

void RoadEditorMainWindow::onSolverClearanceChanged(int value) {
  if (m_updatingUi)
    return;
  float v = static_cast<float>(value) / 10.0f;
  m_viewport->getRoadEditor().getSolverOptions().minClearance = v;
  m_lblClearanceVal->setText(QString("%1m").arg(v, 0, 'f', 1));
  if (m_chkAutoSolve->isChecked())
    onSolveElevationNow();
}

void RoadEditorMainWindow::onSolverRelaxationChanged(int value) {
  if (m_updatingUi)
    return;
  m_viewport->getRoadEditor().getSolverOptions().maxSweeps = value;
  m_lblRelaxationVal->setText(QString::number(value));
  if (m_chkAutoSolve->isChecked())
    onSolveElevationNow();
}

void RoadEditorMainWindow::onLiveAutoSolveToggled(bool checked) {
  m_viewport->getRoadEditor().setAutoSolveElevation(checked);
}

void RoadEditorMainWindow::onSolveElevationNow() {
  m_viewport->getRoadEditor().triggerElevationSolve();
  int sweeps = m_viewport->getRoadEditor().getLastSolveSweeps();
  float timeMs = m_viewport->getRoadEditor().getLastSolveTimeMs();
  if (m_lblSolverStats) {
    m_lblSolverStats->setText(QString("Last solve: %1 sweeps in %2 ms")
                                  .arg(sweeps)
                                  .arg(timeMs, 0, 'f', 1));
  }
}

void RoadEditorMainWindow::onResetSolverDefaults() {
  auto &opt = m_viewport->getRoadEditor().getSolverOptions();
  opt.k_tension = 100.0f;
  opt.k_adhesion = 10.0f;
  opt.k_repulsion = 200.0f;
  opt.minClearance = 5.5f;
  opt.maxSweeps = 200;
  updateSolverPanel();
  onSolveElevationNow();
}

void RoadEditorMainWindow::onRemoveSelectedJunction() {
  int idx = m_viewport->getRoadEditor().getSelectedJunctionIdx();
  if (idx >= 0) {
    m_viewport->getRoadEditor().removeJunction(idx);
    m_inspectorStack->setCurrentWidget(m_placeholderWidget);
    if (m_inspectorScrollArea) {
      m_inspectorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      m_inspectorScrollArea->verticalScrollBar()->setValue(0);
    }
    updateStatsDialog();
    updateStatusBarInfo();
  }
}

void RoadEditorMainWindow::onDeselectJunction() {
  m_viewport->getRoadEditor().selectJunction(-1);
  m_inspectorStack->setCurrentWidget(m_placeholderWidget);
}

void RoadEditorMainWindow::onJunctionListClicked(int row) {
  if (m_updatingUi)
    return;
  if (row >= 0 &&
      row <
          static_cast<int>(m_viewport->getRoadEditor().getJunctions().size())) {
    m_viewport->getRoadEditor().selectJunction(row);
    m_inspectorStack->setCurrentWidget(m_junctionWidget);
    updateJunctionInspector(row);
  }
}
