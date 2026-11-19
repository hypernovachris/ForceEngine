#include "Road/Qt/ChunkInputDialog.h"
#include "Terrain/QuadsphereMath.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QDialogButtonBox>
#include <cmath>

ChunkInputDialog::ChunkInputDialog(uint8_t currentFace, int currentX, int currentZ, bool isQuadsphere, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Select Chunk to Edit");
    setModal(true);
    resize(420, 360);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Mode Selection Group
    QGroupBox* modeGroup = new QGroupBox("Coordinate System", this);
    QHBoxLayout* modeLayout = new QHBoxLayout(modeGroup);
    m_radioQuadsphere = new QRadioButton("Quadsphere L15 (Planetary / Earth Curvature)", modeGroup);
    m_radioLegacy = new QRadioButton("Flat / UTM 512m", modeGroup);
    if (isQuadsphere) {
        m_radioQuadsphere->setChecked(true);
    } else {
        m_radioLegacy->setChecked(true);
    }
    modeLayout->addWidget(m_radioQuadsphere);
    modeLayout->addWidget(m_radioLegacy);
    mainLayout->addWidget(modeGroup);

    // Quadsphere Group
    QGroupBox* quadGroup = new QGroupBox("Quadsphere Chunk Coordinates", this);
    QFormLayout* quadForm = new QFormLayout(quadGroup);

    m_faceCombo = new QComboBox(quadGroup);
    m_faceCombo->addItem("Face 0 (+X)", 0);
    m_faceCombo->addItem("Face 1 (-X)", 1);
    m_faceCombo->addItem("Face 2 (+Y / North America / NJ)", 2);
    m_faceCombo->addItem("Face 3 (-Y)", 3);
    m_faceCombo->addItem("Face 4 (+Z)", 4);
    m_faceCombo->addItem("Face 5 (-Z)", 5);
    m_faceCombo->setCurrentIndex(currentFace <= 5 ? currentFace : 2);
    quadForm->addRow("Cube Face:", m_faceCombo);

    m_tileXSpin = new QSpinBox(quadGroup);
    m_tileXSpin->setRange(0, 32767);
    m_tileXSpin->setValue(currentX >= 0 ? currentX : 16040);
    quadForm->addRow("Tile X (Column):", m_tileXSpin);

    m_tileYSpin = new QSpinBox(quadGroup);
    m_tileYSpin->setRange(0, 32767);
    m_tileYSpin->setValue(currentZ >= 0 ? currentZ : 20958);
    quadForm->addRow("Tile Y (Row):", m_tileYSpin);

    mainLayout->addWidget(quadGroup);

    // GPS Helper Group
    QGroupBox* gpsGroup = new QGroupBox("GPS Coordinates Quick-Teleport (WGS84)", this);
    QFormLayout* gpsForm = new QFormLayout(gpsGroup);

    m_latSpin = new QDoubleSpinBox(gpsGroup);
    m_latSpin->setRange(-90.0, 90.0);
    m_latSpin->setDecimals(6);
    m_latSpin->setValue(40.590010);
    gpsForm->addRow("Latitude (deg N):", m_latSpin);

    m_lonSpin = new QDoubleSpinBox(gpsGroup);
    m_lonSpin->setRange(-180.0, 180.0);
    m_lonSpin->setDecimals(6);
    m_lonSpin->setValue(-74.523460);
    gpsForm->addRow("Longitude (deg W):", m_lonSpin);

    QHBoxLayout* convertRow = new QHBoxLayout();
    QPushButton* convertBtn = new QPushButton("Calculate Chunk from GPS", gpsGroup);
    m_gpsResultLabel = new QLabel("", gpsGroup);
    m_gpsResultLabel->setStyleSheet("color: #4ec9b0; font-weight: bold;");
    convertRow->addWidget(convertBtn);
    convertRow->addWidget(m_gpsResultLabel);
    gpsForm->addRow(convertRow);

    mainLayout->addWidget(gpsGroup);

    // Dialog Buttons
    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    mainLayout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(convertBtn, &QPushButton::clicked, this, &ChunkInputDialog::onConvertGpsClicked);
    connect(m_radioQuadsphere, &QRadioButton::toggled, this, &ChunkInputDialog::onModeChanged);
    connect(m_radioLegacy, &QRadioButton::toggled, this, &ChunkInputDialog::onModeChanged);
}

void ChunkInputDialog::onConvertGpsClicked() {
    double lat = m_latSpin->value();
    double lon = m_lonSpin->value();
    uint8_t face = 0;
    double u = 0.0, v = 0.0;
    Quadsphere::geodeticToQuadsphere(lat, lon, face, u, v);

    int maxTile = 1 << 15; // 32768
    int tx = std::clamp(static_cast<int>(std::floor((u + 1.0) * 0.5 * maxTile)), 0, maxTile - 1);
    int ty = std::clamp(static_cast<int>(std::floor((v + 1.0) * 0.5 * maxTile)), 0, maxTile - 1);

    m_faceCombo->setCurrentIndex(face);
    m_tileXSpin->setValue(tx);
    m_tileYSpin->setValue(ty);
    m_radioQuadsphere->setChecked(true);

    m_gpsResultLabel->setText(QString("-> Face %1 [%2, %3]").arg(face).arg(tx).arg(ty));
}

void ChunkInputDialog::onModeChanged() {
    bool isQuad = m_radioQuadsphere->isChecked();
    m_faceCombo->setEnabled(isQuad);
}

uint8_t ChunkInputDialog::getFace() const {
    return static_cast<uint8_t>(m_faceCombo->currentIndex());
}

int ChunkInputDialog::getTileX() const {
    return m_tileXSpin->value();
}

int ChunkInputDialog::getTileY() const {
    return m_tileYSpin->value();
}

bool ChunkInputDialog::isQuadsphere() const {
    return m_radioQuadsphere->isChecked();
}
