#ifndef CHUNK_INPUT_DIALOG_H
#define CHUNK_INPUT_DIALOG_H

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QRadioButton>
#include <QLabel>
#include <cstdint>

class ChunkInputDialog : public QDialog {
    Q_OBJECT
public:
    explicit ChunkInputDialog(uint8_t currentFace, int currentX, int currentZ, bool isQuadsphere, QWidget* parent = nullptr);

    uint8_t getFace() const;
    int getTileX() const;
    int getTileY() const;
    bool isQuadsphere() const;

private slots:
    void onConvertGpsClicked();
    void onModeChanged();

private:
    QRadioButton* m_radioQuadsphere = nullptr;
    QRadioButton* m_radioLegacy = nullptr;

    // Quadsphere inputs
    QComboBox* m_faceCombo = nullptr;
    QSpinBox* m_tileXSpin = nullptr;
    QSpinBox* m_tileYSpin = nullptr;

    // GPS inputs
    QDoubleSpinBox* m_latSpin = nullptr;
    QDoubleSpinBox* m_lonSpin = nullptr;

    // Legacy inputs
    QSpinBox* m_legacyXSpin = nullptr;
    QSpinBox* m_legacyZSpin = nullptr;

    QLabel* m_gpsResultLabel = nullptr;
};

#endif // CHUNK_INPUT_DIALOG_H
