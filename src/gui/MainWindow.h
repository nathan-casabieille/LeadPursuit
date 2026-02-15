#pragma once

#include "gui/InterceptCanvas.h"
#include "math/Types.h"

#include <QMainWindow>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>

namespace lead_pursuit {

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onSolve();

private:
    QWidget* buildInputPanel();
    QDoubleSpinBox* makeSpinBox(double min, double max, double val, int decimals = 2);

    // Input widgets
    QDoubleSpinBox* hunter_x_{};
    QDoubleSpinBox* hunter_y_{};
    QDoubleSpinBox* hunter_speed_{};

    QDoubleSpinBox* target_x_{};
    QDoubleSpinBox* target_y_{};
    QDoubleSpinBox* target_speed_{};
    QDoubleSpinBox* target_heading_{};

    // Output labels
    QLabel* result_status_{};
    QLabel* result_time_{};
    QLabel* result_heading_{};
    QLabel* result_intercept_{};
    QLabel* result_distance_{};

    InterceptCanvas* canvas_{};
};

} // namespace lead_pursuit
