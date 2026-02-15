#include "gui/MainWindow.h"
#include "math/InterceptionSolver.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QSplitter>

namespace lead_pursuit {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("Lead Pursuit — 2D Kinematic Interception");
    resize(1100, 650);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(buildInputPanel());

    canvas_ = new InterceptCanvas();
    splitter->addWidget(canvas_);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    setCentralWidget(splitter);
}

QDoubleSpinBox* MainWindow::makeSpinBox(double min, double max, double val, int decimals) {
    auto* sb = new QDoubleSpinBox();
    sb->setRange(min, max);
    sb->setValue(val);
    sb->setDecimals(decimals);
    sb->setSingleStep(1.0);
    return sb;
}

QWidget* MainWindow::buildInputPanel() {
    auto* panel = new QWidget();
    auto* layout = new QVBoxLayout(panel);

    // --- Hunter group ---
    auto* hunter_group = new QGroupBox("Hunter");
    auto* hunter_form = new QFormLayout(hunter_group);
    hunter_x_ = makeSpinBox(-1e5, 1e5, 0.0);
    hunter_y_ = makeSpinBox(-1e5, 1e5, 0.0);
    hunter_speed_ = makeSpinBox(0.0, 1e5, 15.0);
    hunter_form->addRow("X:", hunter_x_);
    hunter_form->addRow("Y:", hunter_y_);
    hunter_form->addRow("Speed:", hunter_speed_);
    layout->addWidget(hunter_group);

    // --- Target group ---
    auto* target_group = new QGroupBox("Target");
    auto* target_form = new QFormLayout(target_group);
    target_x_ = makeSpinBox(-1e5, 1e5, 100.0);
    target_y_ = makeSpinBox(-1e5, 1e5, 0.0);
    target_speed_ = makeSpinBox(0.0, 1e5, 8.0);
    target_heading_ = makeSpinBox(0.0, 360.0, 45.0, 1);
    target_heading_->setWrapping(true);
    target_form->addRow("X:", target_x_);
    target_form->addRow("Y:", target_y_);
    target_form->addRow("Speed:", target_speed_);
    target_form->addRow("Heading (°):", target_heading_);
    layout->addWidget(target_group);

    // --- Solve button ---
    auto* solve_btn = new QPushButton("Solve");
    solve_btn->setMinimumHeight(36);
    solve_btn->setStyleSheet("font-weight: bold; font-size: 14px;");
    connect(solve_btn, &QPushButton::clicked, this, &MainWindow::onSolve);
    layout->addWidget(solve_btn);

    // --- Results group ---
    auto* results_group = new QGroupBox("Results");
    auto* results_form = new QFormLayout(results_group);

    result_status_    = new QLabel("—");
    result_time_      = new QLabel("—");
    result_heading_   = new QLabel("—");
    result_intercept_ = new QLabel("—");
    result_distance_  = new QLabel("—");

    results_form->addRow("Status:", result_status_);
    results_form->addRow("Time (s):", result_time_);
    results_form->addRow("Heading (°):", result_heading_);
    results_form->addRow("Intercept:", result_intercept_);
    results_form->addRow("Distance:", result_distance_);
    layout->addWidget(results_group);

    layout->addStretch();
    panel->setMaximumWidth(300);

    return panel;
}

void MainWindow::onSolve() {
    InterceptionParams params;
    params.hunter_pos   = {hunter_x_->value(), hunter_y_->value()};
    params.hunter_speed = hunter_speed_->value();
    params.target_pos   = {target_x_->value(), target_y_->value()};
    params.target_speed = target_speed_->value();
    params.target_heading_deg = target_heading_->value();

    auto result = InterceptionSolver::solve(params);

    if (result.success) {
        result_status_->setText("<span style='color:green;font-weight:bold;'>INTERCEPT FOUND</span>");
        result_status_->setTextFormat(Qt::RichText);
        result_time_->setText(QString::number(result.time, 'f', 4));
        result_heading_->setText(QString::number(result.heading_deg, 'f', 2) + "°");
        result_intercept_->setText(
            QString("(%1, %2)")
                .arg(result.intercept.x, 0, 'f', 2)
                .arg(result.intercept.y, 0, 'f', 2));
        result_distance_->setText(QString::number(result.distance, 'f', 2));
    } else {
        result_status_->setText("<span style='color:red;font-weight:bold;'>NO SOLUTION</span>");
        result_status_->setTextFormat(Qt::RichText);
        result_time_->setText("—");
        result_heading_->setText("—");
        result_intercept_->setText("—");
        result_distance_->setText("—");
    }

    canvas_->setScenario(params, result.success ? std::optional(result) : std::nullopt);
}

} // namespace lead_pursuit
