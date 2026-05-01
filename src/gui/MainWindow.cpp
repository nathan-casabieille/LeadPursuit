#include "gui/MainWindow.h"
#include "math/InterceptionSolver.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QSplitter>
#include <QFont>

namespace lead_pursuit {

static const char* PANEL_STYLE = R"(
QWidget {
    background-color: #0d1020;
    color: #9aadcc;
}
QGroupBox {
    border: 1px solid #1e2840;
    border-radius: 6px;
    margin-top: 22px;
    padding: 10px 6px 8px 6px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 2px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
}
QDoubleSpinBox {
    background-color: #111525;
    border: 1px solid #1e2840;
    border-radius: 4px;
    padding: 3px 6px;
    color: #b8cce8;
    selection-background-color: #2a4878;
    min-height: 26px;
}
QDoubleSpinBox:hover { border-color: #283250; }
QDoubleSpinBox:focus { border-color: #3a5ea0; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background-color: #181e30;
    border: none;
    width: 18px;
}
QLabel {
    color: #7888a8;
    background: transparent;
    font-size: 11px;
}
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #1c3a78, stop:1 #122a58);
    border: 1px solid #243e80;
    border-radius: 6px;
    color: #6aa8f0;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 3px;
    padding: 10px;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #244890, stop:1 #1a3878);
    border-color: #3a58a8;
    color: #88c0ff;
}
QPushButton:pressed { background: #0e2050; border-color: #4870c0; }
)";

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("Lead Pursuit — 2D Kinematic Interception");
    resize(1100, 680);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->setStyleSheet("QSplitter::handle { background: #1a1e30; }");
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
    panel->setStyleSheet(PANEL_STYLE);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 12, 10, 12);
    layout->setSpacing(10);

    // --- Hunter group ---
    auto* hunter_group = new QGroupBox("HUNTER");
    hunter_group->setStyleSheet(
        "QGroupBox { color: #4888d8; border-color: #1e2e50; }"
        "QGroupBox::title { color: #5898e8; }");
    auto* hunter_form = new QFormLayout(hunter_group);
    hunter_form->setSpacing(6);
    hunter_x_ = makeSpinBox(-1e5, 1e5, 0.0);
    hunter_y_ = makeSpinBox(-1e5, 1e5, 0.0);
    hunter_speed_ = makeSpinBox(0.0, 1e5, 15.0);
    hunter_form->addRow("X:", hunter_x_);
    hunter_form->addRow("Y:", hunter_y_);
    hunter_form->addRow("Speed:", hunter_speed_);
    layout->addWidget(hunter_group);

    // --- Target group ---
    auto* target_group = new QGroupBox("TARGET");
    target_group->setStyleSheet(
        "QGroupBox { color: #d86050; border-color: #501a1a; }"
        "QGroupBox::title { color: #e87060; }");
    auto* target_form = new QFormLayout(target_group);
    target_form->setSpacing(6);
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
    auto* solve_btn = new QPushButton("SOLVE");
    solve_btn->setMinimumHeight(42);
    connect(solve_btn, &QPushButton::clicked, this, &MainWindow::onSolve);
    layout->addWidget(solve_btn);

    // --- Results group ---
    auto* results_group = new QGroupBox("RESULTS");
    results_group->setStyleSheet(
        "QGroupBox { color: #40b880; border-color: #1a3828; }"
        "QGroupBox::title { color: #40c880; }");
    auto* results_form = new QFormLayout(results_group);
    results_form->setSpacing(6);

    result_status_    = new QLabel("—");
    result_time_      = new QLabel("—");
    result_heading_   = new QLabel("—");
    result_intercept_ = new QLabel("—");
    result_distance_  = new QLabel("—");

    QFont mono_font("monospace", 10);
    for (auto* lbl : {result_status_, result_time_, result_heading_, result_intercept_, result_distance_}) {
        lbl->setFont(mono_font);
        lbl->setStyleSheet("color: #b0c8e8;");
    }

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
    params.hunter_pos         = {hunter_x_->value(), hunter_y_->value()};
    params.hunter_speed       = hunter_speed_->value();
    params.target_pos         = {target_x_->value(), target_y_->value()};
    params.target_speed       = target_speed_->value();
    params.target_heading_deg = target_heading_->value();

    auto result = InterceptionSolver::solve(params);

    if (result.success) {
        result_status_->setText("<span style='color:#44e8a8;font-weight:bold;'>INTERCEPT FOUND</span>");
        result_status_->setTextFormat(Qt::RichText);
        result_time_->setText(QString::number(result.time, 'f', 4));
        result_heading_->setText(QString::number(result.heading_deg, 'f', 2) + "°");
        result_intercept_->setText(
            QString("(%1, %2)")
                .arg(result.intercept.x, 0, 'f', 2)
                .arg(result.intercept.y, 0, 'f', 2));
        result_distance_->setText(QString::number(result.distance, 'f', 2));
    } else {
        result_status_->setText("<span style='color:#ff6858;font-weight:bold;'>NO SOLUTION</span>");
        result_status_->setTextFormat(Qt::RichText);
        result_time_->setText("—");
        result_heading_->setText("—");
        result_intercept_->setText("—");
        result_distance_->setText("—");
    }

    canvas_->setScenario(params, result.success ? std::optional(result) : std::nullopt);
}

} // namespace lead_pursuit
