#include "gui/InterceptCanvas.h"
#include "math/AngleUtils.h"

#include <QPainter>
#include <QPen>
#include <QFont>
#include <cmath>
#include <algorithm>

namespace lead_pursuit {

InterceptCanvas::InterceptCanvas(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 400);
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
}

void InterceptCanvas::setScenario(const InterceptionParams& params,
                                   const std::optional<InterceptionResult>& result) {
    params_ = params;
    result_ = result;
    has_data_ = true;
    update();
}

void InterceptCanvas::clear() {
    has_data_ = false;
    result_.reset();
    update();
}

QRectF InterceptCanvas::computeWorldBounds() const {
    double min_x = std::min(params_.hunter_pos.x, params_.target_pos.x);
    double max_x = std::max(params_.hunter_pos.x, params_.target_pos.x);
    double min_y = std::min(params_.hunter_pos.y, params_.target_pos.y);
    double max_y = std::max(params_.hunter_pos.y, params_.target_pos.y);

    if (result_ && result_->success) {
        min_x = std::min(min_x, result_->intercept.x);
        max_x = std::max(max_x, result_->intercept.x);
        min_y = std::min(min_y, result_->intercept.y);
        max_y = std::max(max_y, result_->intercept.y);
    }

    // Also extend along target path even if no solution
    double ext_t = result_ && result_->success ? result_->time : 10.0;
    double tx_end = params_.target_pos.x + nav_vx(params_.target_speed, params_.target_heading_deg) * ext_t;
    double ty_end = params_.target_pos.y + nav_vy(params_.target_speed, params_.target_heading_deg) * ext_t;
    min_x = std::min(min_x, tx_end);
    max_x = std::max(max_x, tx_end);
    min_y = std::min(min_y, ty_end);
    max_y = std::max(max_y, ty_end);

    double pad_x = std::max((max_x - min_x) * 0.15, 10.0);
    double pad_y = std::max((max_y - min_y) * 0.15, 10.0);

    return QRectF(min_x - pad_x, min_y - pad_y,
                  (max_x - min_x) + 2.0 * pad_x,
                  (max_y - min_y) + 2.0 * pad_y);
}

QPointF InterceptCanvas::worldToScreen(double wx, double wy) const {
    double sx = (wx - offset_x_) * scale_;
    double sy = (double)height() - (wy - offset_y_) * scale_; // flip Y for screen
    return {sx, sy};
}

void InterceptCanvas::drawGrid(QPainter& painter, const QRectF& world, const QTransform&) {
    painter.save();

    // Determine grid spacing
    double range = std::max(world.width(), world.height());
    double raw_step = range / 8.0;
    double mag = std::pow(10.0, std::floor(std::log10(raw_step)));
    double norm = raw_step / mag;
    double step;
    if (norm < 1.5) step = mag;
    else if (norm < 3.5) step = 2.0 * mag;
    else if (norm < 7.5) step = 5.0 * mag;
    else step = 10.0 * mag;

    QPen grid_pen(QColor(220, 220, 220), 1, Qt::DotLine);
    QPen axis_pen(QColor(180, 180, 180), 1, Qt::SolidLine);
    QFont label_font("monospace", 8);
    painter.setFont(label_font);

    double x_start = std::floor(world.left() / step) * step;
    double x_end = world.right();
    double y_start = std::floor(world.top() / step) * step;
    double y_end = world.bottom() + world.height();

    // Vertical grid lines
    for (double x = x_start; x <= x_end; x += step) {
        QPointF top = worldToScreen(x, y_end);
        QPointF bot = worldToScreen(x, world.top());
        painter.setPen(std::abs(x) < step * 0.01 ? axis_pen : grid_pen);
        painter.drawLine(top, bot);
        QPointF lbl = worldToScreen(x, world.top());
        painter.setPen(Qt::darkGray);
        painter.drawText(QPointF(lbl.x() + 2, lbl.y() - 4), QString::number(x, 'f', 0));
    }

    // Horizontal grid lines
    for (double y = y_start; y <= y_end; y += step) {
        QPointF left = worldToScreen(world.left(), y);
        QPointF right = worldToScreen(x_end, y);
        painter.setPen(std::abs(y) < step * 0.01 ? axis_pen : grid_pen);
        painter.drawLine(left, right);
        QPointF lbl = worldToScreen(world.left(), y);
        painter.setPen(Qt::darkGray);
        painter.drawText(QPointF(lbl.x() + 2, lbl.y() - 4), QString::number(y, 'f', 0));
    }

    painter.restore();
}

void InterceptCanvas::drawNorthArrow(QPainter& painter) {
    painter.save();
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(Qt::black);

    int ax = width() - 35;
    int ay = 40;
    int len = 25;

    painter.drawLine(ax, ay + len, ax, ay - len);
    // Arrowhead
    QPolygon arrow;
    arrow << QPoint(ax, ay - len)
          << QPoint(ax - 5, ay - len + 10)
          << QPoint(ax + 5, ay - len + 10);
    painter.drawPolygon(arrow);

    QFont f("sans-serif", 10, QFont::Bold);
    painter.setFont(f);
    painter.drawText(ax - 4, ay - len - 5, "N");

    painter.restore();
}

void InterceptCanvas::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (!has_data_) {
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "Enter parameters and click Solve");
        return;
    }

    // Compute world-to-screen transform
    QRectF world = computeWorldBounds();
    double sx = (double)width() / world.width();
    double sy = (double)height() / world.height();
    scale_ = std::min(sx, sy);
    offset_x_ = world.left() - ((double)width() / scale_ - world.width()) / 2.0;
    offset_y_ = world.top() - ((double)height() / scale_ - world.height()) / 2.0;

    QTransform dummy;
    drawGrid(painter, world, dummy);
    drawNorthArrow(painter);

    // --- Target path ---
    double draw_t = (result_ && result_->success) ? result_->time * 1.2 : 10.0;
    double tx_end = params_.target_pos.x + nav_vx(params_.target_speed, params_.target_heading_deg) * draw_t;
    double ty_end = params_.target_pos.y + nav_vy(params_.target_speed, params_.target_heading_deg) * draw_t;

    QPen target_path_pen(QColor(220, 60, 60), 2, Qt::DashLine);
    painter.setPen(target_path_pen);
    painter.drawLine(worldToScreen(params_.target_pos.x, params_.target_pos.y),
                     worldToScreen(tx_end, ty_end));

    // --- Hunter path (only if solution exists) ---
    if (result_ && result_->success) {
        QPen hunter_path_pen(QColor(40, 100, 220), 2, Qt::SolidLine);
        painter.setPen(hunter_path_pen);
        painter.drawLine(worldToScreen(params_.hunter_pos.x, params_.hunter_pos.y),
                         worldToScreen(result_->intercept.x, result_->intercept.y));
    }

    // --- Target start (red circle) ---
    painter.setPen(QPen(QColor(220, 60, 60), 2));
    painter.setBrush(QColor(220, 60, 60, 120));
    QPointF ts = worldToScreen(params_.target_pos.x, params_.target_pos.y);
    painter.drawEllipse(ts, 7, 7);
    painter.setPen(Qt::darkRed);
    painter.drawText(ts + QPointF(10, -5), "Target");

    // --- Hunter start (blue circle) ---
    painter.setPen(QPen(QColor(40, 100, 220), 2));
    painter.setBrush(QColor(40, 100, 220, 120));
    QPointF hs = worldToScreen(params_.hunter_pos.x, params_.hunter_pos.y);
    painter.drawEllipse(hs, 7, 7);
    painter.setPen(Qt::darkBlue);
    painter.drawText(hs + QPointF(10, -5), "Hunter");

    // --- Intercept point (green diamond) ---
    if (result_ && result_->success) {
        QPointF ip = worldToScreen(result_->intercept.x, result_->intercept.y);
        painter.setPen(QPen(QColor(30, 180, 30), 2));
        painter.setBrush(QColor(30, 180, 30, 160));
        QPolygonF diamond;
        diamond << ip + QPointF(0, -9) << ip + QPointF(9, 0)
                << ip + QPointF(0, 9) << ip + QPointF(-9, 0);
        painter.drawPolygon(diamond);
        painter.setPen(Qt::darkGreen);
        painter.drawText(ip + QPointF(12, -5), "Intercept");
    }
}

} // namespace lead_pursuit
