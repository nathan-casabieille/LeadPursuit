#include "gui/InterceptCanvas.h"
#include "math/AngleUtils.h"

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QFontMetrics>
#include <cmath>
#include <algorithm>

namespace lead_pursuit {

namespace {
    const QColor BG_COLOR(7, 9, 16);
    const QColor GRID_COLOR(18, 24, 40);
    const QColor AXIS_COLOR(32, 44, 72);
    const QColor GRID_LABEL_COLOR(48, 65, 98);
    const QColor HUNTER_COLOR(70, 150, 255);
    const QColor TARGET_COLOR(255, 88, 78);
    const QColor INTERCEPT_COLOR(48, 210, 148);
    const QColor NORTH_COLOR(170, 195, 228);
}

InterceptCanvas::InterceptCanvas(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 400);
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
    double sy = (double)height() - (wy - offset_y_) * scale_;
    return {sx, sy};
}

void InterceptCanvas::drawGrid(QPainter& painter, const QRectF& world, const QTransform&) {
    painter.save();

    double range = std::max(world.width(), world.height());
    double raw_step = range / 8.0;
    double mag = std::pow(10.0, std::floor(std::log10(raw_step)));
    double norm = raw_step / mag;
    double step;
    if      (norm < 1.5) step = mag;
    else if (norm < 3.5) step = 2.0 * mag;
    else if (norm < 7.5) step = 5.0 * mag;
    else                 step = 10.0 * mag;

    QPen grid_pen(GRID_COLOR, 1, Qt::DotLine);
    QPen axis_pen(AXIS_COLOR, 1, Qt::SolidLine);
    QFont label_font("monospace", 8);
    painter.setFont(label_font);

    double x_start = std::floor(world.left() / step) * step;
    double x_end   = world.right();
    double y_start = std::floor(world.top() / step) * step;
    double y_end   = world.bottom() + world.height();

    for (double x = x_start; x <= x_end; x += step) {
        QPointF top = worldToScreen(x, y_end);
        QPointF bot = worldToScreen(x, world.top());
        painter.setPen(std::abs(x) < step * 0.01 ? axis_pen : grid_pen);
        painter.drawLine(top, bot);
        QPointF lbl = worldToScreen(x, world.top());
        painter.setPen(GRID_LABEL_COLOR);
        painter.drawText(QPointF(lbl.x() + 2, lbl.y() - 4), QString::number(x, 'f', 0));
    }

    for (double y = y_start; y <= y_end; y += step) {
        QPointF left  = worldToScreen(world.left(), y);
        QPointF right = worldToScreen(x_end, y);
        painter.setPen(std::abs(y) < step * 0.01 ? axis_pen : grid_pen);
        painter.drawLine(left, right);
        QPointF lbl = worldToScreen(world.left(), y);
        painter.setPen(GRID_LABEL_COLOR);
        painter.drawText(QPointF(lbl.x() + 2, lbl.y() - 4), QString::number(y, 'f', 0));
    }

    painter.restore();
}

void InterceptCanvas::drawNorthArrow(QPainter& painter) {
    painter.save();

    int ax  = width() - 38;
    int ay  = 44;
    int len = 22;

    // Glow
    painter.setPen(QPen(QColor(NORTH_COLOR.red(), NORTH_COLOR.green(), NORTH_COLOR.blue(), 50), 5));
    painter.drawLine(ax, ay + len, ax, ay - len);

    // Shaft
    painter.setPen(QPen(NORTH_COLOR, 1.5));
    painter.drawLine(ax, ay + len, ax, ay - len);

    // Arrowhead
    painter.setBrush(NORTH_COLOR);
    painter.setPen(Qt::NoPen);
    QPolygon arrow;
    arrow << QPoint(ax, ay - len)
          << QPoint(ax - 5, ay - len + 10)
          << QPoint(ax + 5, ay - len + 10);
    painter.drawPolygon(arrow);

    QFont f("monospace", 9, QFont::Bold);
    painter.setFont(f);
    painter.setPen(NORTH_COLOR);
    painter.drawText(ax - 4, ay - len - 6, "N");

    painter.restore();
}

void InterceptCanvas::drawFighterJet(QPainter& painter, QPointF pos, double heading_deg, QColor color) {
    painter.save();
    painter.translate(pos);
    // nav heading: 0=North=up on screen, 90=East=right. QPainter::rotate is CW.
    painter.rotate(heading_deg);

    // Fighter jet polygon pointing up (North) at scale ~28px tall
    QPolygonF body;
    body << QPointF( 0,   -14)   // nose tip
         << QPointF( 2.5,  -7)   // nose-right
         << QPointF( 3,    -1)   // fuselage right
         << QPointF(13,     7)   // right wing tip
         << QPointF( 4,     9)   // right wing trailing edge
         << QPointF( 5,    13)   // right stabilizer
         << QPointF( 2,    10)   // right stabilizer inner
         << QPointF( 0,    13)   // tail center
         << QPointF(-2,    10)   // left stabilizer inner
         << QPointF(-5,    13)   // left stabilizer
         << QPointF(-4,     9)   // left wing trailing edge
         << QPointF(-13,    7)   // left wing tip
         << QPointF(-3,    -1)   // fuselage left
         << QPointF(-2.5,  -7);  // nose-left

    // Outer glow pass
    painter.setPen(QPen(QColor(color.red(), color.green(), color.blue(), 45), 7));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolygon(body);

    // Body fill
    painter.setPen(QPen(color, 1.5));
    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 55));
    painter.drawPolygon(body);

    // Cockpit highlight
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(185, 225, 255, 190));
    painter.drawEllipse(QPointF(0, -6), 1.8, 2.6);

    painter.restore();
}

void InterceptCanvas::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);

    painter.fillRect(rect(), BG_COLOR);

    if (!has_data_) {
        QFont f("monospace", 11);
        painter.setFont(f);
        painter.setPen(GRID_LABEL_COLOR);
        painter.drawText(rect(), Qt::AlignCenter, "[ Enter parameters and click SOLVE ]");
        return;
    }

    // World-to-screen transform
    QRectF world = computeWorldBounds();
    double sx = (double)width()  / world.width();
    double sy = (double)height() / world.height();
    scale_    = std::min(sx, sy);
    offset_x_ = world.left() - ((double)width()  / scale_ - world.width())  / 2.0;
    offset_y_ = world.top()  - ((double)height() / scale_ - world.height()) / 2.0;

    QTransform dummy;
    drawGrid(painter, world, dummy);
    drawNorthArrow(painter);

    // Trajectory endpoints
    double draw_t = (result_ && result_->success) ? result_->time * 1.2 : 10.0;
    double tx_end = params_.target_pos.x + nav_vx(params_.target_speed, params_.target_heading_deg) * draw_t;
    double ty_end = params_.target_pos.y + nav_vy(params_.target_speed, params_.target_heading_deg) * draw_t;

    auto glowLine = [&](QPointF a, QPointF b, QColor c, Qt::PenStyle style) {
        painter.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 22), 12));
        painter.drawLine(a, b);
        painter.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 65), 5));
        painter.drawLine(a, b);
        painter.setPen(QPen(c, 1.5, style));
        painter.drawLine(a, b);
    };

    // Target trajectory
    QPointF ts = worldToScreen(params_.target_pos.x, params_.target_pos.y);
    QPointF te = worldToScreen(tx_end, ty_end);
    glowLine(ts, te, TARGET_COLOR, Qt::DashLine);

    // Hunter trajectory (solution only)
    if (result_ && result_->success) {
        QPointF hs = worldToScreen(params_.hunter_pos.x, params_.hunter_pos.y);
        QPointF ip = worldToScreen(result_->intercept.x, result_->intercept.y);
        glowLine(hs, ip, HUNTER_COLOR, Qt::SolidLine);
    }

    // Intercept marker
    if (result_ && result_->success) {
        QPointF ip = worldToScreen(result_->intercept.x, result_->intercept.y);
        const QColor& ic = INTERCEPT_COLOR;

        // Outer ring glow
        painter.setPen(QPen(QColor(ic.red(), ic.green(), ic.blue(), 38), 9));
        painter.setBrush(Qt::NoBrush);
        QPolygonF outer_d;
        outer_d << ip + QPointF(0, -17) << ip + QPointF(17, 0)
                << ip + QPointF(0,  17) << ip + QPointF(-17, 0);
        painter.drawPolygon(outer_d);

        // Inner diamond
        painter.setPen(QPen(ic, 1.5));
        painter.setBrush(QColor(ic.red(), ic.green(), ic.blue(), 70));
        QPolygonF inner_d;
        inner_d << ip + QPointF(0, -10) << ip + QPointF(10, 0)
                << ip + QPointF(0,  10) << ip + QPointF(-10, 0);
        painter.drawPolygon(inner_d);

        // Center dot
        painter.setPen(Qt::NoPen);
        painter.setBrush(ic);
        painter.drawEllipse(ip, 2.5, 2.5);
    }

    // Compute hunter heading: use solution heading or point towards target
    double hunter_heading;
    if (result_ && result_->success) {
        hunter_heading = result_->heading_deg;
    } else {
        double dx = params_.target_pos.x - params_.hunter_pos.x;
        double dy = params_.target_pos.y - params_.hunter_pos.y;
        hunter_heading = math_rad_to_nav_deg(std::atan2(dy, dx));
    }

    // Fighter jets (drawn last so they sit on top of trajectories)
    drawFighterJet(painter,
                   worldToScreen(params_.hunter_pos.x, params_.hunter_pos.y),
                   hunter_heading, HUNTER_COLOR);
    drawFighterJet(painter,
                   worldToScreen(params_.target_pos.x, params_.target_pos.y),
                   params_.target_heading_deg, TARGET_COLOR);

    // Labels with dark pill background for readability
    auto drawLabel = [&](QPointF pos, const QString& text, QColor color) {
        QFont f("monospace", 9, QFont::Bold);
        painter.setFont(f);
        QFontMetrics fm(f);
        int tw = fm.horizontalAdvance(text);
        int th = fm.height();
        QPointF lp = pos + QPointF(16, 4);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 0, 0, 150));
        painter.drawRoundedRect(QRectF(lp.x() - 3, lp.y() - th + 2, tw + 8, th + 4), 3, 3);
        painter.setPen(color);
        painter.drawText(lp, text);
    };

    drawLabel(worldToScreen(params_.hunter_pos.x, params_.hunter_pos.y), "HUNTER", HUNTER_COLOR);
    drawLabel(worldToScreen(params_.target_pos.x, params_.target_pos.y), "TARGET", TARGET_COLOR);
    if (result_ && result_->success) {
        drawLabel(worldToScreen(result_->intercept.x, result_->intercept.y), "INTERCEPT", INTERCEPT_COLOR);
    }
}

} // namespace lead_pursuit
