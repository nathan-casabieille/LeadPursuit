#pragma once

#include "math/Types.h"

#include <QWidget>
#include <optional>

namespace lead_pursuit {

class InterceptCanvas : public QWidget {
    Q_OBJECT
public:
    explicit InterceptCanvas(QWidget* parent = nullptr);

    void setScenario(const InterceptionParams& params,
                     const std::optional<InterceptionResult>& result);
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawGrid(QPainter& painter, const QRectF& world, const QTransform& xform);
    void drawNorthArrow(QPainter& painter);

    QPointF worldToScreen(double wx, double wy) const;
    QRectF computeWorldBounds() const;

    InterceptionParams params_{};
    std::optional<InterceptionResult> result_;
    bool has_data_{false};

    // Cached transform info
    double scale_{1.0};
    double offset_x_{0.0};
    double offset_y_{0.0};
};

} // namespace lead_pursuit
