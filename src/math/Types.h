#pragma once

namespace lead_pursuit {

struct Point2D {
    double x{0.0};
    double y{0.0};
};

struct InterceptionParams {
    Point2D hunter_pos;
    double  hunter_speed{0.0};

    Point2D target_pos;
    double  target_speed{0.0};
    double  target_heading_deg{0.0}; // Navigation convention: 0°=N, 90°=E
};

struct InterceptionResult {
    bool    success{false};
    double  time{0.0};              // Seconds to intercept
    double  heading_deg{0.0};       // Hunter heading (nav convention, degrees)
    Point2D intercept;              // Interception coordinates
    double  distance{0.0};         // Distance traveled by hunter
};

} // namespace lead_pursuit
