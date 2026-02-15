#pragma once

#include <cmath>
#include <numbers>

namespace lead_pursuit {

inline constexpr double kEpsilon = 1e-9;

// Degrees ↔ Radians
inline double deg_to_rad(double deg) {
    return deg * std::numbers::pi / 180.0;
}

inline double rad_to_deg(double rad) {
    return rad * 180.0 / std::numbers::pi;
}

// Navigation heading (0°=N, CW) → standard math angle (0°=E, CCW), in radians.
// Nav heading θ corresponds to math angle (90° − θ).
inline double nav_to_math_rad(double nav_deg) {
    return deg_to_rad(90.0 - nav_deg);
}

// Math angle (radians) → navigation heading (degrees), normalized to [0, 360).
inline double math_rad_to_nav_deg(double math_rad) {
    double nav = 90.0 - rad_to_deg(math_rad);
    nav = std::fmod(nav, 360.0);
    if (nav < 0.0) nav += 360.0;
    return nav;
}

// Velocity components from navigation heading.
// X = East, Y = North (standard map convention).
inline double nav_vx(double speed, double nav_deg) {
    return speed * std::sin(deg_to_rad(nav_deg));
}

inline double nav_vy(double speed, double nav_deg) {
    return speed * std::cos(deg_to_rad(nav_deg));
}

} // namespace lead_pursuit
