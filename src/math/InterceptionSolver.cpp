#include "InterceptionSolver.h"
#include "AngleUtils.h"

#include <cmath>
#include <algorithm>

namespace lead_pursuit {

InterceptionResult InterceptionSolver::solve(const InterceptionParams& p) {
    const double dx = p.target_pos.x - p.hunter_pos.x;
    const double dy = p.target_pos.y - p.hunter_pos.y;
    const double dist_sq = dx * dx + dy * dy;

    // Edge case: already co-located
    if (dist_sq < kEpsilon * kEpsilon) {
        return {true, 0.0, 0.0, p.hunter_pos, 0.0};
    }

    // Edge case: hunter cannot move
    if (p.hunter_speed < kEpsilon) {
        return {}; // impossible
    }

    // Target velocity components (X=East, Y=North)
    const double vTx = nav_vx(p.target_speed, p.target_heading_deg);
    const double vTy = nav_vy(p.target_speed, p.target_heading_deg);

    // Quadratic coefficients: a*t² + b*t + c = 0
    const double a = vTx * vTx + vTy * vTy - p.hunter_speed * p.hunter_speed;
    const double b = 2.0 * (dx * vTx + dy * vTy);
    const double c = dist_sq;

    double t = -1.0;

    if (std::abs(a) < kEpsilon) {
        // Linear case (hunter and target speeds effectively equal along bearing)
        if (std::abs(b) < kEpsilon) {
            return {}; // No solution
        }
        double t_lin = -c / b;
        if (t_lin > kEpsilon) {
            t = t_lin;
        }
    } else {
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return {}; // No real solution
        }

        const double sqrt_disc = std::sqrt(discriminant);
        const double t1 = (-b - sqrt_disc) / (2.0 * a);
        const double t2 = (-b + sqrt_disc) / (2.0 * a);

        // Pick smallest positive root
        double t_min = -1.0;
        if (t1 > kEpsilon) t_min = t1;
        if (t2 > kEpsilon && (t_min < 0.0 || t2 < t_min)) t_min = t2;

        if (t_min < 0.0) {
            return {}; // Both roots non-positive
        }
        t = t_min;
    }

    // Compute intercept point
    const Point2D intercept{
        p.target_pos.x + vTx * t,
        p.target_pos.y + vTy * t
    };

    // Hunter heading toward intercept point
    const double hx = intercept.x - p.hunter_pos.x;
    const double hy = intercept.y - p.hunter_pos.y;
    const double heading_rad = std::atan2(hx, hy); // atan2(east, north) = nav angle
    double heading_deg = rad_to_deg(heading_rad);
    if (heading_deg < 0.0) heading_deg += 360.0;

    const double distance = p.hunter_speed * t;

    return {true, t, heading_deg, intercept, distance};
}

} // namespace lead_pursuit
