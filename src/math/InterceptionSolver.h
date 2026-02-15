#pragma once

#include "Types.h"

namespace lead_pursuit {

class InterceptionSolver {
public:
    /// Analytically solve for the earliest interception time.
    /// Returns a result with success=false if no interception is possible.
    static InterceptionResult solve(const InterceptionParams& params);
};

} // namespace lead_pursuit
