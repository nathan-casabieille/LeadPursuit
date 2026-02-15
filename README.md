# Lead Pursuit — 2D Kinematic Interception Solver

A C++20 / Qt6 application that analytically solves the classic **constant-velocity interception problem**: given a target moving in a straight line at constant speed, compute the heading a hunter must take (also at constant speed) to intercept it in minimum time.

No iterative methods, no numerical integration — just a clean quadratic equation derived from first principles.

## The Problem

Two entities move on a 2D plane:

| Entity | Known | Unknown |
|--------|-------|---------|
| **Target** | Position $(x_T, y_T)$, speed $V_T$, heading $\theta_T$ | — |
| **Hunter** | Position $(x_H, y_H)$, speed $V_H$ | Heading $\theta_H$ |

Both travel in straight lines at constant speed. The question is: **what heading should the hunter choose so that both arrive at the same point at the same time?**

This is not a pursuit curve (where the hunter continuously turns toward the target). The hunter picks a single heading at $t = 0$ and holds it — a *lead pursuit* or *interception* course.

## Mathematical Derivation

### Step 1 — Target Future Position

Using navigation convention (0° = North, clockwise), the target's velocity decomposes as:

$$v_{Tx} = V_T \sin(\theta_T), \qquad v_{Ty} = V_T \cos(\theta_T)$$

where $x$ = East and $y$ = North. At time $t$, the target is at:

$$T(t) = (x_T + v_{Tx} \cdot t, \quad y_T + v_{Ty} \cdot t)$$

### Step 2 — The Interception Constraint

The hunter must reach $T(t)$ at the same time $t$. Since the hunter travels at speed $V_H$, the distance it covers is $V_H \cdot t$. This distance must equal the Euclidean distance from the hunter's start to the interception point:

$$\| T(t) - H_0 \|^2 = (V_H \cdot t)^2$$

Expanding with $\Delta x = x_T - x_H$ and $\Delta y = y_T - y_H$:

$$(\Delta x + v_{Tx} \cdot t)^2 + (\Delta y + v_{Ty} \cdot t)^2 = V_H^2 \cdot t^2$$

### Step 3 — Expand and Collect

$$\Delta x^2 + 2 \Delta x \, v_{Tx} \, t + v_{Tx}^2 \, t^2 + \Delta y^2 + 2 \Delta y \, v_{Ty} \, t + v_{Ty}^2 \, t^2 = V_H^2 \, t^2$$

Rearranging into standard quadratic form $at^2 + bt + c = 0$:

$$\boxed{a = v_{Tx}^2 + v_{Ty}^2 - V_H^2 = V_T^2 - V_H^2}$$

$$\boxed{b = 2(\Delta x \cdot v_{Tx} + \Delta y \cdot v_{Ty})}$$

$$\boxed{c = \Delta x^2 + \Delta y^2 = d_0^2}$$

where $d_0$ is the initial distance between hunter and target.

> Note that $a$ simplifies to $V_T^2 - V_H^2$ because $v_{Tx}^2 + v_{Ty}^2 = V_T^2 \sin^2\theta + V_T^2 \cos^2\theta = V_T^2$.

### Step 4 — Solve the Quadratic

$$t = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}$$

This yields zero, one, or two real roots. The **physical constraints** are:

- $t > 0$ (interception must be in the future)
- If two positive roots exist, the **smallest** is the optimal (earliest) interception

### Step 5 — Recover the Hunter Heading

Once $t$ is known, the interception point is:

$$P^* = T(t) = (x_T + v_{Tx} \cdot t, \quad y_T + v_{Ty} \cdot t)$$

The hunter's required heading (in navigation convention) is:

$$\theta_H = \text{atan2}(P^{*}_{x} - x_H, \quad P^{*}_{y} - y_H)$$

Note the argument order: `atan2(east, north)` directly produces a navigation angle (0° = North, CW positive).

## Edge Cases

| Condition | Interpretation | Result |
|-----------|---------------|--------|
| $d_0 \approx 0$ | Already co-located | $t = 0$, immediate intercept |
| $a = 0$ | $V_H = V_T$ (equal speeds) | Degenerates to linear: $t = -c/b$ |
| $\Delta < 0$ | No real roots | Interception impossible |
| Both $t_1, t_2 \le 0$ | Solutions in the past | Interception impossible |
| $V_H = 0$ | Hunter cannot move | Impossible (unless co-located) |
| $V_T > V_H$ and target fleeing | Discriminant goes negative | Correctly returns no solution |

## Worked Example

**Setup:**
- Hunter at $(0, 0)$, speed $V_H = 15$
- Target at $(100, 100)$, speed $V_T = 8$, heading $180°$ (due South)

**Computation:**

$$\Delta x = 100, \quad \Delta y = 100$$

$$v_{Tx} = 8 \sin(180°) = 0, \quad v_{Ty} = 8 \cos(180°) = -8$$

$$a = 0 + 64 - 225 = -161$$

$$b = 2(100 \cdot 0 + 100 \cdot (-8)) = -1600$$

$$c = 100^2 + 100^2 = 20000$$

$$\Delta = (-1600)^2 - 4(-161)(20000) = 2{,}560{,}000 + 12{,}880{,}000 = 15{,}440{,}000$$

$$\sqrt{\Delta} \approx 3929.94$$

$$t_1 = \frac{1600 - 3929.94}{-322} \approx 7.236, \quad t_2 = \frac{1600 + 3929.94}{-322} \approx -17.174$$

Only $t_1 > 0$, so $t^* \approx 7.236$ s.

**Intercept point:**

$$P^* = (100 + 0 \times 7.236, \quad 100 + (-8) \times 7.236) = (100.00, \quad 42.11)$$

**Hunter heading:**

$$\theta_H = \text{atan2}(100, 42.11) \approx 67.15°$$

**Verification:** Hunter distance $= 15 \times 7.236 = 108.54$. Check: $\sqrt{100^2 + 42.11^2} = 108.51$ (matches within floating-point tolerance).

## Project Structure

```
LeadPursuit/
├── CMakeLists.txt
├── README.md
└── src/
    ├── main.cpp
    ├── math/
    │   ├── Types.h               # Point2D, InterceptionParams, InterceptionResult
    │   ├── AngleUtils.h          # Degree/radian and navigation/math conversions
    │   └── InterceptionSolver.h/cpp   # Pure analytical solver (no Qt dependency)
    └── gui/
        ├── MainWindow.h/cpp      # Input panel, results display, layout
        └── InterceptCanvas.h/cpp # Custom QWidget: grid, paths, markers
```

The math engine has **zero dependency on Qt** and can be used standalone.

## Building

Requires C++20 and Qt6.

```bash
cmake -B build
cmake --build build
./build/LeadPursuit
```

## License

MIT
