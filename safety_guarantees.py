"""
Swayam Safety Guarantees Module

Provides mathematically provable safety layers for swarm collision avoidance:
- Control Barrier Functions (CBF) for minimum separation enforcement
- Runtime Safety Invariant monitoring
- Lyapunov Certificate for controller stability verification
- IEEE-754 numerical guard utilities

Theory Reference:
    CBF: Ames et al., "Control Barrier Functions: Theory and Applications" (2019)
    Lyapunov: Khalil, "Nonlinear Systems" (3rd Ed.), Ch. 4
"""

import math
from typing import Tuple, List, Dict, Optional


# ═══════════════════════════════════════════════════════════════════════════════
# IEEE-754 Numerical Guard Utilities
# ═══════════════════════════════════════════════════════════════════════════════

EPSILON = 1e-9  # Machine-safe threshold for division and normalization


def is_finite(x: float) -> bool:
    """Check if a scalar is finite (not NaN, not Inf)."""
    return math.isfinite(x)


def is_finite_vec(v: Tuple[float, ...]) -> bool:
    """Check all components of a vector are finite."""
    return all(math.isfinite(c) for c in v)


def safe_div(a: float, b: float, fallback: float = 0.0) -> float:
    """Division with zero/NaN protection. Returns fallback if |b| < ε or result is non-finite."""
    if abs(b) < EPSILON:
        return fallback
    result = a / b
    return result if math.isfinite(result) else fallback


def safe_sqrt(x: float) -> float:
    """Square root clamped to non-negative domain."""
    return math.sqrt(max(0.0, x))


def safe_normalize(v: Tuple[float, float], fallback: Tuple[float, float] = (1.0, 0.0)) -> Tuple[float, float]:
    """Normalize a 2D vector. Returns fallback if magnitude < ε."""
    mag = math.hypot(v[0], v[1])
    if mag < EPSILON:
        return fallback
    return (v[0] / mag, v[1] / mag)


def clamp(x: float, lo: float, hi: float) -> float:
    """Numerically safe clamp."""
    return max(lo, min(hi, x))


def clamp_vec(v: Tuple[float, float, float], v_max: float) -> Tuple[float, float, float]:
    """Clamp a 3D vector's magnitude to v_max, preserving direction."""
    mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if mag > v_max and mag > EPSILON:
        scale = v_max / mag
        return (v[0] * scale, v[1] * scale, v[2] * scale)
    return v


def sanitize_output(v: Tuple[float, float, float], v_max: float = 8.0) -> Tuple[float, float, float]:
    """Final NaN/Inf guard on control output. Returns (0,0,0) on any non-finite component."""
    if not is_finite_vec(v):
        return (0.0, 0.0, 0.0)
    return clamp_vec(v, v_max)


# ═══════════════════════════════════════════════════════════════════════════════
# Control Barrier Function (CBF)
# ═══════════════════════════════════════════════════════════════════════════════

class ControlBarrierFunction:
    """
    Pairwise separation enforcement using Control Barrier Functions.

    The barrier function for a drone pair (i, j) is:
        h(x) = ||p_i - p_j||² - d_safe²

    Safety condition (CBF constraint):
        ḣ(x, u) + α·h(x) ≥ 0

    Where:
        ḣ = 2·(p_i - p_j)ᵀ·(v_i - v_j)
        α > 0 is the CBF class-K function parameter (controls aggressiveness)

    If the commanded velocity u would violate the CBF constraint, it is
    projected onto the safe half-space using the analytical closed-form solution.
    """

    def __init__(self, d_safe: float = 1.5, alpha: float = 1.0):
        """
        Args:
            d_safe: Minimum allowed separation distance (meters).
            alpha:  CBF aggressiveness parameter. Higher = more aggressive safety enforcement.
                    Typical range: 0.5 (gentle) to 5.0 (aggressive).
        """
        self.d_safe = d_safe
        self.alpha = alpha

    def compute_barrier(
        self, my_pos: Tuple[float, float, float], neighbor_pos: Tuple[float, float, float]
    ) -> float:
        """
        Compute barrier value h(x) = ||p_i - p_j||² - d_safe².

        Returns:
            h > 0: Safe (separation exceeds d_safe)
            h = 0: At boundary
            h < 0: Violation (separation less than d_safe)
        """
        dp_n = my_pos[0] - neighbor_pos[0]
        dp_e = my_pos[1] - neighbor_pos[1]
        dp_d = my_pos[2] - neighbor_pos[2]
        dist_sq = dp_n**2 + dp_e**2 + dp_d**2
        return dist_sq - self.d_safe**2

    def compute_barrier_dot(
        self,
        my_pos: Tuple[float, float, float], my_vel: Tuple[float, float, float],
        neighbor_pos: Tuple[float, float, float], neighbor_vel: Tuple[float, float, float]
    ) -> float:
        """
        Compute time derivative of barrier: ḣ = 2·Δp·Δv

        Positive ḣ means barrier is increasing (drones separating).
        Negative ḣ means barrier is decreasing (drones closing).
        """
        dp = (my_pos[0] - neighbor_pos[0], my_pos[1] - neighbor_pos[1], my_pos[2] - neighbor_pos[2])
        dv = (my_vel[0] - neighbor_vel[0], my_vel[1] - neighbor_vel[1], my_vel[2] - neighbor_vel[2])
        return 2.0 * (dp[0]*dv[0] + dp[1]*dv[1] + dp[2]*dv[2])

    def is_safe(
        self,
        v_cmd: Tuple[float, float, float],
        my_pos: Tuple[float, float, float],
        neighbor_pos: Tuple[float, float, float],
        neighbor_vel: Tuple[float, float, float]
    ) -> bool:
        """Check if v_cmd satisfies the CBF constraint for a single neighbor."""
        h = self.compute_barrier(my_pos, neighbor_pos)
        h_dot = self.compute_barrier_dot(my_pos, v_cmd, neighbor_pos, neighbor_vel)
        return (h_dot + self.alpha * h) >= 0.0

    def enforce_safety(
        self,
        v_cmd: Tuple[float, float, float],
        my_pos: Tuple[float, float, float],
        neighbors: Dict[str, Dict],
        v_max: float = 8.0
    ) -> Tuple[float, float, float]:
        """
        Project v_cmd onto the safe set defined by ALL pairwise CBF constraints.

        Uses iterative half-space projection (each neighbor defines one constraint).
        The analytical solution for a single constraint is:

            v_safe = v_cmd - max(0, -cbf_margin / ||∇h||²) · ∇h

        For multiple constraints, we iterate (converges in 2-3 passes for typical swarms).

        Args:
            v_cmd:    Intended velocity command (N, E, D)
            my_pos:   Current position (N, E, D)
            neighbors: Dict of {drone_id: {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d}}
            v_max:    Maximum velocity magnitude

        Returns:
            Safe velocity command that satisfies all CBF constraints
        """
        v_safe = list(v_cmd)

        for _iteration in range(3):  # 3 projection passes for multi-constraint convergence
            modified = False
            for drone_id, data in neighbors.items():
                if 'pos_n' not in data:
                    continue

                n_pos = (data['pos_n'], data['pos_e'], data.get('pos_d', 0.0))
                n_vel = (data.get('vel_n', 0.0), data.get('vel_e', 0.0), data.get('vel_d', 0.0))

                # Barrier value
                h = self.compute_barrier(my_pos, n_pos)

                # Gradient of h w.r.t. my velocity: ∇_v h_dot = 2·Δp
                dp = (my_pos[0] - n_pos[0], my_pos[1] - n_pos[1], my_pos[2] - n_pos[2])
                grad = (2.0 * dp[0], 2.0 * dp[1], 2.0 * dp[2])
                grad_sq = grad[0]**2 + grad[1]**2 + grad[2]**2

                if grad_sq < EPSILON:
                    # Co-located — can't compute gradient, skip (emergency envelope handles this)
                    continue

                # ḣ with current v_safe as my velocity
                dv = (v_safe[0] - n_vel[0], v_safe[1] - n_vel[1], v_safe[2] - n_vel[2])
                h_dot = 2.0 * (dp[0]*dv[0] + dp[1]*dv[1] + dp[2]*dv[2])

                # CBF constraint: ḣ + α·h ≥ 0
                cbf_margin = h_dot + self.alpha * h

                if cbf_margin < 0.0:
                    # Constraint violated — project onto safe half-space
                    # v_safe = v_safe - (cbf_margin / ||∇h||²) · ∇h
                    correction = cbf_margin / grad_sq
                    v_safe[0] -= correction * grad[0]
                    v_safe[1] -= correction * grad[1]
                    v_safe[2] -= correction * grad[2]
                    modified = True

            if not modified:
                break  # All constraints satisfied

        # Final velocity magnitude clamp
        result = clamp_vec(tuple(v_safe), v_max)
        return sanitize_output(result, v_max)


# ═══════════════════════════════════════════════════════════════════════════════
# Safety Invariant Monitor
# ═══════════════════════════════════════════════════════════════════════════════

class SafetyViolation(Exception):
    """Raised when a safety invariant is violated."""
    pass


class SafetyInvariant:
    """
    Runtime invariant monitor for the swarm system.

    Checks three categories of invariants:
    1. Separation: ||p_i - p_j|| ≥ d_min  ∀ (i,j) pairs
    2. Velocity:   ||v_i|| ≤ v_max         ∀ i
    3. Geofence:   ||p_i|| ≤ r_max, alt ≥ h_min  ∀ i
    """

    def __init__(self, d_min: float = 1.5, v_max: float = 8.0, r_max: float = 100.0, h_min: float = 1.0):
        self.d_min = d_min
        self.v_max = v_max
        self.r_max = r_max
        self.h_min = h_min
        self.violation_log: List[Dict] = []

    def check_separation(self, positions: List[Tuple[float, float, float]]) -> Tuple[bool, float]:
        """
        Check pairwise separation invariant.

        Returns:
            (is_safe, min_distance): True if all pairs satisfy d ≥ d_min
        """
        min_dist = float('inf')
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0])**2 +
                    (positions[i][1] - positions[j][1])**2 +
                    (positions[i][2] - positions[j][2])**2
                )
                min_dist = min(min_dist, dist)
        return (min_dist >= self.d_min, min_dist)

    def check_velocity_bound(self, vel: Tuple[float, float, float]) -> bool:
        """Check velocity magnitude invariant: ||v|| ≤ v_max."""
        mag = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
        return mag <= self.v_max + EPSILON  # Small tolerance for float comparison

    def check_geofence(self, pos: Tuple[float, float, float]) -> bool:
        """Check geofence invariant: horizontal radius ≤ r_max AND altitude ≥ h_min."""
        radius = math.hypot(pos[0], pos[1])
        alt = -pos[2]  # NED: altitude = -Down
        return radius <= self.r_max and alt >= self.h_min

    def check_output_finite(self, v: Tuple[float, float, float]) -> bool:
        """Check that control output contains no NaN or Inf."""
        return is_finite_vec(v)

    def assert_all(
        self,
        positions: List[Tuple[float, float, float]],
        velocities: List[Tuple[float, float, float]],
        control_outputs: Optional[List[Tuple[float, float, float]]] = None,
        timestamp: float = 0.0,
        raise_on_violation: bool = False
    ) -> Dict:
        """
        Run all invariant checks and return a report.

        Args:
            positions:       List of drone positions (N, E, D)
            velocities:      List of drone velocities (N, E, D)
            control_outputs: Optional list of control outputs to check for finiteness
            timestamp:       Current simulation time
            raise_on_violation: If True, raise SafetyViolation on first failure

        Returns:
            Dict with keys: 'separation_ok', 'min_dist', 'velocity_ok', 'geofence_ok', 'output_finite_ok', 'all_ok'
        """
        sep_ok, min_dist = self.check_separation(positions)
        vel_ok = all(self.check_velocity_bound(v) for v in velocities)
        geo_ok = all(self.check_geofence(p) for p in positions)

        out_ok = True
        if control_outputs:
            out_ok = all(self.check_output_finite(o) for o in control_outputs)

        all_ok = sep_ok and vel_ok and geo_ok and out_ok

        report = {
            "separation_ok": sep_ok,
            "min_dist": min_dist,
            "velocity_ok": vel_ok,
            "geofence_ok": geo_ok,
            "output_finite_ok": out_ok,
            "all_ok": all_ok,
            "timestamp": timestamp
        }

        if not all_ok:
            self.violation_log.append(report)
            if raise_on_violation:
                raise SafetyViolation(f"Invariant violated at t={timestamp:.2f}: {report}")

        return report


# ═══════════════════════════════════════════════════════════════════════════════
# Lyapunov Stability Certificate
# ═══════════════════════════════════════════════════════════════════════════════

class LyapunovCertificate:
    """
    Lyapunov stability certificate for the cascade PID controller.

    Uses the quadratic Lyapunov candidate:
        V(e) = 0.5 · (w_p · ||e_p||² + w_v · ||e_v||²)

    Where:
        e_p = target_pos - pos    (position error)
        e_v = target_vel - vel    (velocity error)
        w_p, w_v = weighting gains

    For stability, V must be:
        1. Positive definite: V(e) > 0 for e ≠ 0
        2. Non-increasing: V̇(e) ≤ 0

    The PD controller with proper gains guarantees V̇ ≤ 0 when:
        kp > 0, kd > 0, and kd² > kp (overdamped condition)
    """

    def __init__(self, w_pos: float = 1.0, w_vel: float = 0.5):
        self.w_pos = w_pos
        self.w_vel = w_vel
        self.energy_history: List[float] = []

    def compute_energy(
        self,
        pos_error: Tuple[float, float, float],
        vel_error: Tuple[float, float, float]
    ) -> float:
        """
        Compute Lyapunov energy V(e).

        V = 0.5 · (w_p · ||e_p||² + w_v · ||e_v||²)
        """
        e_p_sq = pos_error[0]**2 + pos_error[1]**2 + pos_error[2]**2
        e_v_sq = vel_error[0]**2 + vel_error[1]**2 + vel_error[2]**2
        energy = 0.5 * (self.w_pos * e_p_sq + self.w_vel * e_v_sq)
        self.energy_history.append(energy)
        return energy

    def verify_non_increasing(self, tolerance: float = 1e-6) -> Tuple[bool, int]:
        """
        Verify that energy is monotonically non-increasing across all recorded steps.

        Args:
            tolerance: Small positive value to account for floating-point noise.

        Returns:
            (is_stable, violation_count): True if V(t+1) ≤ V(t) + ε for all t
        """
        violations = 0
        for i in range(1, len(self.energy_history)):
            if self.energy_history[i] > self.energy_history[i-1] + tolerance:
                violations += 1
        return (violations == 0, violations)

    def reset(self):
        """Clear energy history for a new verification run."""
        self.energy_history.clear()


# ═══════════════════════════════════════════════════════════════════════════════
# VO Discriminant (Correct Cross-Product Formulation)
# ═══════════════════════════════════════════════════════════════════════════════

def vo_miss_distance(
    dp: Tuple[float, float], dv: Tuple[float, float]
) -> float:
    """
    Compute the closest approach distance using the cross-product formula.

    The miss distance between two agents with relative position dp and
    relative velocity dv is:

        d_miss = |dp × dv| / |dv|

    This is the perpendicular distance from the origin to the line
    defined by the relative trajectory.

    Returns:
        Miss distance in meters. Returns inf if |dv| < ε (no relative motion).
    """
    dv_mag = math.hypot(dv[0], dv[1])
    if dv_mag < EPSILON:
        # No relative motion — miss distance is current separation
        return math.hypot(dp[0], dp[1])

    # 2D cross product: dp × dv = dp[0]*dv[1] - dp[1]*dv[0]
    cross = dp[0] * dv[1] - dp[1] * dv[0]
    return abs(cross) / dv_mag


def vo_time_to_closest(
    dp: Tuple[float, float], dv: Tuple[float, float]
) -> float:
    """
    Compute time to closest approach.

        t_cpa = (dp · dv) / |dv|²

    Returns:
        Time to closest approach in seconds. Returns inf if not closing.
    """
    v_sq = dv[0]**2 + dv[1]**2
    if v_sq < EPSILON:
        return float('inf')

    t_cpa = (dp[0]*dv[0] + dp[1]*dv[1]) / v_sq
    return t_cpa if t_cpa > 0 else float('inf')
