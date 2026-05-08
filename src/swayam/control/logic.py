# swarm logic. kinda chaotic but it works. 
# handles all the flocking and avoidance stuff so drones don't play bumper cars
# NED coordinates because up is down and down is up. don't question it.
# - ARYA-mgc (sorry in advance)

import math
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass
from src.swayam.control.safety import ControlBarrierFunction, SafetyInvariant, LyapunovCertificate, is_finite_vec, safe_div, safe_normalize, sanitize_output, vo_miss_distance, vo_time_to_closest, EPSILON

@dataclass
class PIDConfig:
    kp: float = 1.0
    kp_vel: float = 1.0
    ki: float = 0.1
    kd: float = 0.2
    kp_z: float = 0.8
    kp_vel_z: float = 1.2
    ki_z: float = 0.05
    kd_z: float = 0.4
    # degraded mode gains (lower authority when sensor health is bad)
    kp_deg: float = 0.5
    kv_deg: float = 0.5
    ki_deg: float = 0.0
    kd_deg: float = 0.4
    j_max: float = 20.0
    lpf_cutoff_hz: float = 8.0  # was 5.0, bumped to 8 to reduce lag on fast maneuvers

class AutonomousSwarmLogic:

    def __init__(self, my_id: str, pid_config: Optional[PIDConfig]=None):
        self.my_id = my_id
        self.pid_config = pid_config or PIDConfig()
        self.swarm_state: Dict[str, Dict[str, float]] = {}
        self.current_leader: Optional[str] = None
        self.last_election_time: float = 0.0
        self.election_cooldown: float = 4.0
        self.last_emergency_time: float = 0.0
        self.emergency_mode_active: bool = False
        self.telemetry_rate_hz: float = 20.0
        self.swarm_origin: Optional[Tuple[float, float, float, float]] = None
        self.integral_n = 0.0
        self.integral_e = 0.0
        self.integral_d = 0.0
        self.evasion_end_time = 0.0
        self.active_evasion_v = (0.0, 0.0, 0.0)
        self.isolation_start_time: Optional[float] = None
        self.metrics = {'violation_count': 0, 'geofence_breaches': 0, 'min_sep_violations': 0, 'time_in_degraded_mode': 0.0, 'cbf_interventions': 0}
        self.i_term_v = [0.0, 0.0, 0.0]
        self.last_err_v = [0.0, 0.0, 0.0]
        self.last_d_term = [0.0, 0.0, 0.0]
        self.last_a_cmd = [0.0, 0.0, 0.0]
        self.rtl_triggered = False
        self.cbf = ControlBarrierFunction(d_safe=1.5, alpha=2.0)

    def update_swarm_state(self, drone_id: str, data: Dict[str, Any], dt: float=0.0, current_time: float=0.0):
        if drone_id != self.my_id:
            t_remote = data.get('timestamp', current_time)
            age = current_time - (t_remote + dt)
            data['local_arrival_time'] = current_time
            if dt > 0.0 and 'pos_n' in data and ('vel_n' in data):
                dt = min(dt, 0.1)
                v_cap = 15.0

                def clip(val, limit):
                    return max(-limit, min(limit, val))
                v_pred_n = clip(data.get('vel_n', 0.0), v_cap)
                v_pred_e = clip(data.get('vel_e', 0.0), v_cap)
                v_pred_d = clip(data.get('vel_d', 0.0), v_cap)
                pred_pos_n = data['pos_n'] + v_pred_n * dt
                pred_pos_e = data['pos_e'] + v_pred_e * dt
                pred_pos_d = data['pos_d'] + v_pred_d * dt
                if drone_id in self.swarm_state and 'pos_n' in self.swarm_state[drone_id]:
                    alpha = 0.7
                    data['pos_n'] = alpha * pred_pos_n + (1 - alpha) * self.swarm_state[drone_id]['pos_n']
                    data['pos_e'] = alpha * pred_pos_e + (1 - alpha) * self.swarm_state[drone_id]['pos_e']
                    data['pos_d'] = alpha * pred_pos_d + (1 - alpha) * self.swarm_state[drone_id]['pos_d']
                else:
                    data['pos_n'] = pred_pos_n
                    data['pos_e'] = pred_pos_e
                    data['pos_d'] = pred_pos_d
            self.swarm_state[drone_id] = data
        # ignore updates from self, shouldn't happen but just in case

    def prune_stale_telemetry(self, current_time: float, max_age: float=2.0):
        # TODO: memory leak maybe?? fix this later
        stale_ids = []
        for did, data in self.swarm_state.items():
            arrival = data.get('local_arrival_time', current_time)
            if current_time - arrival > max_age:
                stale_ids.append(did)
        for did in stale_ids:
            del self.swarm_state[did]

    def elect_leader(self, current_time: float, my_health_score: float=1.0) -> Optional[str]:
        candidates = [(self.my_id, my_health_score)]
        for drone_id, data in self.swarm_state.items():
            candidates.append((drone_id, data.get('health_score', 1.0)))
        candidates.sort(key=lambda x: (-x[1], x[0]))
        best_candidate, best_health = candidates[0]
        if not self.current_leader:
            self.current_leader = best_candidate
            self.last_election_time = current_time
            return self.current_leader
        if self.current_leader == best_candidate:
            return self.current_leader
        current_leader_health = next((h for d, h in candidates if d == self.current_leader), 0.0)
        in_cooldown = current_time - self.last_election_time < self.election_cooldown
        clearly_healthier = best_health > current_leader_health + 0.2
        if not in_cooldown and clearly_healthier or current_leader_health < 0.1:
            self.current_leader = best_candidate
            self.last_election_time = current_time
        return self.current_leader

    def update_backpressure(self, incoming_rate: float):
        # exponential moving average to smooth rate changes
        # if we don't do this the rate bounces around like crazy
        alpha = 0.1
        target_rate = alpha * incoming_rate + (1.0 - alpha) * self.telemetry_rate_hz
        delta = target_rate - self.telemetry_rate_hz
        delta = max(-5.0, min(5.0, delta))
        new_rate = self.telemetry_rate_hz + delta
        self.telemetry_rate_hz = max(10.0, min(100.0, new_rate))

    # Swarm Formation:
    #      [Leader]
    #      /      \
    #  [D1]        [D2]
    def calculate_formation_target(self, leader_id: str, offset_n: float=2.0, offset_e: float=2.0, offset_d: float=0.0) -> Optional[Tuple[float, float, float]]:
        if leader_id not in self.swarm_state:
            return None
        leader_data = self.swarm_state[leader_id]
        if 'pos_n' not in leader_data:
            return self._calculate_legacy_follow(leader_data, offset_n, offset_e)
        target_n = leader_data['pos_n'] + offset_n
        target_e = leader_data['pos_e'] + offset_e
        target_d = leader_data['pos_d'] + offset_d
        return (target_n, target_e, target_d)

    def _calculate_legacy_follow(self, leader_data, offset_n, offset_e):
        # GAMMA hasn't been flashed with the new firmware yet so we still need this
        target_lat = leader_data.get('lat', 0) + offset_n / 111111.0
        target_lon = leader_data.get('lon', 0) + offset_e / (111111.0 * math.cos(math.radians(leader_data.get('lat', 0))))
        return (target_lat, target_lon, leader_data.get('alt', 0))

    def check_collision_course(self, my_pos: Tuple[float, float, float], my_vel: Tuple[float, float, float], safety_radius: float=5.0, time_horizon: float=3.0) -> list:
        risks = []
        for drone_id, data in self.swarm_state.items():
            if 'pos_n' not in data:
                continue
            dp = [data['pos_n'] - my_pos[0], data['pos_e'] - my_pos[1]]
            # dist = math.sqrt(dp[0]**2 + dp[1]**2) # old way too slow
            dist = math.hypot(dp[0], dp[1])
            dv = [my_vel[0] - data.get('vel_n', 0.0), my_vel[1] - data.get('vel_e', 0.0)]
            v_dot_p = dv[0] * dp[0] + dv[1] * dp[1]
            v_sq = dv[0] ** 2 + dv[1] ** 2
            if dist < 2.0:
                risks.append((drone_id, 'IMMEDIATE', 0.0, dv))
                continue
            if dist < safety_radius:
                if v_dot_p > 0:
                    risks.append((drone_id, 'PREDICTED', max(0.1, dist / (math.sqrt(v_sq) + 0.1)), dv))
                continue
            if v_dot_p > 0:
                v_sq_safe = v_sq + EPSILON
                t_cpa = vo_time_to_closest(dp, dv)
                d_miss = vo_miss_distance(dp, dv)
                if t_cpa < time_horizon and d_miss < safety_radius:
                    risks.append((drone_id, 'PREDICTED', t_cpa, dv))
        return risks

    def compute_evasion_velocity(self, my_pos: Tuple[float, float, float], risks: list, max_v: float) -> Tuple[float, float, float]:
        if not risks:
            return (0.0, 0.0, 0.0)
        severity_order = {'IMMEDIATE': 0, 'PREDICTED': 1}
        sorted_risks = sorted(risks, key=lambda r: (severity_order.get(r[1], 2), r[2]))
        ev_n, ev_e, ev_d = (0.0, 0.0, 0.0)
        has_immediate = False
        for risk_id, severity, tc, dv in sorted_risks[:3]:
            if risk_id not in self.swarm_state or 'pos_n' not in self.swarm_state[risk_id]:
                continue
            if severity == 'IMMEDIATE':
                has_immediate = True
            other_data = self.swarm_state[risk_id]
            dp_n = other_data['pos_n'] - my_pos[0]
            dp_e = other_data['pos_e'] - my_pos[1]
            dist = math.hypot(dp_n, dp_e)
            if dist < 0.1:
                ev_n += 5.0
                continue
            weight = 1.0 / (tc + 0.1)
            if severity == 'IMMEDIATE':
                weight *= 3.0
            repulse_n = -dp_n / dist
            repulse_e = -dp_e / dist
            dv_mag = math.hypot(dv[0], dv[1])
            if dv_mag > 0.1:
                slide_n = -dv[1] / dv_mag
                slide_e = dv[0] / dv_mag
            else:
                slide_n = dp_e / dist
                slide_e = -dp_n / dist
            ev_n += (repulse_n + slide_n * 1.5) * weight
            ev_e += (repulse_e + slide_e * 1.5) * weight
        id_hash = sum((ord(c) for c in self.my_id)) * 137 % 360
        angle = math.radians(id_hash)
        ev_n += math.cos(angle) * 0.5
        ev_e += math.sin(angle) * 0.5
        mag = math.hypot(ev_n, ev_e)
        if mag > 0:
            v_avoid_mag = max_v if has_immediate else min(max_v, max(3.0, mag * 5.0))
            ev_n = ev_n / mag * v_avoid_mag
            ev_e = ev_e / mag * v_avoid_mag
        if has_immediate:
            return (ev_n, ev_e, -2.0)
        else:
            return (ev_n, ev_e, 0.0)

    # caluculate velocity 
    def calculate_flocking_velocity(self, my_pos: Tuple[float, float, float], my_vel: Tuple[float, float, float], separation_weight: float=1.5, alignment_weight: float=1.0, cohesion_weight: float=1.0, desired_separation: float=5.0) -> Tuple[float, float, float]:
        if not self.swarm_state:
            return (0.0, 0.0, 0.0)
        sep_n, sep_e, sep_d = (0.0, 0.0, 0.0)
        ali_n, ali_e, ali_d = (0.0, 0.0, 0.0)
        coh_n, coh_e, coh_d = (0.0, 0.0, 0.0)
        count = 0
        for drone_id, data in self.swarm_state.items():
            if 'pos_n' not in data:
                continue
            dp_n = my_pos[0] - data['pos_n']
            dp_e = my_pos[1] - data['pos_e']
            dp_d = my_pos[2] - data['pos_d']
            dist = math.hypot(dp_n, dp_e)
            if dist > 0.01:
                if dist < desired_separation:
                    scale = (desired_separation - dist) / dist
                    sep_n += dp_n * scale
                    sep_e += dp_e * scale
                    sep_d += dp_d * scale
                ali_n += data.get('vel_n', 0.0)
                ali_e += data.get('vel_e', 0.0)
                ali_d += data.get('vel_d', 0.0)
                coh_n += data['pos_n']
                coh_e += data['pos_e']
                coh_d += data['pos_d']
                count += 1
        if count > 0:
            ali_n = ali_n / count - my_vel[0]
            ali_e = ali_e / count - my_vel[1]
            ali_d = ali_d / count - my_vel[2]
            coh_n = coh_n / count - my_pos[0]
            coh_e = coh_e / count - my_pos[1]
            coh_d = coh_d / count - my_pos[2]

            def normalize(v, k):
                mag = math.hypot(v[0], v[1])
                if mag > k:
                    return (v[0] / mag * k, v[1] / mag * k, v[2])
                return v
            sep = normalize((sep_n, sep_e, sep_d), 3.0)
            ali = normalize((ali_n, ali_e, ali_d), 2.0)
            coh = normalize((coh_n, coh_e, coh_d), 2.0)
            avg_dist = sum((math.hypot(my_pos[0] - d['pos_n'], my_pos[1] - d['pos_e']) for d in self.swarm_state.values() if 'pos_n' in d)) / max(1, count)
            form_error = abs(avg_dist - desired_separation)
            dyn_coh_weight = cohesion_weight * (1.0 + min(form_error, 10.0) * 0.2)
            v_n = sep[0] * separation_weight + ali[0] * alignment_weight + coh[0] * dyn_coh_weight
            v_e = sep[1] * separation_weight + ali[1] * alignment_weight + coh[1] * dyn_coh_weight
            v_d = sep[2] * separation_weight + ali[2] * alignment_weight + coh[2] * dyn_coh_weight
            return (v_n, v_e, v_d)
        return (0.0, 0.0, 0.0)

    def check_emergency_envelope(self, my_pos: Tuple[float, float, float], my_vel: Tuple[float, float, float], critical_radius: float=3.0, current_time: float=0.0) -> Optional[Tuple[float, float, float]]:
        agg_n, agg_e, agg_d = (0.0, 0.0, 0.0)
        in_emergency = False
        for drone_id, data in self.swarm_state.items():
            if 'pos_n' not in data:
                continue
            dp_n = my_pos[0] - data['pos_n']
            dp_e = my_pos[1] - data['pos_e']
            dist = math.hypot(dp_n, dp_e)
            if dist < 0.01:
                agg_n += 6.0
                agg_d += -3.0 if self.my_id > drone_id else 3.0
                in_emergency = True
                continue
            if dist < 2.0:
                in_emergency = True
                scale = 6.0 / dist
                agg_n += dp_n * scale
                agg_e += dp_e * scale
                agg_d += -3.0 if self.my_id > drone_id else 3.0
            elif dist < critical_radius:
                dv_n = my_vel[0] - data.get('vel_n', 0.0)
                dv_e = my_vel[1] - data.get('vel_e', 0.0)
                # why does this only work when i add 0.001
                closing_speed = -(dp_n * dv_n + dp_e * dv_e) / (dist + 1e-06)
                if closing_speed > 0.3:
                    in_emergency = True
                    scale = 5.0 / dist
                    agg_n += dp_n * scale
                    agg_e += dp_e * scale
                    agg_d += -2.0 if self.my_id > drone_id else 2.0
        if in_emergency:
            mag = math.hypot(agg_n, agg_e)
            if mag < 1.0:
                id_hash = sum((ord(c) for c in self.my_id)) * 137 % 360
                angle = math.radians(id_hash)
                agg_n += math.cos(angle) * 4.0
                agg_e += math.sin(angle) * 4.0
                mag = math.hypot(agg_n, agg_e)
            if mag > 6.0:
                agg_n = agg_n / mag * 6.0
                agg_e = agg_e / mag * 6.0
            agg_d = max(-4.0, min(4.0, agg_d))
            self.emergency_mode_active = True
            self.last_emergency_time = current_time
            return (agg_n, agg_e, agg_d)
        if self.emergency_mode_active:
            if current_time - self.last_emergency_time < 0.5:
                # cooldown period - gradually back off, don't just snap to zero
                cool_n, cool_e = (0.0, 0.0)
                for drone_id, data in self.swarm_state.items():
                    if 'pos_n' not in data:
                        continue
                    d_n = my_pos[0] - data['pos_n']
                    d_e = my_pos[1] - data['pos_e']
                    d_dist = math.hypot(d_n, d_e)
                    if 0.01 < d_dist < 5.0:
                        cool_n += d_n / d_dist * (3.0 / d_dist)
                        cool_e += d_e / d_dist * (3.0 / d_dist)
                mag = math.hypot(cool_n, cool_e)
                if mag > 4.0:
                    cool_n = cool_n / mag * 4.0
                    cool_e = cool_e / mag * 4.0
                return (cool_n, cool_e, 0.0)
            else:
                self.emergency_mode_active = False
        return None

    def check_geofence(self, my_pos: Tuple[float, float, float], radius_soft: float=50.0, radius_hard: float=100.0, hard_deck: float=1.0) -> Tuple[bool, bool, Tuple[float, float]]:
        alt = -my_pos[2]
        is_breached = False
        is_near = False
        normal = [0.0, 0.0]
        radius = math.hypot(my_pos[0], my_pos[1])
        if radius > radius_hard or alt < hard_deck:
            is_breached = True
            self.metrics['geofence_breaches'] += 1
            if radius > 0:
                normal = [my_pos[0] / radius, my_pos[1] / radius]
        elif radius > radius_soft:
            is_near = True
            normal = [my_pos[0] / radius, my_pos[1] / radius]
        return (is_breached, is_near, normal)

    def cascade_pid(self, pos, vel, target_pos, target_vel, dt, confidence, max_v, max_a):
        cfg = self.pid_config
        alpha = max(0.0, min(1.0, confidence))
        kp_p = cfg.kp * alpha + cfg.kp_deg * (1 - alpha)
        kp_v = cfg.kp_vel * alpha + cfg.kv_deg * (1 - alpha)
        ki_v = cfg.ki * alpha + cfg.ki_deg * (1 - alpha)
        kd_v = cfg.kd * alpha + cfg.kd_deg * (1 - alpha)
        kp_p_z = cfg.kp_z * alpha + cfg.kp_deg * (1 - alpha)
        kp_v_z = cfg.kp_vel_z * alpha + cfg.kv_deg * (1 - alpha)
        ki_v_z = cfg.ki_z * alpha + cfg.ki_deg * (1 - alpha)
        kd_v_z = cfg.kd_z * alpha + cfg.kd_deg * (1 - alpha)
        v_out = [0.0, 0.0, 0.0]
        for i in range(3):
            cp_p = kp_p_z if i == 2 else kp_p
            cp_v = kp_v_z if i == 2 else kp_v
            ci_v = ki_v_z if i == 2 else ki_v
            cd_v = kd_v_z if i == 2 else kd_v
            err_p = target_pos[i] - pos[i]
            v_cmd = cp_p * err_p
            if target_vel:
                v_cmd += target_vel[i]
            v_cmd = max(-max_v, min(max_v, v_cmd))
            err_v = v_cmd - vel[i]
            v_p = cp_v * err_v
            raw_d = (err_v - self.last_err_v[i]) / dt if dt > 0 else 0.0
            rc = 1.0 / (2 * math.pi * cfg.lpf_cutoff_hz)
            alpha_lpf = dt / (rc + dt) if dt > 0 else 1.0
            d_term = self.last_d_term[i] + alpha_lpf * (raw_d - self.last_d_term[i])
            self.last_d_term[i] = d_term
            self.last_err_v[i] = err_v
            v_d = cd_v * d_term
            a_cmd_raw = v_p + self.i_term_v[i] + v_d
            max_delta_a = cfg.j_max * dt
            a_cmd_jerked = max(self.last_a_cmd[i] - max_delta_a, min(self.last_a_cmd[i] + max_delta_a, a_cmd_raw))
            a_cmd = max(-max_a, min(max_a, a_cmd_jerked))
            self.last_a_cmd[i] = a_cmd
            k_aw = ci_v / cp_p if cp_p > 0 else ci_v
            sat_diff = a_cmd - a_cmd_raw
            self.i_term_v[i] = max(-2.0, min(2.0, self.i_term_v[i] + (ci_v * err_v + k_aw * sat_diff) * dt))
            v_step = vel[i] + a_cmd * dt
            v_out[i] = max(-max_v, min(max_v, v_step))
        return tuple(v_out)

    def calculate_control_output(self, my_pos: Tuple[float, float, float], my_vel: Tuple[float, float, float], mission_target: Optional[Tuple[float, float, float]], dt: float, current_time: float=0.0, max_safe_velocity: float=8.0, a_max: float=4.0, battery_low: bool=False, confidence: float=1.0) -> Tuple[float, float, float]:
        self.prune_stale_telemetry(current_time)
        if self.rtl_triggered:
            return self.cascade_pid(my_pos, my_vel, (0.0, 0.0, -5.0), None, dt, confidence, max_safe_velocity, a_max)
        breached, near, normal = self.check_geofence(my_pos)
        if breached:
            self.rtl_triggered = True
            return self.cascade_pid(my_pos, my_vel, (0.0, 0.0, -5.0), None, dt, confidence, max_safe_velocity, a_max)
        target_v = None
        emergency_vector = self.check_emergency_envelope(my_pos, my_vel, current_time=current_time)
        if emergency_vector:
            self.metrics['min_sep_violations'] += 1
            self.metrics['violation_count'] += 1
            ev = list(emergency_vector)
            for i in range(3):
                ev[i] = max(-max_safe_velocity, min(max_safe_velocity, ev[i]))
            self.last_a_cmd = [0.0, 0.0, 0.0]
            self.i_term_v = [0.0, 0.0, 0.0]
            self.evasion_end_time = 0.0
            return tuple(ev)
        else:
            risks = self.check_collision_course(my_pos, my_vel)
            if risks:
                ev_vel = self.compute_evasion_velocity(my_pos, risks, max_safe_velocity)
                self.active_evasion_v = ev_vel
                self.evasion_end_time = current_time + 1.0
                ev = list(ev_vel)
                for i in range(3):
                    ev[i] = max(-max_safe_velocity, min(max_safe_velocity, ev[i]))
                self.last_a_cmd = [0.0, 0.0, 0.0]
                self.i_term_v = [0.0, 0.0, 0.0]
                return tuple(ev)
            elif current_time < self.evasion_end_time:
                ev = list(self.active_evasion_v)
                for i in range(3):
                    ev[i] = max(-max_safe_velocity, min(max_safe_velocity, ev[i]))
                return tuple(ev)
        if target_v is None:
            active_drones = sum((1 for data in self.swarm_state.values() if current_time - data.get('timestamp', 0) < 3.0))
            if active_drones == 0:
                if self.isolation_start_time is None:
                    self.isolation_start_time = current_time
                iso_duration = current_time - self.isolation_start_time
                if iso_duration < 10.0 and (not battery_low):
                    target_v = (0.0, 0.0, 0.0)
                else:
                    self.rtl_triggered = True
                    return self.cascade_pid(my_pos, my_vel, (0.0, 0.0, -5.0), None, dt, confidence, max_safe_velocity, a_max)
            else:
                self.isolation_start_time = None
            if target_v is None:
                if confidence < 0.5:
                    target_v = (0.0, 0.0, 0.0)
                else:
                    form_v = self.calculate_flocking_velocity(my_pos, my_vel)
                    miss_v = (0.0, 0.0, 0.0)
                    if mission_target:
                        err_n = mission_target[0] - my_pos[0]
                        err_e = mission_target[1] - my_pos[1]
                        err_d = mission_target[2] - my_pos[2]
                        miss_raw = (err_n * 0.5, err_e * 0.5, err_d * 0.5)
                        miss_mag = math.hypot(miss_raw[0], miss_raw[1])
                        if miss_mag > max_safe_velocity:
                            scale = max_safe_velocity / miss_mag
                            miss_v = (miss_raw[0] * scale, miss_raw[1] * scale, miss_raw[2] * scale)
                        else:
                            miss_v = miss_raw
                    target_v = (form_v[0] * 0.5 + miss_v[0] * 0.5, form_v[1] * 0.5 + miss_v[1] * 0.5, form_v[2] * 0.5 + miss_v[2] * 0.5)
        if near:
            v_dot_outward = target_v[0] * normal[0] + target_v[1] * normal[1]
            if v_dot_outward > 0:
                target_v = (target_v[0] - v_dot_outward * normal[0], target_v[1] - v_dot_outward * normal[1], target_v[2])
        v_pre_cbf = target_v
        target_v = self.cbf.enforce_safety(target_v, my_pos, self.swarm_state, max_safe_velocity)
        if target_v != v_pre_cbf:
            self.metrics['cbf_interventions'] += 1
        v_out = [0.0, 0.0, 0.0]
        cfg = self.pid_config
        alpha = max(0.0, min(1.0, confidence))
        kp_v = cfg.kp_vel * alpha + cfg.kv_deg * (1 - alpha)
        kd_v = cfg.kd * alpha + cfg.kd_deg * (1 - alpha)
        for i in range(3):
            err_v = target_v[i] - my_vel[i]
            v_p = kp_v * err_v
            raw_d = safe_div(err_v - self.last_err_v[i], dt, 0.0)
            rc = 1.0 / (2 * math.pi * cfg.lpf_cutoff_hz)
            alpha_lpf = safe_div(dt, rc + dt, 1.0)
            d_term = self.last_d_term[i] + alpha_lpf * (raw_d - self.last_d_term[i])
            self.last_d_term[i] = d_term
            self.last_err_v[i] = err_v
            v_d = kd_v * d_term
            a_cmd_raw = v_p + self.i_term_v[i] + v_d
            max_delta_a = cfg.j_max * dt
            a_cmd_jerked = max(self.last_a_cmd[i] - max_delta_a, min(self.last_a_cmd[i] + max_delta_a, a_cmd_raw))
            a_cmd = max(-a_max, min(a_max, a_cmd_jerked))
            self.last_a_cmd[i] = a_cmd
            k_aw = 0.1
            sat_diff = a_cmd - a_cmd_raw
            self.i_term_v[i] = max(-2.0, min(2.0, self.i_term_v[i] + (0.1 * err_v + k_aw * sat_diff) * dt))
            v_step = my_vel[i] + a_cmd * dt
            v_out[i] = max(-max_safe_velocity, min(max_safe_velocity, v_step))
        return sanitize_output(tuple(v_out), max_safe_velocity)
