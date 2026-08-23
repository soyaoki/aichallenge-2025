"""Plan -> action helpers for comma.ai's driving_supercombo network.

Ported from openpilot ``openpilot/selfdrive/controls/lib/drive_helpers.py`` at
commit 084747c7 (MIT, comma.ai), reduced to the functions ``modeld`` uses to
turn a model plan into a desired acceleration / curvature. See
``THIRD_PARTY_NOTICES.md``.
"""

import numpy as np

DT_MDL = 0.05  # openpilot model step (20 Hz)
DT_CTRL = 0.01  # openpilot control step (100 Hz)

MIN_SPEED = 1.0
MIN_STABLE_DELAY = 0.3

ACCELERATION_DUE_TO_GRAVITY = 9.81

# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2


def should_stop(v_ego: float, a_target: float) -> bool:
    return bool(v_ego < 0.3 and a_target < 0.1)


def clamp(val, min_val, max_val):
    clamped_val = float(np.clip(val, min_val, max_val))
    return clamped_val, clamped_val != val


def smooth_value(val, prev_val, tau, dt=DT_MDL):
    alpha = 1 - np.exp(-dt / tau) if tau > 0 else 1
    return alpha * val + (1 - alpha) * prev_val


def clip_curvature(v_ego, prev_curvature, new_curvature, roll=0.0, dt=DT_CTRL,
                   max_lateral_jerk=MAX_LATERAL_JERK,
                   max_lateral_accel=MAX_LATERAL_ACCEL_NO_ROLL,
                   max_curvature=MAX_CURVATURE):
    """Respect ISO lateral jerk and acceleration limits plus a max curvature."""
    v_ego = max(v_ego, MIN_SPEED)
    max_curvature_rate = max_lateral_jerk / (v_ego ** 2)
    new_curvature = np.clip(new_curvature,
                            prev_curvature - max_curvature_rate * dt,
                            prev_curvature + max_curvature_rate * dt)

    roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
    max_lat_accel = max_lateral_accel + roll_compensation
    min_lat_accel = -max_lateral_accel + roll_compensation
    new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

    new_curvature, limited_max_curv = clamp(new_curvature, -max_curvature, max_curvature)
    return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL):
    if len(speeds) == len(t_idxs):
        v_now = speeds[0]
        a_now = accels[0]
        if action_t < MIN_STABLE_DELAY:
            v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
        else:
            v_target = np.interp(action_t, t_idxs, speeds)
        a_target = 2 * (v_target - v_now) / (action_t) - a_now
    else:
        a_target = 0.0
    return a_target


def curv_from_psis(psi_target, psi_rate, vego, action_t):
    vego = np.clip(vego, MIN_SPEED, np.inf)
    curv_from_psi = psi_target / (vego * action_t)
    return 2 * curv_from_psi - psi_rate / vego


def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
    if action_t < MIN_STABLE_DELAY:
        psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
    else:
        psi_target = np.interp(action_t, t_idxs, yaws)
    psi_rate = yaw_rates[0]
    return curv_from_psis(psi_target, psi_rate, vego, action_t)
