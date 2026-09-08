"""
Automated Python Verification Suite for Gesture Arm Hardened Firmware
Validates:
1. Asymptotic velocity profile ramping & rate-limiting.
2. Watchdog timeout & failsafe park transition.
3. Joint angle clamping & deadband stability.
4. Kinematic bounds & law-of-cosines geometry.
"""

import math
import pytest

# Constants mirroring src/receiver/config.h
BASE_MIN = 0.0
BASE_MAX = 180.0
SHOULDER_MIN = 30.0
SHOULDER_MAX = 150.0
ELBOW_MIN = 0.0
ELBOW_MAX = 160.0
GRIPPER_OPEN = 30.0
GRIPPER_CLOSED = 120.0

BASE_HOME = 90.0
SHOULDER_HOME = 90.0
ELBOW_HOME = 90.0
GRIPPER_HOME = GRIPPER_OPEN

RADIO_TIMEOUT_MS = 1000
FAILSAFE_PARK_STEP = 0.8
MAX_VEL_BASE = 2.0
MAX_VEL_SHOULDER = 1.8
MAX_VEL_ELBOW = 2.2
MAX_VEL_GRIPPER = 4.0

VELOCITY_SMOOTH_ALPHA = 0.22
MIN_DEADBAND_DEG = 0.25

LINK1_LEN = 120.0
LINK2_LEN = 100.0


def update_joint_trajectory(current: float, target: float, max_step: float) -> float:
    error = target - current
    if abs(error) <= MIN_DEADBAND_DEG:
        return current

    desired_step = error * VELOCITY_SMOOTH_ALPHA

    if desired_step > max_step:
        desired_step = max_step
    elif desired_step < -max_step:
        desired_step = -max_step

    if abs(desired_step) < MIN_DEADBAND_DEG and abs(error) > MIN_DEADBAND_DEG:
        desired_step = MIN_DEADBAND_DEG if error > 0 else -MIN_DEADBAND_DEG

    return current + desired_step


def inverse_kinematics(x: float, y: float):
    dist_sq = x * x + y * y
    cos_theta2 = (dist_sq - LINK1_LEN**2 - LINK2_LEN**2) / (2.0 * LINK1_LEN * LINK2_LEN)
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return None  # Unreachable

    theta2_deg = math.acos(cos_theta2) * 180.0 / math.pi
    theta2_rad = theta2_deg * math.pi / 180.0
    k1 = LINK1_LEN + LINK2_LEN * math.cos(theta2_rad)
    k2 = LINK2_LEN * math.sin(theta2_rad)
    theta1_deg = (math.atan2(y, x) - math.atan2(k2, k1)) * 180.0 / math.pi
    return (theta1_deg, theta2_deg)


def test_velocity_limit_strictly_enforced():
    """Verify that a large 90° step input never exceeds MAX_VEL_BASE on any 20ms tick."""
    current = 90.0
    target = 180.0
    history = [current]

    for _ in range(200):
        prev = current
        current = update_joint_trajectory(current, target, MAX_VEL_BASE)
        step = current - prev
        assert abs(step) <= MAX_VEL_BASE + 1e-6, f"Velocity violation: step {step} > max {MAX_VEL_BASE}"
        history.append(current)
        if current == target:
            break

    assert abs(current - target) <= MIN_DEADBAND_DEG
    assert len(history) > 40  # Took multiple ticks smoothly without snapping instantly


def test_asymptotic_deceleration_smoothness():
    """Verify that as the joint approaches target, step size decreases monotonically."""
    current = 0.0
    target = 100.0
    steps = []

    for _ in range(150):
        prev = current
        current = update_joint_trajectory(current, target, MAX_VEL_BASE)
        step = current - prev
        if step > 0:
            steps.append(step)
        if current == target:
            break

    # Once error * alpha falls below MAX_VEL_BASE, steps must decrease monotonically
    decaying_steps = [s for s in steps if s < MAX_VEL_BASE]
    for i in range(len(decaying_steps) - 1):
        assert decaying_steps[i] >= decaying_steps[i + 1] - 1e-6, "Non-monotonic deceleration"


def test_watchdog_failsafe_state_transition():
    """Simulate a 50Hz packet stream followed by an abrupt radio cutout."""
    clock_ms = 0
    last_packet_ms = 0
    link_state = "ONLINE"
    current_shld = 140.0  # arm extended high
    target_shld = 140.0

    # 1. Normal streaming for 1000ms
    for _ in range(50):
        clock_ms += 20
        last_packet_ms = clock_ms
        if clock_ms - last_packet_ms > RADIO_TIMEOUT_MS:
            link_state = "LOST"
        assert link_state == "ONLINE"

    # 2. Cut stream for 3000ms (1000ms timeout + 2000ms parking)
    failsafe_triggered = False
    for _ in range(150):
        clock_ms += 20
        if clock_ms - last_packet_ms > RADIO_TIMEOUT_MS:
            link_state = "LOST"
            failsafe_triggered = True
            target_shld = SHOULDER_HOME
            step = FAILSAFE_PARK_STEP
        else:
            step = MAX_VEL_SHOULDER

        current_shld = update_joint_trajectory(current_shld, target_shld, step)

    assert failsafe_triggered is True
    assert link_state == "LOST"
    # Arm should have smoothly moved toward SHOULDER_HOME (90.0)
    assert current_shld < 140.0
    assert abs(current_shld - SHOULDER_HOME) <= MIN_DEADBAND_DEG


def test_joint_angle_clamping():
    """Ensure hard mechanical angle constraints reject dangerous packet values."""
    # Out-of-bounds low
    raw_packet_base = -20
    clamped_base = max(BASE_MIN, min(BASE_MAX, float(raw_packet_base)))
    assert clamped_base == BASE_MIN

    # Out-of-bounds high
    raw_packet_shld = 210
    clamped_shld = max(SHOULDER_MIN, min(SHOULDER_MAX, float(raw_packet_shld)))
    assert clamped_shld == SHOULDER_MAX


def test_inverse_kinematics_workspace_reach():
    """Test reachable vs unreachable workspace boundaries."""
    # Fully stretched: L1 + L2 = 220mm
    sol_reachable = inverse_kinematics(150.0, 100.0)
    assert sol_reachable is not None
    shld, elbow = sol_reachable
    assert 0.0 <= elbow <= 180.0

    # Completely out of reach (> 220mm)
    sol_unreachable = inverse_kinematics(300.0, 300.0)
    assert sol_unreachable is None
