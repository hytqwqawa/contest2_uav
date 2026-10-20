"""Pure-Python continuity checker; it vetoes, never modifies, setpoints."""
import math
MAX_POS_M = 0.25
MAX_YAW_RAD = math.radians(12.)
REQUIRED_VALID = 1 | 2 | 4 | 1024  # PX/PY/PZ/YAW ignored bits

def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))

def check(previous, candidate):
    if candidate is None: return 'candidate_missing'
    # MAVROS PositionTarget must use FRAME_LOCAL_NED (value 1).
    # The XYZ/YAW values here are ROS-ENU, converted by MAVROS downstream.
    if int(candidate.coordinate_frame) != 1: return 'unsupported_coordinate_frame'
    if int(candidate.type_mask) & REQUIRED_VALID: return 'xyz_or_yaw_ignored'
    vals = [candidate.position.x, candidate.position.y,
            candidate.position.z, candidate.yaw]
    if not all(math.isfinite(float(v)) for v in vals): return 'nonfinite'
    if previous is None: return None
    if int(previous.type_mask) & REQUIRED_VALID: return 'reference_xyz_or_yaw_ignored'
    if int(previous.coordinate_frame) != int(candidate.coordinate_frame):
        return 'frame_mismatch'
    d = math.sqrt(sum((a-b)**2 for a,b in zip(
        (candidate.position.x,candidate.position.y,candidate.position.z),
        (previous.position.x,previous.position.y,previous.position.z))))
    if d > MAX_POS_M: return 'position_jump_%.3fm' % d
    a = abs(angle_delta(candidate.yaw, previous.yaw))
    if a > MAX_YAW_RAD: return 'yaw_jump_%.1fdeg' % math.degrees(a)
    return None

# POINT publishes exactly zero velocity, acceleration and yaw_rate while
# AIRBORNE_HOLD. A stale trajectory sample must not be accepted as a HOLD
# even if its XYZ and yaw happen to match the latest measured position.
def point_hold_feedforward_reason(candidate):
    if candidate is None:
        return 'point_hold_missing'
    mask = int(candidate.type_mask)
    for group, fields, flags, limit in (
        ('velocity', ('x','y','z'), (8,16,32), 0.05),
        ('acceleration', ('x','y','z'), (64,128,256), 0.15),
    ):
        vector = (candidate.velocity if group == 'velocity'
                  else candidate.acceleration_or_force)
        for name, ignore_bit in zip(fields, flags):
            if mask & ignore_bit:
                continue
            value = float(getattr(vector,name))
            if not math.isfinite(value):
                return 'point_hold_nonfinite_' + group
            if abs(value) > limit:
                return 'point_hold_nonzero_' + group
    if not mask & 2048:
        rate = float(candidate.yaw_rate)
        if not math.isfinite(rate):
            return 'point_hold_nonfinite_yaw_rate'
        if abs(rate) > 0.05:
            return 'point_hold_nonzero_yaw_rate'
    return None
