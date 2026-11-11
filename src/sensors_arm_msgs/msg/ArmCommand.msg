# High-level arm command (mirrors sensors-arm /api/arm/command)
string arm_id
string action
# actions: arm|disarm|estop|joints|jog|home|abs_joints|cancel_ramp|connect|disconnect
float64[] joints
float64[] jog_delta
float64 speed_scale
bool cancel_abs_ramp
