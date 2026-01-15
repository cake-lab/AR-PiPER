# Piper-specific teleop metric (unique name to avoid collisions)
# header.stamp MUST be the ORIGINAL camera-capture time from the WS bridge
std_msgs/Header header

# Cartesian target pose (in the chosen base frame)
geometry_msgs/Pose target_pose

# Time spent inside IK solver (post_ik - pre_ik), in milliseconds
float64 ik_solver_delay_ms

# End-to-end latency at control publish time:
# now_at_joint_command_publish - header.stamp  (in ms)
float64 e2e_latency_ms_at_control

# Reserved fields for future expansion to keep schema stable
float64 reserved_1
float64 reserved_2

