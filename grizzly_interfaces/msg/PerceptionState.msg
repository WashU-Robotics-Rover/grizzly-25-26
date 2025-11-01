# PerceptionState.msg
#
# Represents the complete perception state of the Grizzly rover at a given moment.
# This message combines pose, velocity, and terrain information for state estimation.
#
# Used by: central_perception (publisher), planners (subscribers)
# Topic: /perception/state (typical)

# Header contains timestamp and frame information
# - stamp: Time when this state was measured/estimated
# - frame_id: Reference frame (e.g., "map", "odom", "base_link")
std_msgs/Header header

# Robot's position and orientation with uncertainty
# - pose.pose: Position (x, y, z) and orientation (quaternion)
# - pose.covariance: 6x6 covariance matrix representing uncertainty in pose
#   [x, y, z, roll, pitch, yaw] - diagonal elements are variances
geometry_msgs/PoseWithCovariance pose

# Robot's linear and angular velocity with uncertainty
# - twist.twist: Linear velocity (vx, vy, vz) and angular velocity (wx, wy, wz)
# - twist.covariance: 6x6 covariance matrix representing uncertainty in velocities
geometry_msgs/TwistWithCovariance twist

# Terrain classification for current location
# Used for planning traction, speed, and safety
# Values:
#   0 = unknown    - terrain not yet classified
#   1 = flat       - smooth, hard surface (optimal for driving)
#   2 = sand       - soft, granular surface (reduced traction)
#   3 = rocky      - uneven, hard obstacles (slow, careful navigation)
uint8 terrain_class
