# Trajectory point state

# Frame and time of this trajectory sample.
std_msgs/Header header

# Pose of the trajectory sample.
geometry_msgs/Pose pose

# Whether velocity is specified. False means unspecified, not zero.
bool velocity_valid

# Velocity of the trajectory sample, used only when velocity_valid is true.
geometry_msgs/Twist velocity

# Whether acceleration is specified. False means unspecified, not zero.
bool acceleration_valid

# Acceleration of the trajectory sample, used only when acceleration_valid is true.
geometry_msgs/Accel acceleration

# Force/Torque to apply at trajectory sample (optional).
geometry_msgs/Wrench effort
