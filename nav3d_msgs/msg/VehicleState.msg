std_msgs/Header header

# arming_state.
uint8 ARMING_UNKNOWN=0
uint8 DISARMED=1
uint8 ARMED=2
uint8 arming_state

# ground_state.
uint8 GROUND_UNKNOWN=0
uint8 ON_GROUND=1
uint8 IN_AIR=2
uint8 ground_state

# EXTERNAL_CONTROL means a flight controller such as PX4 executes the commands;
# DIRECT_CONTROL is reserved for a future controller operating directly on the actuators.
uint8 CONTROL_UNKNOWN=0
uint8 DIRECT_CONTROL=1
uint8 EXTERNAL_CONTROL=2
uint8 control_state

# failsafe_state.
uint8 FAILSAFE_UNKNOWN=0
uint8 FAILSAFE_INACTIVE=1
uint8 FAILSAFE_ACTIVE=2
uint8 failsafe_state
