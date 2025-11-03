# Operational State Message
# Represents the current operational mode of the rover system

# State constants
uint8 STARTUP = 0      # System initializing
uint8 STANDBY = 1      # Ready but not active
uint8 AUTONOMOUS = 2   # Autonomous operation mode
uint8 MANUAL = 3       # Manual/teleoperation mode
uint8 EMERGENCY = 4    # Emergency stop engaged
uint8 ERROR = 5        # System error state
uint8 SHUTDOWN = 6     # System shutting down

# Current state
uint8 state

# Timestamp of last state change
builtin_interfaces/Time timestamp

# Human-readable state description
string description
