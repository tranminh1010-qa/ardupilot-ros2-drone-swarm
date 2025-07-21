import enum


class DroneState(enum.Enum):
    """Operational states of the drone"""
    # Connection states
    INITIALIZING = 0
    CONNECTING = 1
    CONNECTED = 2

    # Pre-flight states
    DISARMED = 3
    ARMING = 4
    ARMED = 5

    # Operation states
    TAKING_OFF = 6
    FLYING = 7
    RETURNING = 8
    LANDING = 9
    LANDED = 10

    # Error states
    ERROR = 11
    FAILSAFE = 12


class FlightMode(enum.Enum):
    """ArduCopter flight mode numbers"""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17