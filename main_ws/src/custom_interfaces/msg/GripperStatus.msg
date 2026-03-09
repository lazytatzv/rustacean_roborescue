# GripperStatus: status feedback from the gripper
# States: 0=IDLE, 1=MOVING, 2=GRIPPING, 3=OVERLOAD, 4=ERROR
uint8 IDLE=0
uint8 MOVING=1
uint8 GRIPPING=2
uint8 OVERLOAD=3
uint8 ERROR=4

uint8 state
# Current position in Dynamixel raw units
int32 position
# Present current in mA
int16 current
# Present temperature in degrees Celsius
uint8 temperature
