# Details of a detected object for RoboCup Rescue
std_msgs/Header header

# Type of object: "ar_code", "hazmat_sign", "real_object", "heat_sig"
string type

# Unique ID/Name: "2", "poison", "gloves"
string name

# Pose of the object (position and orientation)
geometry_msgs/Pose pose

# Name of the robot that detected the object
string robot_name

# Mode of operation: "A" (Autonomous) or "T" (Teleoperated)
string mode

# Confidence score (0.0 to 1.0)
float32 confidence
