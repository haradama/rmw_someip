# Vehicle status, sent from the native SOME/IP node to ROS 2.
#
# Bytes after SOME/IP native serialization, big endian and packed:
#   timestamp_ms        4 bytes
#   speed_kph           4 bytes, IEEE 754 float
#   steering_angle_deg  4 bytes, IEEE 754 float
#   gear                1 byte
#   mode                4-byte length including the NUL, then the body, then NUL
uint32 timestamp_ms
float32 speed_kph
float32 steering_angle_deg
uint8 gear
string mode
