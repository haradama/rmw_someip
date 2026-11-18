# A command to the vehicle, sent from ROS 2 to the native SOME/IP node.
#
# Bytes after SOME/IP native serialization, big endian and packed:
#   target_speed_kph  4 bytes, IEEE 754 float
#   gear_request      1 byte
float32 target_speed_kph
uint8 gear_request
