# A benchmark sample.
#
# Carries the steady_clock reading from just before the send. Publisher and
# subscriber are separate processes on one host and share that clock, so the
# difference is the one-way latency. Using a ROS Time would fold rcl's own clock
# lookup into the measurement.
uint64 stamp_ns

# Serial number, for spotting loss and duplication.
uint64 sequence

# Filler; its size is what varies across the payload sweep.
uint8[] data
