# Encoder telemetry from Arduino
# Published at fixed control period (e.g. 50Hz)
# Counts are cumulative since Arduino startup

std_msgs/Header header

int32 left_count   # cumulative tick count, left wheel (signed)
int32 right_count  # cumulative tick count, right wheel (signed)
