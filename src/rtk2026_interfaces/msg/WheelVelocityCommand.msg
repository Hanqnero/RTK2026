# Legacy wheel velocity command from Pi to Arduino.
# Active robot stack now sends cmd_vel to Arduino bridge directly.
# Units: ticks/sec (signed), historically converted from cmd_vel by base_controller.

int16 left_tps    # target velocity, left wheel (ticks/sec)
int16 right_tps   # target velocity, right wheel (ticks/sec)
