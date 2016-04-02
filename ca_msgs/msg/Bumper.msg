Header header

# Contact sensors
bool is_left_pressed
bool is_right_pressed

# Bumper light sensors (Create 2 only) in order from left to right
# Value = true if an obstacle detected
bool is_light_left
bool is_light_front_left
bool is_light_center_left
bool is_light_center_right
bool is_light_front_right
bool is_light_right

# Raw light sensor signals
# Values in range [0, 4095]
uint16 light_signal_left
uint16 light_signal_front_left
uint16 light_signal_center_left
uint16 light_signal_center_right
uint16 light_signal_front_right
uint16 light_signal_right
