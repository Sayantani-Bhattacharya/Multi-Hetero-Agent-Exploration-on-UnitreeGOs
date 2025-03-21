# High level command message.

# Fixed variables
# Fixed
uint8[2] head
# Fixed control level being high level. 
uint8 level_flag
# Fixed
uint8 frame_reserve

#
#
uint32[2] sn
#
uint32[2] version
#
uint16 band_width
# Can be 0, 1, 2, 5, 6
# 0 : Body Pose Control, yaw through body pose ??
# 1 : Body Pose Control, yaw through body pose
# 2 : Velocity Control, yaw through steering
# 3 : Continuous motion ??
# 5 : Damping State
# 6 : Lock Joints and enter prone state
uint8 mode 
#
# 1 : For x and y velocities.
# 2 : Unstable and more  aerial ??
uint8 gait_type		   
#
uint8 speed_level			   
#
float32 foot_raise_height		   
# Z-body pose. Z+ is up.
float32 body_height	   
# 
float32[2] position 
# Roll, Pitch, Yaw (-0.2 - 0.2).
float32[3] euler	   
# [X_dot, Y_dot] X+ is straight ahead, Y+ is to the left.
float32[2] velocity 
# theta_dot, Z+ is up.
float32 yaw_speed
# 			   
BmsCmd bms
#
LED[4] led
#
uint8[40] wireless_remote
#
uint32 reserve

#
uint32 crc