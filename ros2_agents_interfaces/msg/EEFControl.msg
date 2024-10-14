uint8 ABSOLUTE=0
uint8 RELATIVE=1
uint8 VELOCITY=2
uint8 EFFORT=3


string frame_id "world"
float64 grasp # 0.0 is open, 1.0 is closed
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
uint8 control_type 1