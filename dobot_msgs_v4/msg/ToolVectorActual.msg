# The controller's pose of the active tool (TCP) in the active user frame,
# from the realtime feedback; published only while it is fresh.
std_msgs/Header header           # stamp: when published; frame_id: base_link
int8 user                        # active user frame (0: the robot base)
int8 tool                        # active tool (0: the flange)
float64 x
float64 y
float64 z
float64 rx
float64 ry
float64 rz