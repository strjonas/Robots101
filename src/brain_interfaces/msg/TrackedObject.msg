std_msgs/Header header
string track_id
string label
float32 confidence
# Bounding box in the image, as fractions of the image size (0..1, origin top-left).
float32 center_x
float32 center_y
float32 width
float32 height
# Direction of the object seen from the robot, radians, positive = to the left.
float32 bearing_rad
float32 estimated_distance_m
# "lidar" if the distance was measured by the lidar at the object's bearing,
# "size" if it was guessed from the box height and a typical object size.
string distance_source
bool actively_tracked
