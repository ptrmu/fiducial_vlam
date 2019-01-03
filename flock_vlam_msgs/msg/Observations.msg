# An observation of several fiducials

std_msgs/Header header

# If vloc figured out a pose for the camera in the map frame, it will pass it along to vmap. The field camera_id
# will be empty if no camera pose was determined
string camera_id
std_msgs/Header map_header
geometry_msgs/PoseWithCovariance camera_pose

# A list of locations of fiducial corners in the image
sensor_msgs/CameraInfo camera_info
ObservationsEntry[] observations