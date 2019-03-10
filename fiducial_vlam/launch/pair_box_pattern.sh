

echo Starting flight pattern

function move_two() {
ros2 topic pub -1 /drone1/cmd_vel geometry_msgs/Twist "{linear: {$1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -1 /drone2/cmd_vel geometry_msgs/Twist "{linear: {$1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep $2
ros2 topic pub -1 /drone1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -1 /drone2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 0.25
}

function cmd_two() {
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: '$1'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: '$1'}"
sleep $2
}

function move_square() {
move_two("x: 0.0, y: 0.3, z: 0.0", 0.25)
move_two("x: -0.3, y: 0.0, z: 0.0", 0.25)
move_two("x: 0.0, y: -0.3, z: 0.0", 0.25)
move_two("x: 0.3, y: 0.0, z: 0.0", 0.25)
}

cmd_two("takeoff", 10)

move_square()
move_square()

cmd_two("land", 5)

