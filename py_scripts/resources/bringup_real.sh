# Run roscore if not already running
ROS_ID=$(pgrep -x roscore)
if [ -z "$ROS_ID" ]
then
	gnome-terminal -e roscore
	sleep 3
fi
# Launch RVIZ and navigation stack if not already running
RVIZ_ID=$(pgrep -x rviz)
if [ -z "$RVIZ_ID" ]
then
	gnome-terminal -e "roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$1/$2.yaml"
	sleep 10
fi