# Run roscore if not already running
ROS_ID=$(pgrep -x roscore)
if [ -z "$ROS_ID" ]
then
	gnome-terminal -e roscore
	sleep 3
fi
# Launch VRep if not already running
VREP_ID=$(pgrep -x vrep)
if [ -z "$VREP_ID" ]
then
	gnome-terminal -e "$1/vrep.sh $2/$3.ttt"
	sleep 10
fi