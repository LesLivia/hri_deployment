#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/parallels/Desktop/hri_deployment/catkin_ws/src/hri_scenarios"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/parallels/Desktop/hri_deployment/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/parallels/Desktop/hri_deployment/catkin_ws/install/lib/python2.7/dist-packages:/home/parallels/Desktop/hri_deployment/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/parallels/Desktop/hri_deployment/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/parallels/Desktop/hri_deployment/catkin_ws/src/hri_scenarios/setup.py" \
    egg_info --egg-base /home/parallels/Desktop/hri_deployment/catkin_ws/build/hri_scenarios \
    build --build-base "/home/parallels/Desktop/hri_deployment/catkin_ws/build/hri_scenarios" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/parallels/Desktop/hri_deployment/catkin_ws/install" --install-scripts="/home/parallels/Desktop/hri_deployment/catkin_ws/install/bin"
