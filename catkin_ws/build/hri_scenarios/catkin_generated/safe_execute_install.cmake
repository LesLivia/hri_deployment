execute_process(COMMAND "/home/parallels/Desktop/hri_deployment/catkin_ws/build/hri_scenarios/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/parallels/Desktop/hri_deployment/catkin_ws/build/hri_scenarios/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
