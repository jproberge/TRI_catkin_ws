execute_process(COMMAND "/home/bdml/catkin_ws/build/robotiq/robotiq_3f_gripper_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bdml/catkin_ws/build/robotiq/robotiq_3f_gripper_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
