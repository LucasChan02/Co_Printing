execute_process(COMMAND "/home/ubuntu/armpi_pro/build/armpi_pro_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/armpi_pro/build/armpi_pro_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
