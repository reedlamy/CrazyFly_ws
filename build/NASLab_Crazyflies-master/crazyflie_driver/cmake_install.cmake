# Install script for directory: /home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/reed/CrazyFly_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/srv" TYPE FILE FILES
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/AddCrazyflie.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/GoTo.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/Land.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/RemoveCrazyflie.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/SetGroupMask.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/StartTrajectory.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/Stop.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/Takeoff.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/UpdateParams.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/UploadTrajectory.srv"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/srv/sendPacket.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/msg" TYPE FILE FILES
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/LogBlock.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/GenericLogData.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/FullState.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/TrajectoryPolynomialPiece.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/crtpPacket.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/Hover.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/Position.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/HoverVel.msg"
    "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/msg/NameArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES "/home/reed/CrazyFly_ws/build/NASLab_Crazyflies-master/crazyflie_driver/catkin_generated/installspace/crazyflie_driver-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/reed/CrazyFly_ws/devel/include/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/reed/CrazyFly_ws/devel/share/roseus/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/reed/CrazyFly_ws/devel/share/common-lisp/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/reed/CrazyFly_ws/devel/share/gennodejs/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/reed/CrazyFly_ws/devel/lib/python2.7/dist-packages/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/reed/CrazyFly_ws/devel/lib/python2.7/dist-packages/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/reed/CrazyFly_ws/build/NASLab_Crazyflies-master/crazyflie_driver/catkin_generated/installspace/crazyflie_driver.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES "/home/reed/CrazyFly_ws/build/NASLab_Crazyflies-master/crazyflie_driver/catkin_generated/installspace/crazyflie_driver-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES
    "/home/reed/CrazyFly_ws/build/NASLab_Crazyflies-master/crazyflie_driver/catkin_generated/installspace/crazyflie_driverConfig.cmake"
    "/home/reed/CrazyFly_ws/build/NASLab_Crazyflies-master/crazyflie_driver/catkin_generated/installspace/crazyflie_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver" TYPE FILE FILES "/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_driver/package.xml")
endif()

