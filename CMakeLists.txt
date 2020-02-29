# Project's name
PROJECT (phoenix-template)

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED)
find_package (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

add_executable(SimpleJoystick ${PROJECT_SOURCE_DIR}/src/SimpleJoystick.cpp)
add_executable(TalonPercentOutput ${PROJECT_SOURCE_DIR}/src/TalonPercentOutput.cpp)
add_executable(CanifierServoTest ${PROJECT_SOURCE_DIR}/src/CanifierServoTest.cpp)

target_link_libraries(SimpleJoystick ${catkin_LIBRARIES})
target_link_libraries(TalonPercentOutput ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(CanifierServoTest ${catkin_LIBRARIES} Threads::Threads)

# Phoenix includes
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi

    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixCanutils.a)
    
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/raspberry/libCTRE_PhoenixCanutils.a)

else()

    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(TalonPercentOutput /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixCanutils.a)
    
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(CanifierServoTest /home/$ENV{USERNAME}/catkin_ws/src/phoenix-template/lib/x86-64/libCTRE_PhoenixCanutils.a)

endif()
