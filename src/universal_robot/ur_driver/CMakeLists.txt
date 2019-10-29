cmake_minimum_required(VERSION 2.8.3)
project(ur_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp tf ur_msgs)

catkin_python_setup()


generate_dynamic_reconfigure_options(
  cfg/URDriver.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure message_runtime
    LIBRARIES roboticArm URcontrolV2
)

###########
## Build ##
###########
include_directories(include
${catkin_INCLUDE_DIRS})
#include_directories(include)

add_library(roboticArm src/roboticArm.cpp)
target_link_libraries(roboticArm ${catkin_LIBRARIES})

add_library(URcontrolV2 src/URcontrolV2.cpp)
target_link_libraries(URcontrolV2 ${catkin_LIBRARIES})

add_executable(test_move_cpp src/test_move.cpp)
add_dependencies(test_move_cpp ur_msgs_gencpp)
target_link_libraries(test_move_cpp ${catkin_LIBRARIES} roboticArm)

add_executable(test_pick_place_cpp src/test_pick_place.cpp src/roboticArm.cpp include/roboticArm.h)
add_dependencies(test_pick_place_cpp ur_msgs_gencpp)
target_link_libraries(test_pick_place_cpp ${catkin_LIBRARIES} roboticArm)

#############
## Install ##
#############
install(PROGRAMS test_move.py test_io.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/ur_driver/driver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES prog prog_reset
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
