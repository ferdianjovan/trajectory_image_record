cmake_minimum_required(VERSION 2.8.3)
project(trajectory_image_record)

find_package(catkin REQUIRED COMPONENTS
  human_trajectory
  rospy
  sensor_msgs
  std_msgs
  message_generation
  mongodb_store
  message_filters
  bayes_people_tracker
  geometry_msgs
  tf
  actionlib
  actionlib_msgs
  flir_pantilt_d46
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ImageRecord.msg
)

add_action_files(
    DIRECTORY action
    FILES RecordImage.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs actionlib actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS human_trajectory rospy sensor_msgs std_msgs message_runtime mongodb_store message_filters bayes_people_tracker geometry_msgs tf actionlib actionlib_msgs flir_pantilt_d46
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trajectory_image_record trajectory_image_record_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_image_record.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
