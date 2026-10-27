cmake_minimum_required(VERSION 3.5.1)
project(gazebo_marker_visuals)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros visualization_msgs geometry_msgs)
find_package(gazebo REQUIRED)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs visualization_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_STANDARD 17)

add_library(gazebo_marker_visuals src/marker_visuals_plugin.cpp)
target_link_libraries(gazebo_marker_visuals ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_marker_visuals LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_marker_store test/test_marker_store.cpp)
  target_link_libraries(test_marker_store ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
endif()
