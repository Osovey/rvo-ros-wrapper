cmake_minimum_required(VERSION 2.8.3)
project(player_map)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs map_server)

add_service_files(FILES GetMap.srv)
generate_messages(DEPENDENCIES nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rvoplayermap
  CATKIN_DEPENDS roscpp nav_msgs map_server)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(rvoplayermap src/map.c src/rosmap.cpp)
target_link_libraries(rvoplayermap ${catkin_LIBRARIES})
add_dependencies(rvoplayermap ${PROJECT_NAME}_gencpp)

add_executable(rvo_draw_paths src/draw_paths.cpp)
target_link_libraries(rvo_draw_paths rvoplayermap)

catkin_add_gtest(test_rosmap src/rosmap_test.cpp)
target_link_libraries(test_rosmap rvoplayermap)
