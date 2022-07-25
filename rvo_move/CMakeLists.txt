cmake_minimum_required(VERSION 2.8.3)
project(rvo_move)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs nav_msgs rvo2
  actionlib_msgs actionlib tf player_map)

add_action_files(DIRECTORY action FILES Move.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES botclient
  CATKIN_DEPENDS roscpp visualization_msgs nav_msgs rvo2
    actionlib_msgs actionlib tf player_map)

include_directories(include ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(botclient src/bot_client.cpp)
target_link_libraries(botclient ${catkin_LIBRARIES})

add_executable(move_server src/move_server.cpp src/rvo_wrapper.cpp)
target_link_libraries(move_server botclient)

add_executable(move_client src/move_client.cpp)
target_link_libraries(move_client botclient)

add_executable(test_node example/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})