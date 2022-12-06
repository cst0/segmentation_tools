cmake_minimum_required(VERSION 3.0.2)
project(segmentation_tools)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy sensor_msgs visualization_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_segment_ransac_node src/segment_ransac.cpp)
add_executable(${PROJECT_NAME}_segment_bounding_box_node
               src/segment_bounding_box.cpp)
add_executable(${PROJECT_NAME}_group_node src/group.cpp)

set_target_properties(${PROJECT_NAME}_segment_ransac_node
                      PROPERTIES OUTPUT_NAME segment_ransac)
set_target_properties(${PROJECT_NAME}_segment_bounding_box_node
                      PROPERTIES OUTPUT_NAME segment_bounding_box)
set_target_properties(${PROJECT_NAME}_group_node PROPERTIES OUTPUT_NAME group)

add_dependencies(${PROJECT_NAME}_segment_ransac_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_segment_bounding_box_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_group_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_segment_ransac_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_segment_bounding_box_node
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_group_node ${catkin_LIBRARIES})
