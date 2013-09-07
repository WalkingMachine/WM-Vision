cmake_minimum_required(VERSION 2.8.3)

project(wm_vision_kernel) 

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()
 
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp rospy message_generation cv_bridge pcl_ros)
find_package(OpenCV 2.4 REQUIRED)
find_package(PCL 1.6 REQUIRED COMPONENTS)

add_service_files(
  FILES
  wm_vision_interface_flow.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs roscpp rospy message_runtime
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME})

include_directories(cfvf include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS} ${PCL_LIBRARY_DIRS})

add_library(lib${PROJECT_NAME} SHARED
  src/vision_kernel/vision_interface.cc
  src/vision_kernel/vision_node.cc
  src/vision_kernel/vision_debug_node.cc
  src/vision_kernel/vision_flow.cc
  src/vision_kernel/vision_parser.cc
  include/vision_kernel/data.h
  include/vision_kernel/node_thread_safe_queue.h
  include/vision_kernel/vision_node_factory.h
  include/vision_kernel/input_manager.h
  include/vision_kernel/input.h
  include/vision_kernel/input_manager.h
  )
  
add_library(lib${PROJECT_NAME}Nodes SHARED
  src/vision_kernel/vision_nodes/input_image_node.cc

  src/vision_kernel/vision_nodes/grayscale_node.cc
  src/vision_kernel/vision_nodes/threshold_node.cc
  src/vision_kernel/vision_nodes/edge_detection_node.cc
  src/vision_kernel/vision_nodes/line_detection_node.cc
  src/vision_kernel/vision_nodes/parallel_lines_node.cc
  src/vision_kernel/vision_nodes/filtred_vertical_line_pairs_node.cc
  src/vision_kernel/vision_nodes/different_color_betwen_door_and_wall_node.cc
  src/vision_kernel/vision_nodes/door_gap_node.cc
  src/vision_kernel/vision_nodes/texture_bottom_of_door_node.cc

  src/vision_kernel/vision_debug_nodes/image_debug_node.cc
  
  src/vision_kernel/vision_nodes/output_object_detail_node.cc
  src/vision_kernel/vision_nodes/output_object_boundingbox_node.cc
  src/vision_kernel/vision_nodes/output_person_node.cc
  src/vision_kernel/vision_nodes/output_gesture_node.cc
  )
 
add_executable(${PROJECT_NAME}
  src/vision_kernel.cc
  )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(lib${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(lib${PROJECT_NAME}Nodes ${PROJECT_NAME}_gencpp)
  
target_link_libraries(lib${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
  
target_link_libraries(lib${PROJECT_NAME}Nodes
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
 
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  lib${PROJECT_NAME}
  lib${PROJECT_NAME}Nodes
  )
 
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
