cmake_minimum_required(VERSION 2.8.3)

project(wm_vision) 

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()
 
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp rospy message_generation cv_bridge)
find_package(OpenCV 2.4 REQUIRED)

add_service_files(
  FILES
  vision_interface_tree.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs roscpp rospy message_runtime
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME})
 
include_directories(cfvt include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

add_library(lib${PROJECT_NAME} SHARED
  src/wm_visionKernel/vision_node.cc
  src/wm_visionKernel/vision_debug_node.cc
  src/wm_visionKernel/vision_tree.cc
  src/wm_visionKernel/vision_parser.cc
  include/wm_visionKernel/data.h
  include/wm_visionKernel/thread_safe_queue.h
  include/wm_visionKernel/vision_node_factory.h
  include/wm_visionKernel/input_manager.h
  include/wm_visionKernel/input.h
  include/wm_visionKernel/input_manager.h
  )
  
add_library(lib${PROJECT_NAME}Nodes SHARED
  src/wm_visionKernel/visionNodes/grayscale_node.cc
  src/wm_visionKernel/visionNodes/openni_node.cc
  src/wm_visionKernel/visionNodes/threshold_node.cc
  src/wm_visionKernel/visionNodes/output_object_detail_node.cc
  src/wm_visionKernel/visionNodes/output_object_boundingbox_node.cc
  src/wm_visionKernel/visionNodes/output_person_node.cc
  src/wm_visionKernel/visionNodes/output_gesture_node.cc
  src/wm_visionKernel/vision_debug_nodes/image_debug_node.cc
  )
 
add_executable(${PROJECT_NAME}
  src/vision_interface.cc
  )
  
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
  lib${PROJECT_NAME}Nodes)
 
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
