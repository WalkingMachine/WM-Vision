cmake_minimum_required(VERSION 2.8.3)

project(wm_vision) 

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()
 
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp rospy message_generation cv_bridge pcl_ros rqt_gui rqt_gui_cpp image_transport sensor_msgs)
find_package(OpenCV 2.4 REQUIRED)
find_package(PCL 1.6 REQUIRED COMPONENTS)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_vision_tools_SRCS
  src/rqt_vision_tools/vision_tools.cpp
  src/rqt_vision_tools/ratio_layouted_frame.cpp
  src/rqt_vision_tools/dependencesqtablewidget.cpp
  src/rqt_vision_tools/dependencesvalue.cpp
  #src/rqt_vision_tools/main.cpp
  src/rqt_vision_tools/mainwindow.cpp
  src/rqt_vision_tools/visiontree.cpp
  src/rqt_vision_tools/visionnode.cpp
  src/rqt_vision_tools/vision_parser.cc
)

set(rqt_vision_tools_HDRS
  include/rqt_vision_tools/vision_tools.h
  include/rqt_vision_tools/ratio_layouted_frame.h
  include/rqt_vision_tools/dependencesqtablewidget.h
  include/rqt_vision_tools/dependencesvalue.h
  include/rqt_vision_tools/mainwindow.h
  include/rqt_vision_tools/visionnode.h
  include/rqt_vision_tools/visiontree.h
  include/rqt_vision_tools/vision_parser.h
)

set(rqt_vision_tools_UIS
  src/rqt_vision_tools/vision_tools.ui
  src/rqt_vision_tools/mainwindow.ui
  src/rqt_vision_tools/visionnode.ui
  src/rqt_vision_tools/dependencesvalue.ui
  src/rqt_vision_tools/visiontree.ui
)

set(rqt_vision_tools_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_python_setup()

add_service_files(
  FILES
  vision_interface_flow.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs roscpp rospy message_runtime rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge
   INCLUDE_DIRS ${rqt_vision_tools_INCLUDE_DIRECTORIES}
   LIBRARIES ${PROJECT_NAME})

qt4_wrap_cpp(rqt_vision_tools_MOCS ${rqt_vision_tools_HDRS})
qt4_wrap_ui(rqt_vision_tools_UIS_H ${rqt_vision_tools_UIS})

include_directories(cfvf include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${rqt_vision_tools_INCLUDE_DIRECTORIES})
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

add_library(lib${PROJECT_NAME}rqt ${rqt_vision_tools_SRCS} ${rqt_vision_tools_MOCS} ${rqt_vision_tools_UIS_H})
 
add_executable(${PROJECT_NAME}
  src/vision_kernel.cc
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
  lib${PROJECT_NAME}Nodes
  )

target_link_libraries(lib${PROJECT_NAME}rqt ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(lib${PROJECT_NAME}rqt)
 
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(PROGRAMS scripts/rqt_vision_tools
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
