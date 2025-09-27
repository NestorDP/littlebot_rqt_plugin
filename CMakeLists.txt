cmake_minimum_required(VERSION 3.8)
project(littlebot_rqt_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(qt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)


set(${PROJECT_NAME}_SRCS
  src/littlebot_rqt_plugin.cpp
  src/littlebot_gui.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/littlebot_rqt_plugin.hpp
  include/${PROJECT_NAME}/littlebot_gui.hpp
)

set(${PROJECT_NAME}_QT_UI_SOURCES
  ui/littlebot_gui.ui
)

set(${PROJECT_NAME}_INCLUDE_DIR
  include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(${PROJECT_NAME}_LIB_MOC ${${PROJECT_NAME}_HEADERS})
qt5_wrap_ui(${PROJECT_NAME}_UI_MOC ${${PROJECT_NAME}_QT_UI_SOURCES})

set(QWT_INCLUDE_DIRS /usr/include/qwt)
set(QWT_LIBRARIES /usr/lib/libqwt-qt5.so)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${QWT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_LIB_MOC}
  ${${PROJECT_NAME}_UI_MOC}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${${PROJECT_NAME}_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${QWT_LIBRARIES}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  pluginlib
  qt_gui_cpp
  Qt5Widgets
  rclcpp
  rqt_gui_cpp
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ui launch scripts
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()