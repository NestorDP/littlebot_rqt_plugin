cmake_minimum_required(VERSION 3.20)
project(littlebot_rqt_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(littlebot_base REQUIRED)
find_package(realtime_tools REQUIRED)

# Qt5 + Charts (no QWT)
find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)

# Find libserial using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)

set(${PROJECT_NAME}_SRCS
  src/littlebot_rqt_plugin.cpp
  src/littlebot_gui.cpp
  src/littlebot_comm.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/littlebot_rqt_plugin.hpp
  include/${PROJECT_NAME}/littlebot_gui.hpp
  include/${PROJECT_NAME}/littlebot_comm.hpp
)

set(${PROJECT_NAME}_QT_UI_SOURCES
  ui/littlebot_gui.ui
)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HEADERS}
  ${${PROJECT_NAME}_QT_UI_SOURCES}
)

# littlebot_base uses C++23 features
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${LIBSERIAL_INCLUDE_DIRS}
  ${littlebot_base_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  PkgConfig::LIBSERIAL
  Qt5::Widgets
  Qt5::Charts
  littlebot_base::littlebot_driver
  littlebot_base::serial_port
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  pluginlib
  qt_gui_cpp
  rclcpp
  rqt_gui_cpp
  std_msgs
  realtime_tools
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
