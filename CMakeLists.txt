cmake_minimum_required(VERSION 3.8)
project(rviz2-panel)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies_pkgs
  rclcpp
  std_msgs
  rviz_common
  Qt5Core
  Qt5Gui
  Qt5Widgets
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# build
include_directories(include)
set(library_name push_button_rviz_panel)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/rviz2_panel.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz2_panel.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/push_button.ui
)

# Generate header for ui
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

add_library(${library_name} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS} 
)
ament_target_dependencies(${library_name} ${dependencies_pkgs})
target_link_libraries(${library_name} rviz_common::rviz_common)

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
target_compile_definitions(${library_name} PRIVATE QT_NO_KEYWORDS)

# install
pluginlib_export_plugin_description_file(rviz_common rviz2_plugin.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies_pkgs})

ament_package()
