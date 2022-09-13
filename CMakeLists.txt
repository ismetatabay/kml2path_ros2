cmake_minimum_required(VERSION 3.5)
project(kml2path_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(GeographicLib REQUIRED)


set (LibKML_INCLUDE_DIRS /usr/include/kml/)
set (LibKML_LIBRARIES /usr/lib/x86_64-linux-gnu/libkmlbase.so /usr/lib/x86_64-linux-gnu/libkmlconvenience.so /usr/lib/x86_64-linux-gnu/libkmldom.so /usr/lib/x86_64-linux-gnu/libkmlengine.so /usr/lib/x86_64-linux-gnu/libkmlregionator.so /usr/lib/x86_64-linux-gnu/libkmlxsd.so)


include_directories(
${LibKML_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS} )

link_directories(
${LibKML_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS} )

add_definitions(
${LibKML_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS} )

add_executable(kml2path src/kml2path.cpp)


target_link_libraries(kml2path
${LibKML_LIBRARIES} ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES})

ament_target_dependencies(kml2path rclcpp std_msgs nav_msgs geometry_msgs)


install(
  TARGETS kml2path
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
