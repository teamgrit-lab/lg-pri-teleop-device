cmake_minimum_required(VERSION 3.8)
project(lg_draco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Draco REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(nlohmann_json REQUIRED)

# 실행 파일 빌드 설정
add_executable(draco_sender src/draco_sender.cpp)
ament_target_dependencies(draco_sender rclcpp sensor_msgs tf2_ros tf2_sensor_msgs tf2_geometry_msgs geometry_msgs nlohmann_json)
target_include_directories(draco_sender PRIVATE ${Draco_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(draco_sender ${Draco_LIBRARIES} ${Boost_LIBRARIES} /usr/local/lib/libdraco.a)

# 1. 실행 파일 설치 (기존 코드)
install(TARGETS draco_sender DESTINATION lib/${PROJECT_NAME})

add_executable(tf_example src/tf_example.cpp)
ament_target_dependencies(tf_example rclcpp tf2_ros geometry_msgs)
install(TARGETS tf_example DESTINATION lib/${PROJECT_NAME})

# 2. Launch 폴더 설치 (추가된 부분)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 3. Config 폴더 설치 (추가된 부분)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
