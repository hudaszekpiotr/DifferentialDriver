cmake_minimum_required(VERSION 3.8)
project(image_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(main src/image_detector_cpp.cpp)
ament_target_dependencies(main rclcpp std_msgs sensor_msgs visualization_msgs cv_bridge image_transport OpenCV)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(train_face_recognition src/train_face_recognition_cpp.cpp)
ament_target_dependencies(train_face_recognition rclcpp std_msgs std_srvs sensor_msgs visualization_msgs cv_bridge image_transport OpenCV interfaces)

target_include_directories(train_face_recognition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(train_face_recognition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS main train_face_recognition
  DESTINATION lib/${PROJECT_NAME})

add_executable(generate_marker src/generate_marker.cpp)
ament_target_dependencies(generate_marker OpenCV)
target_compile_features(generate_marker PUBLIC c_std_99 cxx_std_17)
install(TARGETS generate_marker
  DESTINATION .)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
