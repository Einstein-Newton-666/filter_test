cmake_minimum_required(VERSION 3.8)
project(armor_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GroundTruth.msg"
  DEPENDENCIES std_msgs
)

# 仿真器节点 (合并: 运动 + 相机投影 + 噪声 + PnP)
ament_auto_add_executable(armor_simulation_node src/armor_simulation.cpp)
target_include_directories(armor_simulation_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_get_typesupport_target(ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(armor_simulation_node
  ${OpenCV_LIBS}
  ${ts_target}
)

# 云台仿真节点
ament_auto_add_executable(gimbal_simulation src/gimbal_simulation.cpp)
target_include_directories(gimbal_simulation PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(gimbal_simulation ${ts_target})

# 角度解算器 placeholder
ament_auto_add_executable(angle_solver src/angle_solver.cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_add_gtest(test_noise_model
    test/test_noise_model.cpp
  )
  target_include_directories(test_noise_model PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_noise_model
    ${OpenCV_LIBS}
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  msg
)
