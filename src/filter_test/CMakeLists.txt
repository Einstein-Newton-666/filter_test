cmake_minimum_required(VERSION 3.8)
project(filter_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# GTSAM
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Simulation.msg"
  "msg/Result.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    auto_aim_interfaces
)

ament_auto_add_executable(filter src/filter_test.cpp src/filter.cpp)
target_include_directories(filter PUBLIC 
${EIGEN3_INCLUDE_DIR} 
${OpenCV_INCLUDE_DIRS} 
${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(filter 
${OpenCV_LIBS}
${PROJECT_NAME}__rosidl_typesupport_cpp
)


# 图优化测试节点
ament_auto_add_executable(graph_optimizer_test src/graph_optimizer_test.cpp)
target_include_directories(graph_optimizer_test PUBLIC
${EIGEN3_INCLUDE_DIR}
${GTSAM_INCLUDE_DIR}
${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(graph_optimizer_test
${PROJECT_NAME}__rosidl_typesupport_cpp
gtsam
TBB::tbb
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/filter_test.cpp
    src/filter.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_lint_auto_find_test_dependencies()
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  ament_auto_add_gtest(test_auto_graph_optimizer
    test/test_auto_graph_optimizer.cpp
  )
  target_include_directories(test_auto_graph_optimizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
