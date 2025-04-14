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
find_package(rosidl_default_generators REQUIRED)

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

# ament_auto_add_executable(filter src/filter_test.cpp src/filter.cpp)
# target_include_directories(filter PUBLIC 
# ${EIGEN3_INCLUDE_DIR} 
# ${OpenCV_INCLUDE_DIRS} 
# ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
# )
# target_link_libraries(filter 
# ${OpenCV_LIBS}
# ${PROJECT_NAME}__rosidl_typesupport_cpp
# )


ament_auto_add_executable(armor_simulation src/armor_simulation.cpp)
target_include_directories(armor_simulation PUBLIC 
${EIGEN3_INCLUDE_DIR} 
${OpenCV_INCLUDE_DIRS} 
${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(armor_simulation 
${OpenCV_LIBS}
${PROJECT_NAME}__rosidl_typesupport_cpp
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_auto_add_gtest(${PROJECT_NAME}_test 
#     test/filter_test.cpp 
#     src/filter.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_lint_auto_find_test_dependencies()
#   # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
