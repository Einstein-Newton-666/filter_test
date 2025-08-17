cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Anglesolver.msg"
  "msg/TargetPose.msg"
  "msg/Vehicle.msg"
  "msg/Vehicles.msg"
  "msg/CameraMsg.msg"

  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/RecieveData.msg"
  "msg/SendData.msg"

  "msg/ArmorInfo.msg"
  "msg/EnemyInfo.msg"
  "msg/TrackerInfo.msg"
  "msg/TrackerPredict.msg"
  "msg/TrackerTarget.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_auto_package()
