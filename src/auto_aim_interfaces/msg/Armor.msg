string number                           # 装甲板编号
string type                             # 装甲板类型 "large" or "small"
string color                            # 装甲板颜色 "red" or "blue"
float32 area                            # 装甲板面积
float32 distance_to_image_center        # 装甲板到图像中心的距离
geometry_msgs/Pose pose                 # 装甲板位姿
int32 priority                          # 装甲板优先级
int32 camera_id                         # 相机ID
geometry_msgs/Point[6] detected_points  # 识别得装甲板图像坐标系下6个角点
float32 yaw
float32 optimized_yaw
