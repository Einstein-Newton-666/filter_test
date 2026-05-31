# 机器人中心真值 (与 CV 模型 11 维状态 [xc,vxc,yc,vyc,za,vza,yaw,vyaw,r1,r2,dz] 对齐)
std_msgs/Header header
float64 x       # xc  机器人中心 x
float64 y       # yc
float64 z       # za  机器人中心高度 (= z1)
float64 yaw     # yaw
float64 vx      # v_xc
float64 vy      # v_yc
float64 vz      # v_za (恒为0)
float64 vyaw    # v_yaw
float64 r1      # 偶数装甲板半径
float64 r2      # 奇数装甲板半径
float64 dz      # 高度差 z2-z1
