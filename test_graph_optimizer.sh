#!/bin/bash

# 图优化测试脚本
# 测试两种模式的收敛性

export LD_LIBRARY_PATH=/home/gaoyuan/filter_test/install/auto_aim_interfaces/lib:/home/gaoyuan/filter_test/install/filter_test/lib:/home/gaoyuan/filter_test/install/armor_simulation/lib:/usr/local/lib:$LD_LIBRARY_PATH

echo "=========================================="
echo "图优化框架测试"
echo "=========================================="
echo ""

# 测试1：YPD观测模式
echo "=== 测试1：YPD观测模式 ==="
echo "启动仿真器..."
/home/gaoyuan/filter_test/install/armor_simulation/lib/armor_simulation/armor_simulation_node &
SIM_PID=$!
sleep 2

echo "启动图优化器（YPD模式）..."
/home/gaoyuan/filter_test/install/filter_test/lib/filter_test/graph_optimizer_test --ros-args -p use_2d_observation:=false &
GO_PID=$!

echo "运行20秒..."
sleep 20

kill $SIM_PID $GO_PID 2>/dev/null
wait $SIM_PID $GO_PID 2>/dev/null
echo "YPD模式测试完成"
echo ""

sleep 2

# 测试2：像素坐标观测模式
echo "=== 测试2：像素坐标观测模式 ==="
echo "启动仿真器..."
/home/gaoyuan/filter_test/install/armor_simulation/lib/armor_simulation/armor_simulation_node &
SIM_PID=$!
sleep 2

echo "启动图优化器（2D模式）..."
/home/gaoyuan/filter_test/install/filter_test/lib/filter_test/graph_optimizer_test --ros-args -p use_2d_observation:=true &
GO_PID=$!

echo "运行20秒..."
sleep 20

kill $SIM_PID $GO_PID 2>/dev/null
wait $SIM_PID $GO_PID 2>/dev/null
echo "像素坐标模式测试完成"
echo ""

echo "=========================================="
echo "所有测试完成！"
echo "=========================================="
