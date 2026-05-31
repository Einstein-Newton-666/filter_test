#pragma once

/**
 * Auto Graph Optimizer - 统一头文件
 *
 * 包含此头文件即可使用整个自动因子图优化框架
 *
 * 使用示例 (2D 像素观测):
 *   #include "auto_graph_optimizer/auto_graph_optimizer.hpp"
 *
 *   auto layout = auto_graph::VariableLayout(11)
 *       .addDynamic("pos_vel",  {0,1,2,3,4,5})
 *       .addDynamic("yaw_vyaw", {6,7})
 *       .addStatic ("radius",   {8,9})
 *       .addStatic ("dz",       {10});
 *
 *   auto_graph::GraphOptimizer opt(config);
 *   opt.initialize(layout, x0, P0);
 *
 *   // 每帧:
 *   opt.advanceFrame(dt);
 *   opt.addMotionFactor<MyMotionModel>("pos_vel", model, Q_sub);
 *   opt.addCustomFactor<MyFactor>(...);
 *   opt.solve();
 *   VectorXd state = opt.getState();
 */

#include "graph_optimizer.hpp"

// 运动模型基类
#include "models/motion_model.hpp"

// 观测模型基类
#include "models/measure_model.hpp"

// 辅助函数
#include "utils/helpers.hpp"

// 基础类型定义
#include "utils/types.hpp"
