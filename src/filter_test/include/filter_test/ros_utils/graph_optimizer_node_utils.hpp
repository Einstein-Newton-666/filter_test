#pragma once

#include "filter_test/graph_optimizer/armor_model.hpp"

#include <rclcpp/rclcpp.hpp>

namespace filter_test {

// graph_optimizer_test 和 filter_graph_optimizer 共用同一套参数声明/加载逻辑，
// 避免两个 ROS 节点的默认值或 yaml 字段逐渐分叉。
void declareTrackerParameters(rclcpp::Node& node);

graph_optimizer::TrackerConfig loadTrackerConfigFromParameters(
    rclcpp::Node& node);

}  // namespace filter_test
