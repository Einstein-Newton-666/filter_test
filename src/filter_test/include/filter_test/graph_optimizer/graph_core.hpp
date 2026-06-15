#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace auto_graph {

// Var<T> 是模型声明阶段返回的强类型变量句柄。模型后续通过句柄取 key、
// 插入初值、添加 prior 和读取 estimate，避免到处重复写字符串和模板类型。
template<typename T>
struct Var {
    std::string name;
};

enum class SmootherType { Incremental, Batch };

// GraphOptimizerConfig 只包含求解器行为参数，模型噪声和观测噪声放在具体模型配置中。
struct GraphOptimizerConfig {
    double relinearize_threshold = 0.01;
    int relinearize_skip = 1;
    int cold_start_frames = 5;
    double smoother_lag = 0.0;
    SmootherType smoother_type = SmootherType::Incremental;
    int update_iterations = 1;  // Total update() calls per solve; <=1 means single update.
    bool verbose = false;
};

// solve() 的结构化返回值。tracker 根据这些标志决定是否发布估计、是否保留上一帧状态。
struct SolveResult {
    uint64_t frame_id = 0;
    bool attempted = false;
    bool optimized = false;
    bool cold_start = false;
    bool failed = false;
    std::string error_message;
};

// GraphOptimizer 是 typed GTSAM 图的单一核心类：声明变量、映射 key、写入
// Values/Factors、调用 iSAM2/fixed-lag smoother，并读取最新 estimate。
// 它不包含装甲板、相机、匹配或 ROS 语义。
class GraphOptimizer {
public:
    explicit GraphOptimizer(const GraphOptimizerConfig& cfg = GraphOptimizerConfig{});

    template<typename T>
    Var<T> declareDynamic(const std::string& name, char prefix) {
        add<T>(name, prefix, false);
        return Var<T>{name};
    }

    template<typename T>
    Var<T> declareStatic(const std::string& name, char prefix) {
        add<T>(name, prefix, true);
        return Var<T>{name};
    }

    template<typename T>
    gtsam::Key key(const Var<T>& var) const {
        checkType<T>(var.name);
        return key(var.name, frame_id_);
    }

    template<typename T>
    gtsam::Key key(const Var<T>& var, uint64_t frame_id) const {
        checkType<T>(var.name);
        return key(var.name, frame_id);
    }

    template<typename T>
    gtsam::Key keyPrev(const Var<T>& var) const {
        return key(var, frame_id_ - 1);
    }

    void beginInit();
    void finishInit();

    uint64_t beginFrame();
    SolveResult solve();
    uint64_t getFrameId() const;

    template<typename T>
    void insert(const Var<T>& var, const T& value) {
        insertAux(key(var), value);
    }

    // upsert 只作用于本轮 solve 前的初值缓存。它允许模型先按运动模型插入
    // 当前帧初值，再用外部前端估计覆盖该初值；已经进入 latest_estimate_ 的
    // 历史 key 不会被改写。
    template<typename T>
    void upsert(const Var<T>& var, const T& value) {
        upsertAux(key(var), value);
    }

    // insertAux 用于模型自己生成的辅助 key (未注册变量)，如每帧的装甲板 Pose3。
    // 辅助 key 不在变量表中，所以这里只做重复插入保护。
    template<typename T>
    void insertAux(gtsam::Key key, const T& value) {
        if (latest_estimate_.exists(key)) return;
        if (!initial_values_.exists(key)) {
            initial_values_.insert(key, value);
        }
    }

    template<typename T>
    void upsertAux(gtsam::Key key, const T& value) {
        if (latest_estimate_.exists(key)) return;
        if (initial_values_.exists(key)) {
            initial_values_.update(key, value);
        } else {
            initial_values_.insert(key, value);
        }
    }

    template<typename T>
    void addPrior(const Var<T>& var,
                  const T& value,
                  const gtsam::SharedNoiseModel& noise) {
        insert(var, value);
        graph_.addPrior(key(var), value, noise);
    }

    template<typename T>
    void addAuxPrior(gtsam::Key key,
                     const T& value,
                     const gtsam::SharedNoiseModel& noise) {
        graph_.addPrior(key, value, noise);
    }

    template<typename FactorType, typename... Args>
    void addFactor(Args&&... args) {
        graph_.emplace_shared<FactorType>(std::forward<Args>(args)...);
    }

    template<typename T>
    T estimate(const Var<T>& var) const {
        return estimate<T>(key(var));
    }

    template<typename T>
    T estimate(const Var<T>& var, uint64_t frame_id) const {
        return estimate<T>(key(var, frame_id));
    }

    // 优先读最新优化结果；冷启动或本帧尚未优化时回退到待提交初值。
    template<typename T>
    T estimate(gtsam::Key key) const {
        if (latest_estimate_.exists(key)) {
            return latest_estimate_.at<T>(key);
        }
        if (initial_values_.exists(key)) {
            return initial_values_.at<T>(key);
        }
        throw std::runtime_error("GraphOptimizer::estimate: key not found");
    }

    void reset();

private:
    struct Entry {
        char prefix = 0;
        bool is_static = false;
        std::type_index type = typeid(void);
    };

    template<typename T>
    void add(const std::string& name, char prefix, bool is_static) {
        if (entries_.find(name) != entries_.end()) {
            throw std::runtime_error("GraphOptimizer: duplicate variable '" + name + "'");
        }
        for (const auto& [existing_name, entry] : entries_) {
            (void)existing_name;
            if (entry.prefix == prefix) {
                throw std::runtime_error("GraphOptimizer: duplicate key prefix");
            }
        }
        entries_.emplace(name, Entry{prefix, is_static, std::type_index(typeid(T))});
    }

    const Entry& at(const std::string& name) const;
    gtsam::Key key(const std::string& name, uint64_t frame_id) const;
    std::vector<gtsam::Key> staticKeys() const;

    template<typename T>
    void checkType(const std::string& name) const {
        const auto& entry = at(name);
        if (entry.type != std::type_index(typeid(T))) {
            throw std::runtime_error(
                "GraphOptimizer: variable '" + name + "' requested with wrong type");
        }
    }

    void ensureSmoother();
    gtsam::FixedLagSmoother& smoother();
    void smootherReset();

    GraphOptimizerConfig config_;
    std::unordered_map<std::string, Entry> entries_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    gtsam::Values latest_estimate_;
    std::unique_ptr<gtsam::ISAM2> isam2_;
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> inc_smoother_;
    std::unique_ptr<gtsam::BatchFixedLagSmoother> batch_smoother_;
    uint64_t frame_id_ = 0;
    bool initialized_ = false;
    int cold_start_count_ = 0;
};

}  // namespace auto_graph
