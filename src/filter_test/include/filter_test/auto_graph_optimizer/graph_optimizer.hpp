#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <functional>

namespace auto_graph {


/**
 * 变量布局 — 描述全状态向量如何拆分为 GTSAM 子变量
 *
 * 用法:
 *   auto layout = VariableLayout(11)
 *       .addDynamic("pos_vel", {0,1,2,3,4,5})
 *       .addDynamic("yaw",     {6,7})
 *       .addStatic ("radius",  {8,9})
 *       .addStatic ("dz",      {10});
 */
class VariableLayout {
public:
    struct Group {
        std::string name;
        std::vector<int> indices;   // 在全状态中的索引
        bool is_static;             // true=全局共享, false=每帧新建
        char key_prefix;            // GTSAM Symbol(prefix, index)
        int dim() const { return static_cast<int>(indices.size()); }
    };

    explicit VariableLayout(int full_dim) : full_dim_(full_dim) {}

    // ── 声明变量组 ──

    VariableLayout& addDynamic(const std::string& name, std::vector<int> indices) {
        groups_.push_back({name, std::move(indices), false, next_prefix_++});
        return *this;
    }

    VariableLayout& addStatic(const std::string& name, std::vector<int> indices) {
        groups_.push_back({name, std::move(indices), true, next_prefix_++});
        return *this;
    }

    // ── 查询 ──

    int fullDim() const { return full_dim_; }
    const std::vector<Group>& groups() const { return groups_; }

    const Group* findGroup(const std::string& name) const {
        for (auto& g : groups_)
            if (g.name == name) return &g;
        return nullptr;
    }

    /// 获取某个变量组在指定帧的 GTSAM Key
    gtsam::Key key(const std::string& name, uint64_t frame_id = 0) const {
        auto* g = findGroup(name);
        if (!g) throw std::runtime_error("VariableLayout::key: unknown group '" + name + "'");
        return gtsam::Symbol(g->key_prefix, g->is_static ? 0 : frame_id);
    }

    /// 获取所有动态变量组名
    std::vector<std::string> dynamicNames() const {
        std::vector<std::string> names;
        for (auto& g : groups_)
            if (!g.is_static) names.push_back(g.name);
        return names;
    }

    /// 获取所有变量组名
    std::vector<std::string> allNames() const {
        std::vector<std::string> names;
        for (auto& g : groups_) names.push_back(g.name);
        return names;
    }

    // ── Gather: 从 GTSAM Values 组装全状态向量 ──

    Eigen::VectorXd gather(const gtsam::Values& values, uint64_t frame_id) const {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(full_dim_);
        for (auto& g : groups_) {
            gtsam::Key k = gtsam::Symbol(g.key_prefix, g.is_static ? 0 : frame_id);
            if (!values.exists(k)) continue;  // 静默跳过 (冷启动阶段可能无 key)
            auto vec = values.at<gtsam::Vector>(k);
            for (int j = 0; j < g.dim(); ++j)
                x[g.indices[j]] = vec[j];
        }
        return x;
    }

    // ── 将全状态向量拆散到 Values ──

    void scatterInto(const Eigen::VectorXd& x, gtsam::Values& values, uint64_t frame_id) const {
        for (auto& g : groups_) {
            gtsam::Key k = gtsam::Symbol(g.key_prefix, g.is_static ? 0 : frame_id);
            gtsam::Vector v(g.dim());
            for (int j = 0; j < g.dim(); ++j)
                v[j] = x[g.indices[j]];
            values.insert(k, v);
        }
    }

    // ── Jacobian 块提取 ──

    /// 从全 Jacobian (行数任意, 列数=full_dim) 提取某个变量组的列块
    Eigen::MatrixXd jacBlock(const Eigen::MatrixXd& full_J, const std::string& name) const {
        auto* g = findGroup(name);
        if (!g) throw std::runtime_error("VariableLayout::jacBlock: unknown group '" + name + "'");
        Eigen::MatrixXd block(full_J.rows(), g->dim());
        for (int j = 0; j < g->dim(); ++j)
            block.col(j) = full_J.col(g->indices[j]);
        return block;
    }

private:
    int full_dim_;
    std::vector<Group> groups_;
    char next_prefix_ = 'a';
};


// ── 类型擦除的函数类型 ──
//    linearize(x_full) → (输出向量, Jacobian 矩阵)
using LinearizeFunc = std::function<std::pair<Eigen::VectorXd, Eigen::MatrixXd>(
    const Eigen::VectorXd&)>;

// ─────────────────────────────────────────────────────────────
// AutoMotionFactor — 2-key 运动因子 (组内)
//
//   x_prev_i  →  x_cur_i
//
// evaluateError 内部: 重建全状态 → autodiff → 提取该组的误差和 Jacobian
// ─────────────────────────────────────────────────────────────
class AutoMotionFactor : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
public:
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;

    /// @param k_prev   前一帧该组的 key
    /// @param k_cur    当前帧该组的 key
    /// @param noise    过程噪声 (子 Q 矩阵)
    /// @param linearize_func  全状态 autodiff: x → (x_pred, F)
    /// @param group_start     该组在全状态中的起始索引
    /// @param group_dim       该组维度
    /// @param x0_prev_full    线性化点的全状态 (用于填充非本组变量)
    AutoMotionFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                     const gtsam::SharedNoiseModel& noise,
                     LinearizeFunc linearize_func,
                     int group_start, int group_dim,
                     const Eigen::VectorXd& x0_prev_full)
        : Base(noise, k_prev, k_cur),
          linearize_func_(std::move(linearize_func)),
          group_start_(group_start),
          group_dim_(group_dim),
          x0_prev_full_(x0_prev_full) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x_prev_i,
        const gtsam::Vector& x_cur_i,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override
    {
        // 重建全状态: 用 x_prev_i 覆盖对应段, 其余保持线性化点值
        Eigen::VectorXd x_full = x0_prev_full_;
        x_full.segment(group_start_, group_dim_) = x_prev_i;

        // autodiff
        auto [x_pred, F] = linearize_func_(x_full);

        // 提取该组的预测值和 Jacobian 块
        Eigen::VectorXd x_pred_i = x_pred.segment(group_start_, group_dim_);
        Eigen::MatrixXd F_ii = F.block(group_start_, group_start_, group_dim_, group_dim_);

        if (H1) *H1 = F_ii;
        if (H2) *H2 = -gtsam::Matrix::Identity(group_dim_, group_dim_);

        return x_pred_i - x_cur_i;
    }

private:
    LinearizeFunc linearize_func_;
    int group_start_;
    int group_dim_;
    Eigen::VectorXd x0_prev_full_;   // 线性化点 (k-1 帧的全状态)
};

// ─────────────────────────────────────────────────────────────
// AutoMeasureFactor — N-key 观测因子
//
//   所有动态+静态变量  →  观测误差
//
// 内部: 从 Values 重建全状态 → autodiff → 按组分配 Jacobian 块
// ─────────────────────────────────────────────────────────────
class AutoMeasureFactor : public gtsam::NoiseModelFactor {
public:
    /// @param keys       所有子变量的 key 列表 (顺序需与 indices 对应)
    /// @param noise      观测噪声 R
    /// @param linearize_func  全状态 autodiff: x → (z_pred, H)
    /// @param layout     变量布局 (用于 gather 和 Jacobian 拆分)
    /// @param frame_id   当前帧 id (用于 gather)
    /// @param observation 观测值 z
    AutoMeasureFactor(const gtsam::KeyVector& keys,
                      const gtsam::SharedNoiseModel& noise,
                      LinearizeFunc linearize_func,
                      const VariableLayout& layout,
                      uint64_t frame_id,
                      const Eigen::VectorXd& observation)
        : gtsam::NoiseModelFactor(noise, keys),
          linearize_func_(std::move(linearize_func)),
          layout_(layout),
          frame_id_(frame_id),
          observation_(observation) {}

    /// Values 版本的 evaluateError — GTSAM 优化时调用
    gtsam::Vector unwhitenedError(
        const gtsam::Values& x,
        gtsam::OptionalMatrixVecType H = nullptr) const override
    {
        // 1. 从 Values gather 全状态
        Eigen::VectorXd x_full = layout_.gather(x, frame_id_);

        // 2. autodiff
        auto [z_pred, H_full] = linearize_func_(x_full);

        // 3. Jacobian 拆分
        if (H) {
            H->clear();
            for (auto& g : layout_.groups()) {
                Eigen::MatrixXd H_i = layout_.jacBlock(H_full, g.name);
                H->push_back(H_i);
            }
        }

        return z_pred - observation_;
    }

private:
    LinearizeFunc linearize_func_;
    const VariableLayout& layout_;   // 注意: 引用, 生命周期由 GraphOptimizer 保证
    uint64_t frame_id_;
    Eigen::VectorXd observation_;
};


enum class SmootherType { Incremental, Batch };

struct GraphOptimizerConfig {
    double relinearize_threshold = 0.01;
    int    relinearize_skip       = 1;
    int    cold_start_frames      = 5;   // 冷启动帧数, 前N帧只建图不优化
    int    max_window_frames      = 20;  // 周期性重置帧数, 0=不限制
    double smoother_lag           = 0.0; // 固定滞后帧数, 0=纯iSAM2无边际化, N=保留最近N帧
    SmootherType smoother_type    = SmootherType::Incremental;  // 平滑器类型
    int    extra_iterations       = 0;   // 每次update后额外iSAM2迭代 (GTSAM示例使用2-3次)
    bool   verbose                = false;
};

// ── SingleMotionFactor: 简化版 per-group 运动因子 ──
//   prev_group_vec → cur_group_vec
//   不涉及全状态重建, 直接对 group 子向量做 autodiff
class SingleMotionFactor : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
public:
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;
    using LinearizeFunc = std::function<std::pair<Eigen::VectorXd, Eigen::MatrixXd>(
        const Eigen::VectorXd&)>;

    SingleMotionFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                       const gtsam::SharedNoiseModel& noise,
                       LinearizeFunc f)
        : Base(noise, k_prev, k_cur), f_(std::move(f)) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x_prev, const gtsam::Vector& x_cur,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override
    {
        auto [x_pred, F] = f_(x_prev);
        if (H1) *H1 = F;
        if (H2) *H2 = -gtsam::Matrix::Identity(x_cur.size(), x_cur.size());
        return x_pred - x_cur;
    }

private:
    LinearizeFunc f_;
};

/**
 * GraphOptimizer V2 — iSAM2 + 变量拆分 + autodiff 自动因子
 *
 * 每帧流程:
 *   predict(dt, Q)    → 为每组动态变量添加 AutoMotionFactor (2-key)
 *   update(model, z, R) → 添加 AutoMeasureFactor (N-key, 所有变量)
 *   solve()           → iSAM2 增量优化 → gather 全状态
 *
 * 用法:
 *   auto layout = VariableLayout(11)
 *       .addDynamic("pos_vel", {0,1,2,3,4,5})
 *       .addDynamic("yaw",     {6,7})
 *       .addStatic ("radius",  {8,9})
 *       .addStatic ("dz",      {10});
 *
 *   GraphOptimizer opt(config);
 *   opt.initialize(layout, x0, P0);
 *
 *   // 每帧:
 *   opt.predict<MyMotionModel>(dt, Q);
 *   opt.update(measure_model, z, R);
 *   opt.solve();
 *   VectorXd state = opt.getState();
 */
class GraphOptimizer {
public:
    explicit GraphOptimizer(const GraphOptimizerConfig& cfg = GraphOptimizerConfig{})
        : config_(cfg) {}

    // ── 初始化 ──

    void initialize(const VariableLayout& layout,
                    const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0) {
        layout_ = layout;
        x_ = x0;
        frame_id_ = 0;
        smootherReset();

        // 将初态 scatter 到 Values
        layout_.scatterInto(x0, initial_values_, 0);

        // 为每组添加先验因子
        for (auto& g : layout_.groups()) {
            gtsam::Key k = layout_.key(g.name, g.is_static ? 0 : frame_id_);

            // 从 P0 的对角线提取该组的 sigma
            Eigen::VectorXd sigmas(g.dim());
            for (int j = 0; j < g.dim(); ++j) {
                double p_val = P0(g.indices[j], g.indices[j]);
                sigmas[j] = std::sqrt(std::max(p_val, 1e-8));
            }

            graph_.addPrior(k, gtsam::Vector(x0.segment(g.indices[0], g.dim())),
                            gtsam::noiseModel::Diagonal::Sigmas(sigmas));
        }

        initialized_ = true;
        cold_start_count_ = 0;
        // 注意: 不清空 graph_, prior 因子将在首次 isam2 update 时一并提交

        if (config_.verbose) {
            std::cout << "[GO] initialized, state_dim=" << layout_.fullDim()
                      << " groups=" << layout_.groups().size() << std::endl;
        }
    }

    // ── 预测: 添加运动因子 ──

    template<typename MotionModelType>
    void predict(double dt, const Eigen::MatrixXd& Q) {
        if (!initialized_) throw std::runtime_error("GraphOptimizer::predict: not initialized");

        MotionModelType model(dt);
        auto [x_pred, F] = model.linearize(x_);
        Eigen::VectorXd x0_prev = x_;  // 保存线性化点

        x_ = x_pred;   // 缓存预测值 (供 matchArmor 使用)
        uint64_t prev_id = frame_id_++;

        for (auto& g : layout_.groups()) {
            if (g.is_static) continue;  // 静态变量无运动因子

            gtsam::Key k_prev = layout_.key(g.name, prev_id);
            gtsam::Key k_cur  = layout_.key(g.name, frame_id_);

            // 注册初始值 (跳过已在 iSAM2 中的 key)
            if (!initial_values_.exists(k_cur)) {
                bool in_isam2 = hasSmoother() && smootherLinearizationPoint().exists(k_cur);
                if (!in_isam2) {
                    gtsam::Vector v(g.dim());
                    for (int j = 0; j < g.dim(); ++j)
                        v[j] = x_pred[g.indices[j]];
                    initial_values_.insert(k_cur, v);
                }
            }

            // 该组的子 Q 矩阵 (正则化确保正定, CWNA 离散化 det=0)
            Eigen::MatrixXd Q_sub(g.dim(), g.dim());
            for (int r = 0; r < g.dim(); ++r)
                for (int c = 0; c < g.dim(); ++c)
                    Q_sub(r, c) = Q(g.indices[r], g.indices[c]);
            // CWNA 离散化产生半正定 Q (det=0), GTSAM 需要正定
            Q_sub.diagonal() += Eigen::VectorXd::Constant(g.dim(), 1e-8);

            // 创建运动因子
            auto linearize_func = [model](const Eigen::VectorXd& x) mutable {
                return model.linearize(x);
            };

            graph_.emplace_shared<AutoMotionFactor>(
                k_prev, k_cur,
                gtsam::noiseModel::Gaussian::Covariance(Q_sub),
                linearize_func,
                static_cast<int>(g.indices[0]), g.dim(),
                x0_prev);
        }
    }

    // ── 分步运动因子 (per-group, 配合 advanceFrame 使用) ──

    /// 推进帧号
    void advanceFrame(double /*dt*/) {
        if (!initialized_) throw std::runtime_error("GraphOptimizer::advanceFrame: not initialized");
        frame_id_++;
    }

    /// 为单个 group 添加运动因子 (per-group model)
    /// @param group_name 变量组名
    /// @param model      运动模型 (只对该组维度操作)
    /// @param Q_sub      该组的过程噪声矩阵 (group_dim × group_dim)
    template<typename MotionModelType>
    void addMotionFactor(const std::string& group_name,
                         MotionModelType model,
                         const Eigen::MatrixXd& Q_sub) {
        if (!initialized_) throw std::runtime_error("GraphOptimizer::addMotionFactor: not initialized");

        auto* g = layout_.findGroup(group_name);
        if (!g || g->is_static) return;

        uint64_t prev_id = frame_id_ - 1;
        gtsam::Key k_prev = layout_.key(group_name, prev_id);
        gtsam::Key k_cur  = layout_.key(group_name, frame_id_);

        if (!initial_values_.exists(k_cur)) {
            bool in_isam2 = hasSmoother() && smootherLinearizationPoint().exists(k_cur);
            if (!in_isam2) {
                gtsam::Vector v(g->dim());
                for (int j = 0; j < g->dim(); ++j)
                    v[j] = x_[g->indices[j]];
                initial_values_.insert(k_cur, v);
            }
        }

        // 缓存当前帧预测值到 x_
        Eigen::VectorXd x_sub(g->dim());
        for (int j = 0; j < g->dim(); ++j)
            x_sub[j] = x_[g->indices[j]];
        auto [x_pred_sub, F_sub] = model.linearize(x_sub);
        for (int j = 0; j < g->dim(); ++j)
            x_[g->indices[j]] = x_pred_sub[j];

        auto linearize_func = [model](const Eigen::VectorXd& x) mutable {
            return model.linearize(x);
        };

        // Q 正则化
        Eigen::MatrixXd Q_reg = Q_sub;
        Q_reg.diagonal() += Eigen::VectorXd::Constant(g->dim(), 1e-8);

        graph_.emplace_shared<SingleMotionFactor>(
            k_prev, k_cur,
            gtsam::noiseModel::Gaussian::Covariance(Q_reg),
            linearize_func);
    }

    // ── 观测: 添加观测因子 (autodiff 路径) ──

    template<typename MeasureModelType>
    void update(const MeasureModelType& model,
                const Eigen::VectorXd& z,
                const Eigen::MatrixXd& R) {
        if (!initialized_) throw std::runtime_error("GraphOptimizer::update: not initialized");

        // 构建 key 列表 (所有动态+静态变量)
        gtsam::KeyVector keys;
        for (auto& g : layout_.groups())
            keys.push_back(layout_.key(g.name, g.is_static ? 0 : frame_id_));

        // 注册初始值 (跳过已在 iSAM2 中的 key)
        for (auto& g : layout_.groups()) {
            gtsam::Key k = layout_.key(g.name, g.is_static ? 0 : frame_id_);
            bool in_isam2 = hasSmoother() && smootherLinearizationPoint().exists(k);
            if (!initial_values_.exists(k) && !in_isam2) {
                gtsam::Vector v(g.dim());
                for (int j = 0; j < g.dim(); ++j)
                    v[j] = x_[g.indices[j]];
                initial_values_.insert(k, v);
            }
        }

        auto linearize_func = [model](const Eigen::VectorXd& x) mutable {
            return model.linearize(x);
        };

        graph_.emplace_shared<AutoMeasureFactor>(
            keys,
            gtsam::noiseModel::Gaussian::Covariance(R),
            linearize_func,
            layout_,
            frame_id_,
            z);
    }

    /// 便捷重载: 用默认构造的 MeasureModel
    template<typename MeasureModelType>
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) {
        update(MeasureModelType(0), z, R);
    }

    // ── 自定义因子 (手写路径) ──

    template<typename FactorType, typename... Args>
    void addCustomFactor(Args&&... args) {
        graph_.emplace_shared<FactorType>(std::forward<Args>(args)...);
    }

    /// 添加 Pose3 先验因子
    void addPose3Prior(gtsam::Key key, const gtsam::Pose3& pose,
                       const gtsam::SharedNoiseModel& noise) {
        graph_.addPrior(key, pose, noise);
    }

    /// 查询变量 key (供自定义因子使用)
    gtsam::Key key(const std::string& name, uint64_t frame_id = 0) const {
        return layout_.key(name, frame_id);
    }

    /// 创建 armor Pose3 变量 key (per-frame, per-armor)
    gtsam::Key armorPoseKey(uint64_t frame_id, int armor_idx) const {
        return gtsam::Symbol('h' + armor_idx, frame_id);
    }

    /// 插入 armor Pose3 初值 (跳过已在 iSAM2 中的 key)
    void insertArmorPose(gtsam::Key k, const gtsam::Pose3& pose) {
        bool in_isam2 = hasSmoother() && smootherLinearizationPoint().exists(k);
        if (!initial_values_.exists(k) && !in_isam2)
            initial_values_.insert(k, pose);
    }

    /// 获取所有中心状态变量的 keys (供自定义因子使用)
    gtsam::KeyVector centerKeys() const {
        gtsam::KeyVector keys;
        for (auto& g : layout_.groups())
            keys.push_back(layout_.key(g.name, g.is_static ? 0 : frame_id_));
        return keys;
    }

    // ── 优化 ──

    void solve() {
        if (!initialized_ || graph_.empty()) return;

        cold_start_count_++;
        frames_since_reset_++;

        // 冷启动: 前 N 帧只累积因子和初值, 不优化 (避免约束不足时发散)
        if (config_.cold_start_frames > 0 && cold_start_count_ <= config_.cold_start_frames) {
            if (config_.verbose) {
                std::cout << "[GO] cold start " << cold_start_count_
                          << "/" << config_.cold_start_frames << std::endl;
            }
            // 不清空 graph_ 和 initial_values_, 让它们跨帧累积
            // 注意: 静态变量 key 不变, check exists 防止重复插入; 动态变量 key 每帧递增, 无冲突
            return;
        }

        // 冷启动结束标记: 清空计数器, 后续不再进入此分支
        if (cold_start_count_ == config_.cold_start_frames + 1 && config_.verbose) {
            std::cout << "[GO] cold start done, submitting " << graph_.size()
                      << " factors" << std::endl;
        }

        // ── 滑动窗口边际化 (旧方案, 仅 smoother_lag=0 时使用) ──
        if (config_.smoother_lag <= 0 && config_.max_window_frames > 0 &&
            frames_since_reset_ >= config_.max_window_frames) {
            slideWindow();
        }

        try {
            if (!hasSmoother()) {
                if (config_.smoother_lag <= 0) {
                    // 纯 ISAM2 (对齐 jlu, 无边际化)
                    gtsam::ISAM2Params p;
                    p.relinearizeThreshold = config_.relinearize_threshold;
                    p.relinearizeSkip = config_.relinearize_skip;
                    p.enablePartialRelinearizationCheck = true;
                    p.findUnusedFactorSlots = true;
                    isam2_ = std::make_unique<gtsam::ISAM2>(p);
                } else if (config_.smoother_type == SmootherType::Incremental) {
                    gtsam::ISAM2Params p;
                    p.relinearizeThreshold = config_.relinearize_threshold;
                    p.relinearizeSkip = config_.relinearize_skip;
                    p.enablePartialRelinearizationCheck = true;
                    p.findUnusedFactorSlots = true;
                    inc_smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
                        config_.smoother_lag, p);
                } else {
                    gtsam::LevenbergMarquardtParams lm;
                    lm.setMaxIterations(10);
                    lm.setRelativeErrorTol(1e-5);
                    lm.setAbsoluteErrorTol(1e-5);
                    batch_smoother_ = std::make_unique<gtsam::BatchFixedLagSmoother>(
                        config_.smoother_lag, lm);
                }
            }

            if (config_.smoother_lag <= 0) {
                // ── 纯 ISAM2 路径 (对齐 jlu_tracker) ──
                isam2_->update(graph_, initial_values_);
                // 额外迭代收敛
                for (int i = 1; i < config_.extra_iterations; ++i) {
                    isam2_->update();
                }
                auto est = isam2_->calculateEstimate();
                x_ = layout_.gather(est, frame_id_);
            } else {
                // ── FixedLagSmoother 路径 (边际化) ──
                gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
                for (const auto& [key, value] : initial_values_) {
                    timestamps[key] = static_cast<double>(frame_id_);
                }
                for (auto& g : layout_.groups()) {
                    if (g.is_static) {
                        timestamps[layout_.key(g.name, 0)] =
                            static_cast<double>(frame_id_);
                    }
                }
                smoother().update(graph_, initial_values_, timestamps);
                for (int i = 1; i < config_.extra_iterations; ++i) {
                    smoother().update();
                }
                auto est = smoother().calculateEstimate();
                x_ = layout_.gather(est, frame_id_);
            }
            graph_.resize(0);
            initial_values_.clear();

            if (config_.verbose) {
                std::cout << "[GO] solved f=" << frame_id_
                          << " x=[" << x_[0] << "," << x_[2] << "," << x_[4]
                          << "] yaw=" << x_[6] << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[GO] iSAM2 error f=" << frame_id_ << ": " << e.what() << std::endl;
            graph_.resize(0);
            initial_values_.clear();  // 同步清空, 避免下次重复 key
        }
    }

    // ── 查询 ──

    Eigen::VectorXd getState() const { return x_; }
    uint64_t getFrameId() const { return frame_id_; }

    /// 获取边际协方差 (从 iSAM2)
    Eigen::MatrixXd getCovariance() const {
        if (!hasSmoother()) return Eigen::MatrixXd::Identity(layout_.fullDim(), layout_.fullDim());
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(layout_.fullDim(), layout_.fullDim());
        try {
            for (auto& g : layout_.groups()) {
                gtsam::Key k = layout_.key(g.name, g.is_static ? 0 : frame_id_);
                auto cov = smootherMarginalCovariance(k);
                for (int r = 0; r < g.dim(); ++r)
                    for (int c = 0; c < g.dim(); ++c)
                        P(g.indices[r], g.indices[c]) = cov(r, c);
            }
        } catch (...) {}
        return P;
    }

    void reset() {
        graph_.resize(0);
        initial_values_.clear();
        smootherReset();
        x_.setZero();
        frame_id_ = 0;
        initialized_ = false;
        cold_start_count_ = 0;
        frames_since_reset_ = 0;
    }

private:
    void slideWindow() {
        Eigen::VectorXd x_saved = x_;
        Eigen::MatrixXd P_saved = getCovariance();
        uint64_t saved_frame = frame_id_;

        smootherReset();
        initial_values_.clear();
        graph_.resize(0);
        frames_since_reset_ = 0;

        // 重置 frame_id, 用当前估计做先验
        frame_id_ = 0;
        for (auto& g : layout_.groups()) {
            gtsam::Key k = layout_.key(g.name, g.is_static ? 0 : frame_id_);
            Eigen::VectorXd sigmas(g.dim());
            for (int j = 0; j < g.dim(); ++j) {
                sigmas[j] = std::sqrt(std::max(P_saved(g.indices[j], g.indices[j]), 1e-8));
            }
            graph_.addPrior(k, gtsam::Vector(x_saved.segment(g.indices[0], g.dim())),
                            gtsam::noiseModel::Diagonal::Sigmas(sigmas));
        }
        layout_.scatterInto(x_saved, initial_values_, 0);
        if (config_.verbose)
            std::cout << "[GO] window reset, saved f=" << saved_frame
                      << " x=[" << x_saved[0] << "," << x_saved[2] << "]"
                      << " yaw=" << x_saved[6] << std::endl;
    }

    GraphOptimizerConfig config_;
    VariableLayout layout_{0};
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    std::unique_ptr<gtsam::ISAM2> isam2_;                         // smoother_lag=0 时使用
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> inc_smoother_;
    std::unique_ptr<gtsam::BatchFixedLagSmoother> batch_smoother_;
    Eigen::VectorXd x_;
    uint64_t frame_id_ = 0;
    bool initialized_ = false;
    int cold_start_count_ = 0;
    int frames_since_reset_ = 0;

    // ── 平滑器辅助方法 ──

    bool hasSmoother() const { return isam2_ || inc_smoother_ || batch_smoother_; }

    bool useFixedLag() const { return config_.smoother_lag > 0; }

    gtsam::FixedLagSmoother& smoother() {
        if (inc_smoother_) return *inc_smoother_;
        return *batch_smoother_;
    }

    const gtsam::Values& smootherLinearizationPoint() const {
        if (isam2_) return isam2_->getLinearizationPoint();
        if (inc_smoother_) return inc_smoother_->getLinearizationPoint();
        return batch_smoother_->getLinearizationPoint();
    }

    gtsam::Matrix smootherMarginalCovariance(gtsam::Key key) const {
        if (isam2_) return isam2_->marginalCovariance(key);
        if (inc_smoother_) return inc_smoother_->marginalCovariance(key);
        return batch_smoother_->marginalCovariance(key);
    }

    void smootherReset() {
        isam2_.reset();
        inc_smoother_.reset();
        batch_smoother_.reset();
    }
};

} // namespace auto_graph
