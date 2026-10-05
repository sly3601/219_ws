// 只读 Gazebo 诊断插件；独立于控制器，不写任何关节/机器人状态。
#include "sysu219_guide_controller/debug/DebugConfig.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <thread>
#include <unistd.h>
#include <vector>

namespace gazebo {
class GazeboPhysicsDebug : public ModelPlugin {
    using V3 = ignition::math::Vector3d;
    struct Frame {
        double sim_s = 0, dt_s = 0, est_s = 0, sample_us = 0, mass = 0, contact_s = -1;
        int state = -1, contact_count = 0, contact_available = 0;
        std::array<double, 12> q{}, qd{};
        V3 body_p, body_v, body_rpy, body_w, com_p, com_v;
        std::array<V3, 4> foot_p{}, foot_v{}, contact_force{};
        std::array<int, 4> foot_contacts{};
    };
public:
    ~GazeboPhysicsDebug() override {
        connection_.reset();
        subscription_.reset();
        contact_sub_.reset();
        // 仿真关闭时导出未录满的起步窗口；此时更新回调已停止。
        if (capturing_ && !ready_.load(std::memory_order_acquire)) publish();
        stop_.store(true);
        if (worker_.joinable()) worker_.join();
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
        if (!quadruped_debug::kLargeDebug) return; // 关闭时无线程/缓冲/订阅。
        model_ = model;
        world_ = model->GetWorld();
        node_ = gazebo_ros::Node::Get(sdf);
        body_ = model->GetLink("base");
        if (!body_) body_ = model->GetLink("trunk");
        const std::array<std::string, 4> legs{{"FR", "FL", "RR", "RL"}};
        const std::array<std::string, 3> joints{{"hip", "thigh", "calf"}};
        for (size_t leg = 0; leg < 4; ++leg) {
            for (size_t joint = 0; joint < 3; ++joint)
                joints_[leg * 3 + joint] = model->GetJoint(legs[leg] + "_" + joints[joint] + "_joint");
            // URDF 固定关节可能被 Gazebo 合并。取脚球碰撞体，避免改 URDF 物理模型。
            for (const auto& link : model->GetLinks())
                for (const auto& collision : link->GetCollisions())
                    if (link->GetName() == legs[leg] + "_foot" ||
                        collision->GetName().find(legs[leg] + "_foot") != std::string::npos)
                        feet_[leg] = collision;
        }
        if (!body_ || std::any_of(joints_.begin(), joints_.end(), [](const auto& j) { return !j; }) ||
            std::any_of(feet_.begin(), feet_.end(), [](const auto& f) { return !f; })) {
            RCLCPP_ERROR(node_->get_logger(), "[PHYSICS_DEBUG] missing body/joint/foot collision; diagnostic disabled");
            return;
        }
        for (const auto& link : model->GetLinks()) {
            if (!link->GetInertial()) continue;
            const double mass = link->GetInertial()->Mass();
            if (mass > 0) massive_links_.push_back({link, mass});
        }
        const double step = world_->Physics()->GetMaxStepSize();
        if (!std::isfinite(step) || step <= 0) return;
        nominal_step_ = step;
        // 前 2 秒 + 起步后 10 秒。容量上限防止极小物理步长耗尽内存。
        const size_t capacity = static_cast<size_t>(std::min(50000.0, std::ceil(12.0 / step) + 2.0));
        ring_.resize(capacity);
        contact_node_.reset(new transport::Node());
        contact_node_->Init(world_->Name());
        // 只订阅：使 Gazebo 提供实际接触信息，不设置物理参数、不改变接触判定。
        contact_sub_ = contact_node_->Subscribe("~/physics/contacts", &GazeboPhysicsDebug::contacts, this);
        subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/estimator_debug", rclcpp::QoS(10).best_effort(),
            [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() != 28 || !std::isfinite(msg->data[0]) ||
                    !std::isfinite(msg->data[1])) return;
                est_s_.store(msg->data[0]);
                state_.store(static_cast<int>(msg->data[1]));
            });
        worker_ = std::thread([this] { writeLoop(); });
        connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboPhysicsDebug::sample, this));
        RCLCPP_INFO(node_->get_logger(),
            "[PHYSICS_DEBUG] enabled engine=%s step_ms=%.3f capacity=%zu sample=physics_end; records stand-before/trot-10s",
            world_->Physics()->GetType().c_str(), step * 1000.0, capacity);
    }

private:
    void contacts(ConstContactsPtr&) {} // 不在 Gazebo transport 线程计算或写盘。

    void sample() {
        if (ready_.load(std::memory_order_acquire)) return; // 后台导出时冻结。
        const auto begin = std::chrono::steady_clock::now();
        const double now = world_->SimTime().Double();
        const int state = state_.load();
        if (state < 0) return; // 必须收到开启诊断的控制器消息才记录。
        if (now < last_s_) { head_ = count_ = 0; capturing_ = false; last_state_ = -1; }
        if (now == last_s_) return;
        if (state == 6 && last_state_ != 6) { capturing_ = true; start_s_ = now; }
        const bool exited = capturing_ && state != 6;
        Frame f;
        f.sim_s = now; f.dt_s = last_s_ >= 0 && now > last_s_ ? now - last_s_ : 0.0;
        f.est_s = est_s_.load(); f.state = state;
        f.body_p = body_->WorldPose().Pos(); f.body_rpy = body_->WorldPose().Rot().Euler();
        f.body_v = body_->WorldLinearVel(); f.body_w = body_->WorldAngularVel();
        for (size_t j = 0; j < joints_.size(); ++j) {
            f.q[j] = joints_[j]->Position(0); f.qd[j] = joints_[j]->GetVelocity(0);
        }
        for (const auto& item : massive_links_) {
            f.mass += item.second;
            f.com_p += item.second * item.first->WorldCoGPose().Pos();
            f.com_v += item.second * item.first->WorldCoGLinearVel();
        }
        if (f.mass > 0) { f.com_p /= f.mass; f.com_v /= f.mass; }
        for (size_t leg = 0; leg < 4; ++leg) {
            const auto link = feet_[leg]->GetLink();
            f.foot_p[leg] = feet_[leg]->WorldPose().Pos();
            f.foot_v[leg] = link->WorldCoGLinearVel() +
                link->WorldAngularVel().Cross(f.foot_p[leg] - link->WorldCoGPose().Pos());
        }
        const auto manager = world_->Physics()->GetContactManager();
        f.contact_available = (manager->NeverDropContacts() ||
            manager->SubscribersConnected(feet_[0].get(), feet_[1].get())) ? 1 : 0;
        f.contact_count = static_cast<int>(manager->GetContactCount());
        const auto& all_contacts = manager->GetContacts();
        for (unsigned i = 0; i < manager->GetContactCount() && i < all_contacts.size(); ++i) {
            const auto contact = all_contacts[i];
            if (!contact || !contact->collision1 || !contact->collision2) continue;
            f.contact_s = std::max(f.contact_s, contact->time.Double());
            for (size_t leg = 0; leg < 4; ++leg) {
                const bool first = contact->collision1 == feet_[leg].get();
                const bool second = contact->collision2 == feet_[leg].get();
                if (!first && !second) continue;
                // 排除本机器人自身碰撞；实际脚-环境接触与计划 contact 分开保存。
                const auto other = first ? contact->collision2 : contact->collision1;
                if (other->GetLink()->GetModel() == model_) continue;
                f.foot_contacts[leg] += contact->count;
                for (int k = 0; k < contact->count; ++k)
                    f.contact_force[leg] += first ? contact->wrench[k].body1Force : contact->wrench[k].body2Force;
            }
        }
        f.sample_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count();
        ring_[head_] = f; head_ = (head_ + 1) % ring_.size();
        count_ = std::min(count_ + 1, ring_.size());
        last_s_ = now; last_state_ = state;
        if (!capturing_) {
            // 保留进入 trot 前约 2 秒；按时间裁剪，不假设物理步长恒定。
            while (count_ > 1 && ring_[(head_ + ring_.size() - count_) % ring_.size()].sim_s < now - 2.0) --count_;
        } else if (exited || now - start_s_ >= 10.0) publish();
    }

    void publish() { capturing_ = false; ready_.store(true, std::memory_order_release); }
    static void row(std::ostream& out, const Frame& f, bool header) {
        if (header) out << "sim_s,dt_s,est_s,state,sample_us,mass,contact_available,contact_count,contact_s";
        else out << f.sim_s << ',' << f.dt_s << ',' << f.est_s << ',' << f.state << ','
                 << f.sample_us << ',' << f.mass << ',' << f.contact_available << ',' << f.contact_count << ',' << f.contact_s;
        auto vector = [&](const char* name, const auto& values) {
            for (size_t j = 0; j < values.size(); ++j) {
                out << ','; if (header) out << name << '_' << j; else out << values[j];
            }
        };
        auto xyz = [&](const std::string& name, const V3& value) {
            for (int k = 0; k < 3; ++k) { out << ','; if (header) out << name << '_' << k; else out << value[k]; }
        };
        vector("q", f.q); vector("qd", f.qd);
        xyz("body_p", f.body_p); xyz("body_v", f.body_v); xyz("body_rpy", f.body_rpy);
        xyz("body_w", f.body_w); xyz("com_p", f.com_p); xyz("com_v", f.com_v);
        for (size_t leg = 0; leg < 4; ++leg) {
            xyz("foot_p_" + std::to_string(leg), f.foot_p[leg]);
            xyz("foot_v_" + std::to_string(leg), f.foot_v[leg]);
            xyz("contact_force_" + std::to_string(leg), f.contact_force[leg]);
        }
        vector("foot_contacts", f.foot_contacts); out << '\n';
    }

    void writeLoop() {
        while (!stop_.load() || ready_.load(std::memory_order_acquire)) {
            if (!ready_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue;
            }
            try {
                const auto stamp = std::chrono::system_clock::now().time_since_epoch();
                const std::string path = "/tmp/physics_debug_" + std::to_string(getpid()) + "_" +
                    std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(stamp).count()) + ".csv";
                std::ofstream out(path); out.exceptions(std::ios::failbit | std::ios::badbit);
                out << std::setprecision(17);
                const size_t first = (head_ + ring_.size() - count_) % ring_.size();
                row(out, ring_[first], true);
                for (size_t i = 0; i < count_; ++i) row(out, ring_[(first + i) % ring_.size()], false);
                out.close();
                // 后台汇总站立段：用完整物理步积分排除控制频率采样混叠。
                std::array<double, 12> stand_error{};
                double stand_s = 0.0, max_sample_us = 0.0;
                size_t gaps = 0;
                for (size_t i = 0; i < count_; ++i) {
                    const auto& f = ring_[(first + i) % ring_.size()];
                    max_sample_us = std::max(max_sample_us, f.sample_us);
                    if (i == 0) continue;
                    const auto& previous = ring_[(first + i - 1) % ring_.size()];
                    const double dt = f.sim_s - previous.sim_s;
                    if (dt <= 0 || dt > 1.5 * nominal_step_) { ++gaps; continue; }
                    if (f.state != 4 || previous.state != 4) continue;
                    stand_s += dt;
                    for (size_t j = 0; j < 12; ++j)
                        stand_error[j] += 0.5 * (f.qd[j] + previous.qd[j]) * dt - (f.q[j] - previous.q[j]);
                }
                if (stand_s > 0.0) for (auto& error : stand_error) error /= stand_s;
                RCLCPP_INFO(node_->get_logger(),
                    "[PHYSICS_CHECK] stand_s=%.3f gaps=%zu max_sample_us=%.1f "
                    "stand_velocity_bias_rad_s thigh=[%.4f %.4f %.4f %.4f] calf=[%.4f %.4f %.4f %.4f]",
                    stand_s, gaps, max_sample_us, stand_error[1], stand_error[4], stand_error[7], stand_error[10],
                    stand_error[2], stand_error[5], stand_error[8], stand_error[11]);
                RCLCPP_INFO(node_->get_logger(), "[PHYSICS_DEBUG] saved=%s frames=%zu begin_s=%.6f end_s=%.6f",
                    path.c_str(), count_, ring_[first].sim_s, ring_[(head_ + ring_.size() - 1) % ring_.size()].sim_s);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[PHYSICS_DEBUG] write failed: %s", e.what());
            }
            head_ = count_ = 0;
            ready_.store(false, std::memory_order_release);
        }
    }

    physics::ModelPtr model_; physics::WorldPtr world_; physics::LinkPtr body_;
    std::array<physics::JointPtr, 12> joints_{};
    std::array<physics::CollisionPtr, 4> feet_{};
    std::vector<std::pair<physics::LinkPtr, double>> massive_links_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    transport::NodePtr contact_node_; transport::SubscriberPtr contact_sub_;
    event::ConnectionPtr connection_;
    std::vector<Frame> ring_;
    size_t head_ = 0, count_ = 0;
    double last_s_ = -1.0, start_s_ = 0.0, nominal_step_ = 0.0;
    int last_state_ = -1;
    bool capturing_ = false;
    std::atomic<int> state_{-1}; std::atomic<double> est_s_{0.0};
    std::atomic<bool> ready_{false}, stop_{false};
    std::thread worker_;
};
GZ_REGISTER_MODEL_PLUGIN(GazeboPhysicsDebug)
} // namespace gazebo
