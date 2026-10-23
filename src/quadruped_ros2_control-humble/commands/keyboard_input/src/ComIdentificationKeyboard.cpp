#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <spawn.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
class Terminal {
public:
    Terminal() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0)
            throw std::runtime_error("请在交互式终端运行质心辨识键盘");
        auto raw = saved_; raw.c_lflag &= ~(ICANON | ECHO); raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) throw std::runtime_error("终端设置失败");
    }
    ~Terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
private:
    termios saved_{};
};
class Keyboard final : public rclcpp::Node {
public:
    Keyboard() : Node("com_identification_keyboard") {
        processor_ = declare_parameter<std::string>("processor_path",
            ament_index_cpp::get_package_share_directory("sysu219_guide_controller") +
            "/COM_identification/process_com.py");
        pub_ = create_publisher<std_msgs::msg::String>("/com_identification/command", 10);
        status_sub_ = create_subscription<std_msgs::msg::UInt8>("/com_identification/status", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                fresh_ = std::chrono::steady_clock::now();
                if (status_ == msg->data) return;
                status_ = msg->data;
                const char* states[] = {"未进入（仅稳定FixStand可进入）", "保持，等待a",
                    "缓慢过渡", "等待持续停稳", "正式采集", "已停稳，准备交还FixStand", "中止保持，未失能",
                    "采集完成，缓慢返回原FixStand目标", "返回后等待静止，尚未交接"};
                if (status_ < 9) RCLCPP_INFO(get_logger(), "%s", states[status_]);
            });
        feedback_sub_ = create_subscription<std_msgs::msg::String>("/com_identification/feedback", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { RCLCPP_INFO(get_logger(), "%s", msg->data.c_str()); });
        dataset_sub_ = create_subscription<std_msgs::msg::String>("/com_identification/dataset",
            rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.empty() || msg->data == processed_) return;
                processed_ = msg->data;
                // No shell and no controller-thread processing. Child is entirely offline.
                char python[] = "python3";
                char* args[] = {python, const_cast<char*>(processor_.c_str()),
                                const_cast<char*>(processed_.c_str()), nullptr};
                const int error = posix_spawnp(&child_, python, nullptr, nullptr, args, environ);
                if (error != 0) {
                    child_ = -1; RCLCPP_ERROR(get_logger(), "自动处理启动失败(%d)，数据在 %s", error, processed_.c_str());
                }
                else RCLCPP_INFO(get_logger(), "采集文件已关闭，启动离线计算：%s", processed_.c_str());
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
            send("heartbeat");
            if (child_ > 0) {
                int status = 0;
                if (waitpid(child_, &status, WNOHANG) == child_) {
                    if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
                        RCLCPP_INFO(get_logger(), "计算结束：查看采集目录下 results.json 和 report.md；未修改模型");
                    else RCLCPP_WARN(get_logger(), "计算未通过或无法启动：检查终端及采集目录 report.md");
                    child_ = -1;
                }
            }
        });
        RCLCPP_INFO(get_logger(),
            "\n质心辨识：先用原流程稳定站到FixStand，关闭其他运动键盘。\n"
            " a：一键进入并完成13姿态全流程（约4～5分钟）；b：只接管保持\n"
            " 1：立即中止并保持最后关节目标/增益，不失能、不自动回中\n"
            " h：中止后人工请求8秒以上慢速回中，不恢复采集\n"
            " x / Ctrl+C：发送1后退出键盘；掉线超过0.5秒也中止保持\n"
            " 全程四足落地，默认站高±5mm、roll/pitch±2°，不抬腿、不步行。\n"
            " 1是软件中止保持，并非保证站稳的硬件急停；无独立足底力反馈。\n"
            " 正常完成后慢速返回、确认静止，再自动回原FixStand；之后用原键盘手动趴下，再断电。\n"
            " 全程不自动失能/趴下/断电；中止时不自动退出，每次激活仅采一套。\n"
            " 自动计算使用本键盘所在电脑的文件路径；远程键盘需共享采集目录。\n");
    }
    void stop() { send("1"); }
    bool key(char c) {
        if (c == '1' || c == 'x') { stop(); return c != 'x'; }
        if (std::chrono::steady_clock::now() - fresh_ > std::chrono::milliseconds(500)) {
            RCLCPP_WARN(get_logger(), "控制器状态不新鲜，未发送动作"); return true;
        }
        if (c == 'a' || c == 'b' || c == 'h') send(std::string(1, c));
        return true;
    }
private:
    void send(const std::string& value) { std_msgs::msg::String msg; msg.data = value; pub_->publish(msg); }
    std::string processor_, processed_;
    pid_t child_ = -1;
    uint8_t status_ = 255;
    std::chrono::steady_clock::time_point fresh_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_sub_, dataset_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, signalHandler); std::signal(SIGTERM, signalHandler);
    int result = 0;
    try {
        Terminal terminal;
        auto node = std::make_shared<Keyboard>();
        while (rclcpp::ok() && !interrupted) {
            rclcpp::spin_some(node);
            fd_set set; FD_ZERO(&set); FD_SET(STDIN_FILENO, &set); timeval timeout{0, 10000};
            const int ready = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
            if (ready < 0) { if (errno == EINTR) continue; break; }
            if (ready > 0) { char c = 0; if (read(STDIN_FILENO, &c, 1) <= 0 || !node->key(c)) break; }
        }
        for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
            node->stop(); rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
    rclcpp::shutdown(); return result;
}
