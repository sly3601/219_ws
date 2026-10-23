#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
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
                    "缓慢过渡", "定时保持，记录运动质量", "原始数据记录中", "记录完成，准备交还FixStand", "中止保持，未失能",
                    "采集完成，缓慢返回原FixStand目标", "返回后定时保持，尚未交接"};
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
                    child_ = -1;
                    RCLCPP_ERROR(get_logger(),
                        "[COM_ID][SPAWN_FAILED] Python进程未启动：error=%d (%s)，processor=%s，dataset=%s",
                        error, std::strerror(error), processor_.c_str(), processed_.c_str());
                }
                else RCLCPP_INFO(get_logger(),
                    "[COM_ID][PROCESS_STARTED] 采集文件已关闭；pid=%d，processor=%s，dataset=%s",
                    static_cast<int>(child_), processor_.c_str(), processed_.c_str());
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
            send("heartbeat");
            if (child_ > 0) {
                int status = 0;
                const auto finished = waitpid(child_, &status, WNOHANG);
                if (finished == child_) {
                    if (WIFEXITED(status)) {
                        const int code = WEXITSTATUS(status);
                        if (code == 0)
                            RCLCPP_INFO(get_logger(),
                                "[COM_ID][QUALITY_PASSED] 离线计算成功，内部检查通过；参数仍为未独立验证的候选值。报告：%s/report.md，详细诊断：%s/debug.json",
                                processed_.c_str(), processed_.c_str());
                        else if (code == 2)
                            RCLCPP_WARN(get_logger(),
                                "[COM_ID][QUALITY_FAILED] 离线计算成功、候选参数已生成；内部质量未通过。具体姿态/关节/数值/阈值见上方输出及 %s/report.md；详细诊断：%s/debug.json",
                                processed_.c_str(), processed_.c_str());
                        else if (code == 3)
                            RCLCPP_WARN(get_logger(),
                                "[COM_ID][DATA_REJECTED] 离线程序已运行，但输入数据未通过完整性或有效性检查。原因及处理阶段：%s/report.md，%s/debug.json",
                                processed_.c_str(), processed_.c_str());
                        else if (code == 4)
                            RCLCPP_ERROR(get_logger(),
                                "[COM_ID][PROCESSING_ERROR] 离线处理或文件读写异常；查看上方具体异常，诊断文件可写时保存在 %s/debug.json",
                                processed_.c_str());
                        else
                            RCLCPP_ERROR(get_logger(),
                                "[COM_ID][UNEXPECTED_EXIT] Python退出码=%d；查看上方Python错误（含依赖/脚本路径等）。processor=%s，dataset=%s",
                                code, processor_.c_str(), processed_.c_str());
                    } else if (WIFSIGNALED(status)) {
                        RCLCPP_ERROR(get_logger(),
                            "[COM_ID][PROCESS_SIGNAL] 离线进程被信号%d终止；processor=%s，dataset=%s",
                            WTERMSIG(status), processor_.c_str(), processed_.c_str());
                    }
                    child_ = -1;
                } else if (finished < 0 && errno != EINTR) {
                    const int error = errno;
                    RCLCPP_ERROR(get_logger(), "[COM_ID][WAIT_FAILED] waitpid失败：error=%d (%s)，pid=%d，dataset=%s",
                                 error, std::strerror(error), static_cast<int>(child_), processed_.c_str());
                    child_ = -1;
                }
            }
        });
        RCLCPP_INFO(get_logger(),
            "\n质心辨识：先用原流程稳定站到FixStand，关闭其他运动键盘。\n"
            " a：一键完成6段辨识（默认约40秒，停稳等待可能延长）；b：只接管保持\n"
            " 1：立即中止并保持最后关节目标/增益，不失能、不自动回中\n"
            " h：中止后人工请求平滑回中（默认3秒），不恢复采集\n"
            " x / Ctrl+C：发送1后退出键盘；掉线超过0.5秒也中止保持\n"
            " 全程四足落地，默认roll/pitch±4°，不抬腿、不步行；倾角为目标变化量。\n"
            " 顺序：原位、pitch正、roll正、pitch负、roll负、回中复测；不再插入升降和重复回中。\n"
            " 默认每次过渡3秒、定时保持1秒、记录2秒；运动质量只标记，不打断整套记录。\n"
            " 1是软件中止保持，并非保证站稳的硬件急停；无独立足底力反馈。\n"
            " 正常完成后平滑返回、定时保持，再自动回原FixStand；之后用原键盘手动趴下，再断电。\n"
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
