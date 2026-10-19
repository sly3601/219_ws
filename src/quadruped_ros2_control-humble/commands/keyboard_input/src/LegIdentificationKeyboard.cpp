#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <cerrno>
#include <chrono>
#include <cctype>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }

class Terminal {
public:
    Terminal() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0)
            throw std::runtime_error("请在交互式终端运行专用辨识键盘");
        auto raw = saved_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            throw std::runtime_error("无法设置终端输入模式");
    }
    ~Terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
private:
    termios saved_{};
};

class LegIdentificationKeyboard : public rclcpp::Node {
public:
    LegIdentificationKeyboard() : Node("leg_identification_keyboard") {
        pub_ = create_publisher<std_msgs::msg::String>("/leg_identification/command", 10);
        sub_ = create_subscription<std_msgs::msg::UInt8>("/leg_identification/status", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                last_status_ = std::chrono::steady_clock::now();
                if (status_ == msg->data) return;
                status_ = msg->data;
                switch (status_) {
                    case 0: RCLCPP_INFO(get_logger(), "未在辨识状态：仅 b 可请求从PASSIVE进入，其他动作键无效。"); break;
                    case 1: RCLCPP_INFO(get_logger(), "已进入辨识，尚未开始驱动；q 单独归位，测试键自动先准备再开始。"); break;
                    case 2: RCLCPP_INFO(get_logger(), "归位准备完成，中心姿态保持；选择 w/e/r/t/y/u/a 开始测试。"); break;
                    case 3: RCLCPP_INFO(get_logger(), "正在准备：缓慢归位或等待实测到位；正式测试尚未开始，1 可急停。"); break;
                    case 4: RCLCPP_INFO(get_logger(), "正在执行辨识动作；h 平滑回中，1 急停。"); break;
                    case 5: RCLCPP_WARN(get_logger(), "测试/全套流程已中止；左前腿保持触发时的位置和增益，不失能。q 归位，或重新选择测试。"); break;
                    default: break;
                }
            });
        feedback_sub_ = create_subscription<std_msgs::msg::String>(
            "/leg_identification/feedback", rclcpp::QoS(64).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // The controller sends notices, while only this terminal renders them.
                if (msg->data.size() < 2 || msg->data[1] != '\n') return;
                const char* text = msg->data.c_str() + 2;
                if (msg->data[0] == 'W') RCLCPP_WARN(get_logger(), "%s", text);
                else RCLCPP_INFO(get_logger(), "%s", text);
            });
        heartbeat_ = create_wall_timer(std::chrono::milliseconds(100), [this] { send("heartbeat"); });
        help();
    }

    void stop() { send("1"); }

    bool key(unsigned char input) {
        const char c = static_cast<char>(std::tolower(input));
        if (c == '1') { stop(); RCLCPP_WARN(get_logger(), "已发送急停：中止测试，保持当前位置和增益，不失能。"); return true; }
        if (c == 'x') { stop(); return false; }
        if (c == '?') { help(); return true; }
        if (c >= '0' && c <= '9') {
            RCLCPP_WARN(get_logger(), "数字键只有 1 用于急停；动作使用字母键。"); return true;
        }
        if (std::chrono::steady_clock::now() - last_status_ > std::chrono::milliseconds(500)) {
            RCLCPP_WARN(get_logger(), "尚无新鲜的控制器状态，动作未发送。"); return true;
        }
        if (c == 'b') {
            if (status_ == 0) { send("enter"); RCLCPP_INFO(get_logger(), "请求进入辨识：仅当前为PASSIVE时接受。"); }
            else RCLCPP_INFO(get_logger(), "已经在辨识状态，不能退出。");
            return true;
        }
        if (status_ == 0) { RCLCPP_WARN(get_logger(), "先在PASSIVE按 b 进入辨识。"); return true; }
        if (std::string("qwertyuah").find(c) == std::string::npos) return true;
        send(std::string(1, c));
        const char* text = "平滑回到中心保持";
        switch (c) {
            case 'q': text = "仅归位准备，不开始正式测试"; break;
            case 'w': text = "先准备，再做三个姿态静态保持：15秒×3"; break;
            case 'e': text = "先准备，再做髋关节耸肩侧往复：0～37°，20秒"; break;
            case 'r': text = "先准备，再做大腿关节激励：中心0.5135、幅值±0.80 rad，20秒"; break;
            case 't': text = "先准备，再做小腿关节激励：中心-1.605、幅值±0.48 rad，20秒"; break;
            case 'y': text = "先准备，再做三关节联合多频激励：40秒×3"; break;
            case 'u': text = "先准备，再做独立验证轨迹：40秒×2"; break;
            case 'a': text = "先准备，再做全套实验：正式动作段305秒，另加准备与归位时间"; break;
            default: break;
        }
        RCLCPP_INFO(get_logger(), "已发送 %c 请求：%s（等待控制器执行确认）", c, text);
        return true;
    }

private:
    void send(const std::string& command) {
        std_msgs::msg::String msg; msg.data = command; pub_->publish(msg);
    }
    void help() {
        RCLCPP_INFO(get_logger(),
            "\n腿部参数辨识专用键盘（仅左前腿）\n"
            "  b：从PASSIVE进入辨识；进入后不再切换其他FSM\n"
            "  q：仅缓慢归位到辨识中心[0, 0.5135, -1.605]rad，不开始正式测试\n"
            "  w：静态   e：髋   r：大腿   t：小腿\n"
            "  y：联合多频   u：独立验证   a：全套实验\n"
            "  h：平滑回中保持   1：急停并保持当前位置，不失能   ?：帮助\n"
            "  x / Ctrl+C：先急停，再退出键盘（不退出辨识FSM）\n"
            "  先 b 进入辨识，再选测试；测试键自带缓慢归位和到位确认。\n"
            "  本终端依次显示：开始准备、准备完成、正式开始、测试完成、数据记录完成。\n"
            "  髋只做0～37°耸肩侧往复，负角溜肩侧不做正式激励。\n"
            "  大腿[-0.2865, 1.3135]rad，小腿[-2.085, -1.125]rad。\n"
            "  单关节20秒，联合/验证每次40秒；源数据采集仍为250Hz。\n"
            "  运动异常只按明显越界（原限位外约20°）或角速度超过10rad/s判断；中止后保持，不清零增益。\n"
            "  每段有测试名称与次数；急停或切换只提示中止，不伪报测试完成。");
    }
    uint8_t status_ = 255;
    std::chrono::steady_clock::time_point last_status_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_;
};
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
    int result = 0;
    try {
        Terminal terminal;
        auto node = std::make_shared<LegIdentificationKeyboard>();
        while (rclcpp::ok() && !interrupted) {
            rclcpp::spin_some(node);
            fd_set set; FD_ZERO(&set); FD_SET(STDIN_FILENO, &set);
            timeval timeout{0, 10000};
            const int selected = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
            if (selected < 0) { if (errno == EINTR) continue; break; }
            if (selected == 0) continue;
            unsigned char c = 0;
            const auto count = read(STDIN_FILENO, &c, 1);
            if (count <= 0 || !node->key(c)) break;
        }
        for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
            node->stop();
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } catch (const std::exception& e) {
        std::cerr << "辨识键盘：" << e.what() << '\n'; result = 1;
    }
    rclcpp::shutdown();
    return result;
}
