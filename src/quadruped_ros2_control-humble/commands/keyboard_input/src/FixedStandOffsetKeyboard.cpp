//
// Created by sly on 26-4-14.
//

#include <keyboard_input/FixedStandOffsetKeyboard.hpp>

#include <sys/select.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>

// 关节名称表：索引必须和控制器里的 joint 顺序严格一致
static const char* kJointNames[12] = {
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint"
};

FixedStandOffsetKeyboard::FixedStandOffsetKeyboard()
    : Node("fixedstand_offset_keyboard_node")
{
    fixedstand_offset_pub_ =
        create_publisher<std_msgs::msg::Int32MultiArray>("/fixedstand_offset_cmd", 10);

    timer_ = create_wall_timer(
        std::chrono::microseconds(100),
        std::bind(&FixedStandOffsetKeyboard::timer_callback, this));

    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);

    RCLCPP_INFO(get_logger(), "FixedStand 微调键盘节点已启动。");
    RCLCPP_INFO(get_logger(), "[ / ] ：切换当前选中的关节（0~11）");
    RCLCPP_INFO(get_logger(), "- / + ：让当前关节 offset 减少 / 增加 0.01 rad");
    RCLCPP_INFO(get_logger(), "r     ：将当前关节的 offset 清零");
    RCLCPP_INFO(get_logger(), "当前默认选中的关节：%s", kJointNames[selected_joint_]);
}

void FixedStandOffsetKeyboard::timer_callback()
{
    if (!kbhit())
    {
        return;
    }

    char key = getchar();
    check_fixedstand_offset(key);
}

bool FixedStandOffsetKeyboard::check_fixedstand_offset(char key)
{
    switch (key)
    {
    case '[':
        selected_joint_ = (selected_joint_ + 11) % 12;
        RCLCPP_INFO(get_logger(), "当前选中的关节：%s", kJointNames[selected_joint_]);
        return true;

    case ']':
        selected_joint_ = (selected_joint_ + 1) % 12;
        RCLCPP_INFO(get_logger(), "当前选中的关节：%s", kJointNames[selected_joint_]);
        return true;

    case '-':
    case '_':
        publish_fixedstand_offset(selected_joint_, -1);
        RCLCPP_INFO(get_logger(), "关节 %s 的 offset 减少 0.01 rad", kJointNames[selected_joint_]);
        return true;

    case '=':
    case '+':
        publish_fixedstand_offset(selected_joint_, +1);
        RCLCPP_INFO(get_logger(), "关节 %s 的 offset 增加 0.01 rad", kJointNames[selected_joint_]);
        return true;

    case 'r':
    case 'R':
        publish_fixedstand_offset(selected_joint_, 0);
        RCLCPP_INFO(get_logger(), "关节 %s 的 offset 已清零", kJointNames[selected_joint_]);
        return true;

    default:
        return false;
    }
}

void FixedStandOffsetKeyboard::publish_fixedstand_offset(int joint_index, int op)
{
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(joint_index);
    msg.data.push_back(op);
    fixedstand_offset_pub_->publish(msg);
}

bool FixedStandOffsetKeyboard::kbhit()
{
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FixedStandOffsetKeyboard>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}