//
// Created by sly on 26-4-14.
//
#ifndef FIXEDSTANDOFFSETKEYBOARD_H
#define FIXEDSTANDOFFSETKEYBOARD_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <termios.h>

class FixedStandOffsetKeyboard final : public rclcpp::Node {
public:
    FixedStandOffsetKeyboard();

    ~FixedStandOffsetKeyboard() override {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

private:
    void timer_callback();
    bool check_fixedstand_offset(char key);
    void publish_fixedstand_offset(int joint_index, int op);
    static bool kbhit();

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fixedstand_offset_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int selected_joint_ = 0;

    termios old_tio_{}, new_tio_{};
};

#endif // FIXEDSTANDOFFSETKEYBOARD_H