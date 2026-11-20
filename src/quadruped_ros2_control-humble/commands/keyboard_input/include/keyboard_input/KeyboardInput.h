//
// Created by biao on 24-9-11.
//


#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class KeyboardInput final : public rclcpp::Node {
public:
    KeyboardInput();
    void press(int key, bool repeat);
    void release(int key);
    void releaseMotion();
    void stopMotion();
    void update();
    bool trottingActive() const;

private:
    using Clock = std::chrono::steady_clock;
    float heldFraction(Clock::time_point now) const;

    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr motion_publisher_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr trotting_frames_;
    Clock::time_point last_trotting_frame_{};
    Clock::time_point clear_command_at_{};
    Clock::time_point pressed_at_{};
    Clock::time_point released_at_{};
    int direction_ = 0;
    float released_fraction_ = 0.0f;
};


#endif //KEYBOARDINPUT_H
