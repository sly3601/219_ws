//
// Created by biao on 24-9-11.
//

#include <keyboard_input/KeyboardInput.h>

#include <QApplication>
#include <QKeyEvent>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

namespace {
constexpr double hold_threshold_seconds = 0.10;
constexpr double ramp_seconds = 3.0 - hold_threshold_seconds;
constexpr auto trotting_timeout = std::chrono::milliseconds(250);

bool isDirection(int key) {
    return key == Qt::Key_W || key == Qt::Key_S || key == Qt::Key_A || key == Qt::Key_D;
}

class KeyboardWindow final : public QWidget {
public:
    explicit KeyboardWindow(KeyboardInput& input) : input_(input) {
        setWindowTitle(QStringLiteral("SYSU219 键盘控制"));
        setFocusPolicy(Qt::StrongFocus);
        auto* layout = new QVBoxLayout(this);
        layout->addWidget(&status_);
        layout->addWidget(new QLabel(QStringLiteral("W/S 前后走   A/D 原地转   长按 3 秒到上限"), this));
        layout->addWidget(new QLabel(QStringLiteral("1 被动   2 固定姿态   4 踏步   空格清零"), this));
        refreshStatus();
    }

    void refreshStatus() {
        status_.setText(input_.trottingActive() ? QStringLiteral("MPC/WBC 踏步 · WASD 已启用")
                                               : QStringLiteral("WASD 等待进入 MPC/WBC 踏步"));
    }

private:
    void keyPressEvent(QKeyEvent* event) override {
        if (event->matches(QKeySequence::Close) ||
            (event->key() == Qt::Key_C && event->modifiers() == Qt::ControlModifier)) {
            close();
        } else if (!(event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier))) {
            input_.press(event->key(), event->isAutoRepeat());
        }
    }

    void keyReleaseEvent(QKeyEvent* event) override {
        if (!event->isAutoRepeat()) input_.release(event->key());
    }

    void focusOutEvent(QFocusEvent* event) override {
        input_.releaseMotion();
        QWidget::focusOutEvent(event);
    }

    KeyboardInput& input_;
    QLabel status_;
};
}  // namespace

KeyboardInput::KeyboardInput() : Node("keyboard_input_node") {
    publisher_ = create_publisher<control_input_msgs::msg::Inputs>("/control_input", 10);
    motion_publisher_ = create_publisher<control_input_msgs::msg::Inputs>("/trotting_motion_input", 1);
    // 足端标记只在 StateTrotting 实际运行时周期发布。
    trotting_frames_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/foot_markers", rclcpp::QoS(1).best_effort(),
        [this](const visualization_msgs::msg::MarkerArray&) { last_trotting_frame_ = Clock::now(); });
}

bool KeyboardInput::trottingActive() const {
    return last_trotting_frame_ != Clock::time_point{} && Clock::now() - last_trotting_frame_ < trotting_timeout;
}

float KeyboardInput::heldFraction(Clock::time_point now) const {
    const double held_seconds = std::chrono::duration<double>(now - pressed_at_).count();
    return static_cast<float>(std::clamp((held_seconds - hold_threshold_seconds) / ramp_seconds, 0.0, 1.0));
}

void KeyboardInput::press(int key, bool repeat) {
    if (!rclcpp::ok()) return;
    if (isDirection(key)) {
        if (!repeat && trottingActive() && direction_ == 0) {
            direction_ = key;
            pressed_at_ = Clock::now();
        }
        return;
    }

    if (key >= Qt::Key_0 && key <= Qt::Key_9) {
        if (repeat) return;
        stopMotion();
        last_trotting_frame_ = {};
        inputs_ = control_input_msgs::msg::Inputs{};
        inputs_.command = key == Qt::Key_0 ? 10 : key - Qt::Key_0;
        publisher_->publish(inputs_);
        clear_command_at_ = Clock::now() + std::chrono::milliseconds(20);
        return;
    }

    if (key == Qt::Key_Space) {
        if (repeat) return;
        stopMotion();
        inputs_ = control_input_msgs::msg::Inputs{};
        clear_command_at_ = {};
        publisher_->publish(inputs_);
        return;
    }

    if (trottingActive() || inputs_.command != 0) return;
    constexpr float step = 0.05f;
    switch (key) {
        case Qt::Key_I: inputs_.ry = std::min(inputs_.ry + step, 1.0f); break;
        case Qt::Key_K: inputs_.ry = std::max(inputs_.ry - step, -1.0f); break;
        case Qt::Key_J: inputs_.rx = std::max(inputs_.rx - step, -1.0f); break;
        case Qt::Key_L: inputs_.rx = std::min(inputs_.rx + step, 1.0f); break;
        default: return;
    }
    publisher_->publish(inputs_);
}

void KeyboardInput::release(int key) {
    if (key == direction_) releaseMotion();
}

void KeyboardInput::releaseMotion() {
    if (direction_ == 0 || released_at_ != Clock::time_point{}) return;
    released_at_ = Clock::now();
    released_fraction_ = heldFraction(released_at_);
    if (released_fraction_ == 0.0f) stopMotion();
}

void KeyboardInput::stopMotion() {
    direction_ = 0;
    released_at_ = {};
    if (rclcpp::ok()) motion_publisher_->publish(control_input_msgs::msg::Inputs{});
}

void KeyboardInput::update() {
    const auto now = Clock::now();
    if (clear_command_at_ != Clock::time_point{} && now >= clear_command_at_) {
        inputs_.command = 0;
        clear_command_at_ = {};
        publisher_->publish(inputs_);
    }

    if (direction_ == 0) return;
    if (!trottingActive()) {
        stopMotion();
        return;
    }

    float fraction = heldFraction(now);
    if (released_at_ != Clock::time_point{}) {
        const double released_seconds = std::chrono::duration<double>(now - released_at_).count();
        fraction = released_fraction_ - static_cast<float>(released_seconds / ramp_seconds);
        if (fraction <= 0.0f) {
            stopMotion();
            return;
        }
    }

    control_input_msgs::msg::Inputs motion;
    switch (direction_) {
        case Qt::Key_W: motion.ly = fraction; break;
        case Qt::Key_S: motion.ly = -fraction; break;
        case Qt::Key_A: motion.rx = -fraction; break;
        case Qt::Key_D: motion.rx = fraction; break;
    }
    motion_publisher_->publish(motion);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto input = std::make_shared<KeyboardInput>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(input);

    KeyboardWindow window(*input);
    window.show();
    window.setFocus();
    QTimer timer;
    timer.setTimerType(Qt::PreciseTimer);
    QObject::connect(&timer, &QTimer::timeout, [&] {
        if (!rclcpp::ok()) {
            app.quit();
            return;
        }
        executor.spin_some();
        input->update();
        window.refreshStatus();
    });
    timer.start(20);

    const int result = app.exec();
    if (rclcpp::ok()) {
        input->stopMotion();
        rclcpp::shutdown();
    }
    return result;
}
