#ifndef IMU_HPP
#define IMU_HPP

#include <inttypes.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <unistd.h>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <math.h>
#include <fstream>

#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <thread>
#include <array>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <limits>

// hipnuc（超核电子）串口协议解码器，源码已内嵌到本包 src/ 下。
#include "hipnuc_dec.h"

using namespace std;
using namespace Eigen;

// 命名空间名沿用 FDILink（历史遗留，避免牵动 HardwareSysu219 的引用）。
namespace FDILink
{
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295
#define GRA_ACC 9.8

// 顺序与 hw_debug/imu.* 接口一致。
// [0] rx_sequence        有效 IMU 组合帧计数（quat+gyr+acc 齐全）
// [1] rx_steady_s        最近一次有效组合帧的 steady 秒
// [2] hi83_frame_count   hi83 帧计数
// [3] hi91_frame_count   hi91 帧计数
// [4] device_time_us     设备时间戳(us)
// [5] frame_tag          当前帧 tag（0x83 / 0x91）
// [6] gyr_valid          角速度是否有效(0/1)
// [7] acc_valid          加速度是否有效(0/1)
// [8] crc_error_count    解码/校验错误次数（实时覆盖）
// [9] reserved           保留（恒 0）
// [10] rx_thread_running 读取线程运行标志（实时覆盖）
// [11] source_valid      当前组合是否完整有效(0/1)
using ImuDiagnostics = std::array<double, 12>;

class imu : public rclcpp::Node
{
public:
  // 构造函数（串口/波特率/frame_id/debug 由硬件接口从 YAML 参数文件读入后传入）
  imu(const std::string& serial_port = "/dev/sysu219/ttyACM4",
      int serial_baud = 921600,
      const std::string& frame_id = "imu_link",
      bool debug = false);
  // 析构函数
  ~imu();
  // 初始化函数
  void init();

  // 启动读取数据的线程
  void read_imu_loop();

  //检查配置的波特率和串口能否成功打开,用于在接口类中调用，调试用
  void print_serial_info();

  sensor_msgs::msg::Imu return_latest_imu_data(
      ImuDiagnostics* diagnostics = nullptr, bool* had_new_data = nullptr);

  bool check_newdata();

  //激活串口
  void activate_port();
  //判断串口是否打开，在每次读取数据前调用，用来检查串口状态
  bool is_serial_open();
  //关闭串口，析构函数调用
  void deactivate_port();

private:
  bool if_debug_;
  std::string imu_frame_id_;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;

  // hipnuc 解码器原始缓存（逐字节喂入 hipnuc_input）
  hipnuc_raw_t raw_;

  // 线程相关
  std::thread read_thread_;
  std::mutex imu_mutex_;       // 数据缓存互斥锁
  sensor_msgs::msg::Imu latest_imu_data_; // 最新IMU数据缓存
  ImuDiagnostics latest_imu_diagnostics_{}; // 与上面组合在同一mutex下发布/读取。

  // 诊断计数
  std::uint64_t rx_sequence_ = 0;
  double rx_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t hi83_count_ = 0;
  std::uint64_t hi91_count_ = 0;
  std::atomic<std::uint64_t> crc_error_{0};
  std::atomic<bool> rx_thread_running_{false};
  bool has_new_data_ = false;          // 是否有新数据标记
  std::atomic<bool> is_running_ = false; // 线程运行标记（原子类型）
};

} // namespace FDILink

#endif
