#include <hardware_sysu219/imu.hpp>
#include <chrono>
#include <cstring>

namespace FDILink
{
  // 构造函数：串口/波特率/frame_id/debug 均由硬件接口从 YAML 参数文件读入后传入
  imu::imu(const std::string& serial_port, int serial_baud,
           const std::string& frame_id, bool debug)
    : rclcpp::Node("IMU_Node")
  {
    if_debug_ = debug;
    serial_port_ = serial_port;
    serial_baud_ = serial_baud;
    serial_timeout_ = 1000;
    imu_frame_id_ = frame_id;

    memset(&raw_, 0, sizeof(raw_));

    latest_imu_diagnostics_.fill(std::numeric_limits<double>::quiet_NaN());
    latest_imu_diagnostics_[0] = latest_imu_diagnostics_[2] = 0.0;
    latest_imu_diagnostics_[3] = 0.0;
    latest_imu_diagnostics_[11] = 0.0;

    //开始接收IMU数据
    init();
  }

  // 析构函数
  imu::~imu()
  {
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    deactivate_port();
  }

  // 初始化函数，初始化串口，开启数据接收线程
  void imu::init()
  {
    activate_port();
    read_thread_ = std::thread(&imu::read_imu_loop, this);
  }

  // 启动读取数据的线程
  void imu::read_imu_loop()
  {
    struct RunningGuard {
      std::atomic<bool>& running;
      explicit RunningGuard(std::atomic<bool>& value) : running(value) {
        running.store(true, std::memory_order_relaxed);
      }
      ~RunningGuard() { running.store(false, std::memory_order_relaxed); }
    } running_guard(rx_thread_running_);

    is_running_ = true;
    while (is_running_)
    {
      if (!is_serial_open())
      {
        RCLCPP_WARN(this->get_logger(),"serial port is not open!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //等待1秒后重试
        continue;
      }

      // 逐字节读入并喂给 hipnuc 解码器
      uint8_t byte = 0;
      size_t n = serial_.read(&byte, 1);
      if (n != 1)
      {
        continue;
      }

      int rev = hipnuc_input(&raw_, byte);
      if (rev < 0)
      {
        crc_error_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (rev == 0)
      {
        continue; // 同步中或数据不足，继续读下一字节
      }

      // rev > 0：完整解码出一帧
      double received_steady_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();

      bool has_quat = false, has_gyr = false, has_acc = false;
      double qw = 0, qx = 0, qy = 0, qz = 0;
      double gx = 0, gy = 0, gz = 0;
      double ax = 0, ay = 0, az = 0;
      double device_time_us = std::numeric_limits<double>::quiet_NaN();
      double frame_tag = std::numeric_limits<double>::quiet_NaN();

      if (raw_.hi83.tag == 0x83)
      {
        ++hi83_count_;
        frame_tag = 0x83;
        const uint32_t bm = raw_.hi83.data_bitmap;
        has_quat = bm & HI83_BMAP_QUAT;
        has_gyr  = bm & HI83_BMAP_GYR_B;
        has_acc  = bm & HI83_BMAP_ACC_B;
        if (has_quat) { qw = raw_.hi83.quat[0]; qx = raw_.hi83.quat[1]; qy = raw_.hi83.quat[2]; qz = raw_.hi83.quat[3]; }
        if (has_gyr)  { gx = raw_.hi83.gyr_b[0]; gy = raw_.hi83.gyr_b[1]; gz = raw_.hi83.gyr_b[2]; } // 单位：rad/s
        if (has_acc)  { ax = raw_.hi83.acc_b[0]; ay = raw_.hi83.acc_b[1]; az = raw_.hi83.acc_b[2]; } // 单位：m/s^2
        if (bm & HI83_BMAP_SYSTEM_TIME) device_time_us = static_cast<double>(raw_.hi83.system_time_us);
      }
      else if (raw_.hi91.tag == 0x91)
      {
        ++hi91_count_;
        frame_tag = 0x91;
        has_quat = true;
        has_gyr  = true;
        has_acc  = true;
        qw = raw_.hi91.quat[0]; qx = raw_.hi91.quat[1]; qy = raw_.hi91.quat[2]; qz = raw_.hi91.quat[3];
        gx = raw_.hi91.gyr[0] * DEG_TO_RAD; // deg/s -> rad/s
        gy = raw_.hi91.gyr[1] * DEG_TO_RAD;
        gz = raw_.hi91.gyr[2] * DEG_TO_RAD;
        ax = raw_.hi91.acc[0] * GRA_ACC;    // g -> m/s^2
        ay = raw_.hi91.acc[1] * GRA_ACC;
        az = raw_.hi91.acc[2] * GRA_ACC;
        device_time_us = static_cast<double>(raw_.hi91.system_time) * 1000.0; // ms -> us
      }
      else
      {
        continue; // 未知帧类型，忽略
      }

      // 只有当四元数、角速度、加速度都齐全时才组装成有效 IMU 组合。
      if (has_quat && has_gyr && has_acc)
      {
        sensor_msgs::msg::Imu temp_imu_data;
        temp_imu_data.header.stamp = this->now();
        temp_imu_data.header.frame_id = imu_frame_id_.c_str();

        // hipnuc 四元数顺序为 (w, x, y, z)。
        Eigen::Quaterniond q_ahrs(qw, qx, qy, qz);

        // N300 原始轴：X 向机身左、Y 向机身后、Z 向上。
        // 机身 = M * IMU，M = [[0,-1,0],[1,0,0],[0,0,1]]；在此统一变换一次。
        const double sq2 = 0.7071067811865476;  // sqrt(2)/2
        Eigen::Quaterniond conj_q_m(sq2, 0.0, 0.0, -sq2);  // conj(q_M)，(w,x,y,z)
        Eigen::Quaterniond q_out = q_ahrs * conj_q_m;

        temp_imu_data.orientation.w = q_out.w();
        temp_imu_data.orientation.x = q_out.x();
        temp_imu_data.orientation.y = q_out.y();
        temp_imu_data.orientation.z = q_out.z();

        temp_imu_data.angular_velocity.x = -gy;
        temp_imu_data.angular_velocity.y =  gx;
        temp_imu_data.angular_velocity.z =  gz;
        temp_imu_data.linear_acceleration.x = -ay;
        temp_imu_data.linear_acceleration.y =  ax;
        temp_imu_data.linear_acceleration.z =  az;

        ++rx_sequence_;
        rx_steady_s_ = received_steady_s;

        ImuDiagnostics diagnostics{{
            static_cast<double>(rx_sequence_), rx_steady_s_,
            static_cast<double>(hi83_count_), static_cast<double>(hi91_count_),
            device_time_us, frame_tag,
            has_gyr ? 1.0 : 0.0, has_acc ? 1.0 : 0.0,
            static_cast<double>(crc_error_.load(std::memory_order_relaxed)), 0.0,
            1.0, 1.0}};
        {
          std::lock_guard<std::mutex> lock(imu_mutex_);
          latest_imu_data_ = temp_imu_data;
          latest_imu_diagnostics_ = diagnostics;
          has_new_data_ = true;
        }
      }
    }
  }

  //激活串口
  void imu::activate_port()
  {
    try
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Serial port initialized successfully! "
        "Port: %s, Baudrate: %d, "
        "Parity: None, Stopbits: 1, Bytesize: 8, "
        "Flow control: None, Timeout: %d ms",
        serial_port_.c_str(),   // 串口号（std::string转C字符串）
        serial_baud_,           // 波特率（整数）
        serial_timeout_         // 超时时间（毫秒）
      );
      serial_.setPort(serial_port_);
      serial_.setBaudrate(serial_baud_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none); //default is parity_none
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
      serial_.setTimeout(time_out);
      serial_.open();
      RCLCPP_INFO(this->get_logger(),"SUCCESS: Serial Port Opened");
    }
    catch (serial::IOException &e)  // 抓取异常
    {
      RCLCPP_ERROR(this->get_logger(),"Unable to open port ");
      exit(0);
    }
    if (serial_.isOpen())
    {
      RCLCPP_INFO(this->get_logger(),"Serial Port initialized");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),"Unable to initial Serial port ");
      exit(0);
    }
  }

  //判断串口是否打开，在每次读取数据前调用，用来检查串口状态
  bool imu::is_serial_open()
  {
    return serial_.isOpen();
  }

  //关闭串口，析构函数调用
  void imu::deactivate_port()
  {
    if (serial_.isOpen())
     {serial_.close();}
    RCLCPP_INFO(this->get_logger(),"Serial Port Closed，IMU Stop get data");
  }

  //检查配置的波特率和串口能否成功打开,用于在接口类中调用，调试用
  void imu::print_serial_info()
  {
    RCLCPP_INFO(this->get_logger(), "SUCCESS: Serial Port: %s, Baud Rate: %d, Timeout: %d", serial_port_.c_str(), serial_baud_, serial_timeout_);
  }

  sensor_msgs::msg::Imu imu::return_latest_imu_data(ImuDiagnostics* diagnostics, bool* had_new_data)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (had_new_data) *had_new_data = has_new_data_;
    if (diagnostics) {
      *diagnostics = latest_imu_diagnostics_;
      // 健康计数即使没有新有效IMU组合也继续可观测。
      (*diagnostics)[8] = static_cast<double>(crc_error_.load(std::memory_order_relaxed));
      (*diagnostics)[9] = 0.0;
      (*diagnostics)[10] = rx_thread_running_.load(std::memory_order_relaxed) ? 1.0 : 0.0;
    }
    has_new_data_ = false;
    return latest_imu_data_;
  }

  bool imu::check_newdata()
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return has_new_data_;
  }

} // namespace FDILink
