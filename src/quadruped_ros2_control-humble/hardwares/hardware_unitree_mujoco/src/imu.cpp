#include <hardware_unitree_mujoco/imu.hpp>
//#include <Eigen/Eigen>



namespace FDILink
{
    // 构造函数
  imu::imu(): rclcpp::Node ("IMU_Node")
  {
    
    this->declare_parameter("if_debug_",false);
    this->get_parameter("if_debug_", if_debug_);

    this->declare_parameter<std::string>("serial_port_","/dev/ttyUSB0");
    this->get_parameter("serial_port_", serial_port_);

    this->declare_parameter<std::int64_t>("serial_baud_",921600);
    this->get_parameter("serial_baud_", serial_baud_);  

    this->declare_parameter<std::int64_t>("serial_timeout_",1000);
    this->get_parameter("serial_timeout_", serial_timeout_);  
    
    
    this->declare_parameter<std::string>("imu_frame_id_","imu_link");
    this->get_parameter("imu_frame_id_", imu_frame_id_);  

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
    //初始化串口
    activate_port();
    //开启读取数据线程
    read_thread_ = std::thread(&imu::read_imu_loop, this);

  }
  
  
  // 启动读取数据的线程
  void imu::read_imu_loop()
  {
    is_running_ = true;
    while (is_running_)
    {
      if (is_serial_open())
      {
        //check head start   检查起始 数据帧头
      uint8_t check_head[1] = {0xff};  
      size_t head_s = serial_.read(check_head, 1);
      if (if_debug_){
          if (head_s != 1)
          {
              RCLCPP_ERROR(this->get_logger(),"Read serial port time out! can't read pack head.");
          }
          std::cout << std::endl;
          std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
      }
      if (check_head[0] != FRAME_HEAD)
      {
         continue;
      }
      
      //check head type   检查数据类型
      uint8_t head_type[1] = {0xff};
      size_t type_s = serial_.read(head_type, 1);
      if (if_debug_){
          std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
      }
      if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && 
          head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND&& 
          head_type[0] != 0xff)
      {
          RCLCPP_WARN(this->get_logger(),"head_type error: %02X",head_type[0]);
          continue;
      }
      
      //check head length    检查对应数据类型的长度是否符合
      uint8_t check_len[1] = {0xff};
      size_t len_s = serial_.read(check_len, 1);
      if (if_debug_){
          std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
      }
      if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
      {
          RCLCPP_WARN(this->get_logger(),"head_len error (imu)");
          continue;
      }
      else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
      {
          RCLCPP_WARN(this->get_logger(),"head_len error (ahrs)");
          continue;
      }
      else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
      {
          RCLCPP_WARN(this->get_logger(),"head_len error (insgps)");
          continue;
      }
      else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)
      {
          RCLCPP_WARN(this->get_logger(),"head_len error (GEODETIC_POS)");
          continue;
      }
      else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
      {
          uint8_t ground_sn[1];
          size_t ground_sn_s = serial_.read(ground_sn, 1);
          if (++read_sn_ != ground_sn[0])
          {
              if ( ground_sn[0] < read_sn_)
              {
                  if(if_debug_){
                      RCLCPP_WARN(this->get_logger(),"detected sn lost.");
                  }
                  sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
                  read_sn_ = ground_sn[0];
                  // continue;
              }
              else
              {
                  if(if_debug_){
                      RCLCPP_WARN(this->get_logger(),"detected sn lost.");
                  }
                  sn_lost_ += (int)(ground_sn[0] - read_sn_);
                  read_sn_ = ground_sn[0];
                  // continue;
              }
          }
          uint8_t ground_ignore[500];
          size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
          continue;
      }
      
      //read head sn 
      uint8_t check_sn[1] = {0xff};
      size_t sn_s = serial_.read(check_sn, 1);
      uint8_t head_crc8[1] = {0xff};
      size_t crc8_s = serial_.read(head_crc8, 1);
      uint8_t head_crc16_H[1] = {0xff};
      uint8_t head_crc16_L[1] = {0xff};
      size_t crc16_H_s = serial_.read(head_crc16_H, 1);
      size_t crc16_L_s = serial_.read(head_crc16_L, 1);
      if (if_debug_){
          std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
          std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
          std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
          std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
      }
      
      // put header & check crc8 & count sn lost
      // check crc8 进行crc8数据校验
      if (head_type[0] == TYPE_IMU)
      {
          imu_frame_.frame.header.header_start   = check_head[0];
          imu_frame_.frame.header.data_type      = head_type[0];
          imu_frame_.frame.header.data_size      = check_len[0];
          imu_frame_.frame.header.serial_num     = check_sn[0];
          imu_frame_.frame.header.header_crc8    = head_crc8[0];
          imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
          imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
          uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
          if (CRC8 != imu_frame_.frame.header.header_crc8)
          {
              RCLCPP_WARN(this->get_logger(),"header_crc8 error");
              continue;
          }
          if(!first_sn_){
              read_sn_  = imu_frame_.frame.header.serial_num - 1;
              first_sn_ = true;
          }
          //check sn 
          imu::checkSN(TYPE_IMU);
      }
      else if (head_type[0] == TYPE_AHRS)
      {
          ahrs_frame_.frame.header.header_start   = check_head[0];
          ahrs_frame_.frame.header.data_type      = head_type[0];
          ahrs_frame_.frame.header.data_size      = check_len[0];
          ahrs_frame_.frame.header.serial_num     = check_sn[0];
          ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
          ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
          ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
          uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
          if (CRC8 != ahrs_frame_.frame.header.header_crc8)
          {
              RCLCPP_WARN(this->get_logger(),"header_crc8 error");
              continue;
          }
          if(!first_sn_){
              read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
              first_sn_ = true;
          }
          //check sn 
          imu::checkSN(TYPE_AHRS);
      }
      
      // check crc16 进行crc16数据校验
      if (head_type[0] == TYPE_IMU)
      {
          uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
          uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
          uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
          size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
          // if (if_debug_){
          //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
          //   {
          //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
          //   }
          //   std::cout << std::dec << std::endl;
          // }
          uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
          if (if_debug_)
          {          
              std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
              std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
              std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
              std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
              bool if_right = ((int)head_crc16 == (int)CRC16);
              std::cout << "if_right: " << if_right << std::endl;
          }
          
          if (head_crc16 != CRC16)
          {
              RCLCPP_WARN(this->get_logger(),"check crc16 faild(imu).");
              continue;
          }
          else if(imu_frame_.frame.frame_end != FRAME_END)
          {
              RCLCPP_WARN(this->get_logger(),"check frame end.");
              continue;
          }
      }
      else if (head_type[0] == TYPE_AHRS)
      {
          uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
          uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
          uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
          size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
          // if (if_debug_){
          //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
          //   {
          //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
          //   }
          //   std::cout << std::dec << std::endl;
          // }
          uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
          if (if_debug_)
          {          
              std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
              std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
              std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
              std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
              bool if_right = ((int)head_crc16 == (int)CRC16);
              std::cout << "if_right: " << if_right << std::endl;
          }
          
          if (head_crc16 != CRC16)
          {
              RCLCPP_WARN(this->get_logger(),"check crc16 faild(ahrs).");
              continue;
          }
          else if(ahrs_frame_.frame.frame_end != FRAME_END)
          {
              RCLCPP_WARN(this->get_logger(),"check frame end.");
              continue;
          }
      }
      
      // publish magyaw topic
      //读取IMU数据进行解析
      if (head_type[0] == TYPE_IMU)
      {
          sensor_msgs::msg::Imu temp_imu_data;
          temp_imu_data.header.stamp = this->now();
          temp_imu_data.header.frame_id = imu_frame_id_.c_str();
          Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                    ahrs_frame_.frame.data.data_pack.Qx,
                                    ahrs_frame_.frame.data.data_pack.Qy,
                                    ahrs_frame_.frame.data.data_pack.Qz);
          //用于转换imu姿态
          Eigen::Quaterniond q_r =                          
              Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitZ()) * 
              Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitY()) * 
              Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
          Eigen::Quaterniond q_rr =                          
              Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
              Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
              Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitX());
          Eigen::Quaterniond q_xiao_rr =
              Eigen::AngleAxisd( PI/2, Eigen::Vector3d::UnitZ()) * 
              Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
              Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitX()); 
          //根据不同的需求，选择不同的坐标变换方式，到时再更改
          Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
          temp_imu_data.orientation.w = q_out.w();
          temp_imu_data.orientation.x = q_out.x();
          temp_imu_data.orientation.y = q_out.y();
          temp_imu_data.orientation.z = q_out.z();
          temp_imu_data.angular_velocity.x =  imu_frame_.frame.data.data_pack.gyroscope_x;
          temp_imu_data.angular_velocity.y = -imu_frame_.frame.data.data_pack.gyroscope_y;
          temp_imu_data.angular_velocity.z = -imu_frame_.frame.data.data_pack.gyroscope_z;
          temp_imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
          temp_imu_data.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
          temp_imu_data.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
          {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_imu_data_ = temp_imu_data;
            has_new_data_ = true;
          }
      
      }
        
      }
      else
      {
        RCLCPP_WARN(this->get_logger(),"serial port is not open!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //等待1秒后重试
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
  //检查序列号函数
  void imu::checkSN(int type)
  {
    switch (type)
    {
    case TYPE_IMU:
      if (++read_sn_ != imu_frame_.frame.header.serial_num)
      {
        if ( imu_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
        }
      }
      read_sn_ = imu_frame_.frame.header.serial_num;
      break;

    case TYPE_AHRS:
      if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
      {
        if ( ahrs_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
        }
      }
      read_sn_ = ahrs_frame_.frame.header.serial_num;
      break;

    default:
      break;
    }
  }

  sensor_msgs::msg::Imu imu::return_latest_imu_data()
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    has_new_data_ = false;
    return latest_imu_data_;
  }
  bool imu::check_newdata()
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return has_new_data_;
  }


  


} // namespace FDILink