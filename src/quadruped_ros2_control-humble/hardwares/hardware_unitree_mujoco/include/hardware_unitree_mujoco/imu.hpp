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
#include <hardware_unitree_mujoco/crc_table.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <thread>

using namespace std;
using namespace Eigen;
namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_GROUND 0xf0

#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x48 //80
#define GEODETIC_POS_LEN 0x20 //32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295



#pragma pack(1)
struct fdilink_header
{
	uint8_t  header_start;
	uint8_t  data_type;
	uint8_t  data_size;
	uint8_t  serial_num;
	uint8_t  header_crc8;
    uint8_t  header_crc16_h;
	uint8_t  header_crc16_l;
};
#pragma pack()

#pragma pack(1)
struct IMUData_Packet_t
{
	float gyroscope_x;          //unit: rad/s
	float gyroscope_y;          //unit: rad/s
	float gyroscope_z;          //unit: rad/s
	float accelerometer_x;      //m/s^2
	float accelerometer_y;      //m/s^2
	float accelerometer_z;      //m/s^2
	float magnetometer_x;       //mG
	float magnetometer_y;       //mG
	float magnetometer_z;       //mG
	float imu_temperature;      //C
	float Pressure;             //Pa
	float pressure_temperature; //C
	int64_t Timestamp;          //us
};
#pragma pack()


//for IMU=========================
#pragma pack(1)
struct read_imu_struct{
  fdilink_header     header;    //7                
  union data
  {
	IMUData_Packet_t   data_pack; //56
	uint8_t            data_buff[56]; //56
  }data;
  uint8_t            frame_end; //1                  
};            

struct read_imu_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[57];
};                           

union imu_frame_read{
  struct read_imu_struct frame;
  read_imu_tmp read_buf;
  uint8_t read_tmp[64];
};
#pragma pack()
//for IMU------------------------


#pragma pack(1)

struct AHRSData_Packet_t
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	int64_t Timestamp; //unit: us
};

struct read_ahrs_struct{
  fdilink_header     header;    //7                
  union data
  {
	AHRSData_Packet_t  data_pack; //48
	uint8_t            data_buff[48]; //48
  }data;
  uint8_t            frame_end; //1                  
};       
#pragma pack()
 
struct read_ahrs_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[49];
};                           

union ahrs_frame_read{
  struct read_ahrs_struct frame;
  read_ahrs_tmp read_buf;
  uint8_t read_tmp[56];
};
#pragma pack()



class imu : public rclcpp::Node
{
public:
  // 构造函数
  imu();
  // 析构函数
  ~imu();
  // 初始化函数
  void init();
  
  // 启动读取数据的线程
  void read_imu_loop();
  
  //检查配置的波特率和串口能否成功打开,用于在接口类中调用，调试用
  void print_serial_info();

  sensor_msgs::msg::Imu return_latest_imu_data();

  bool check_newdata();
  


  //激活串口
  void activate_port();
  //判断串口是否打开，在每次读取数据前调用，用来检查串口状态
  bool is_serial_open();
  //关闭串口，析构函数调用
  void deactivate_port();
  
  //检查序列号函数
  void checkSN(int type);
  

private:
  bool if_debug_;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool first_sn_ = false;
  std::string imu_frame_id_;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;
  //data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  
  // 线程相关
  std::thread read_thread_;
  std::mutex imu_mutex_;       // 数据缓存互斥锁
  sensor_msgs::msg::Imu latest_imu_data_; // 最新IMU数据缓存
  bool has_new_data_ = false;          // 是否有新数据标记
  std::atomic<bool> is_running_ = false; // 线程运行标记（原子类型）
  
 
  
}; 
} // namespace FDILink

#endif