#include "hardware_unitree_mujoco/motor_hw.hpp"

#include <signal.h>
namespace damiao
{    
     
// 定义不同电机类型的限位参数数组（Num_Of_Motor为电机类型总数，需在头文件中定义）
// Limit_param结构体包含：Q_MAX(位置最大值rad)、DQ_MAX(速度最大值rad/s)、TAU_MAX(扭矩最大值Nm)
Limit_param limit_param[Num_Of_Motor]= // 限制参数的初始化在这里
            {
                    {12.5, 30, 10 }, // DM4310 - 位置±12.5rad, 速度±30rad/s, 扭矩±10Nm
                    {12.5, 50, 10 }, // DM4310_48V
                    {12.5, 10, 28 },  // DM4340
                    {12.5, 10, 28 }, // DM4340_48V
                    {12.5, 45, 12 }, // DM6006
                    {12.5, 45, 20 }, // DM8006
                    {12.5, 45, 54 }, // DM8009
                    {12.5,25,  200}, // DM10010L
                    {12.5,20, 200},  // DM10010
                    {12.5,280,1},    // DMH3510
                    {12.5,45,10},    // DMH6215
                    {12.5,45,10}     // DMG6220
            };
            
// 构造函数：根据电机类型索引获取对应的物理限制参数
Motor::Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
        : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type) {
    this->limit_param = damiao::limit_param[Motor_Type];  // 直接数组索引访问，时间复杂度O(1)
    RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
        "Created motor: Type=%d, Slave_id=%d, Master_id=%d", Motor_Type, Slave_id, Master_id);
}

// 更新电机状态缓存：位置(rad)、速度(rad/s)、力矩(Nm)
void Motor::receive_data(float q, float dq, float tau)
{
    this->state_q = q;
    this->state_dq = dq;
    this->state_tau = tau;
}

// 存储float类型参数到电机参数映射表
// 参数说明：
// - key：参数键（对应电机寄存器ID）
// - value：要存储的float类型参数值
// 存储float参数：使用union的floatValue成员，isFloat标志置true

void Motor::set_param(int key, float value)
{
    ValueType v{};
    v.value.floatValue = value;
    v.isFloat = true;
    param_map[key] = v;  // unordered_map插入/更新，平均O(1)
}

// 存储uint32参数：使用union的uint32Value成员，isFloat标志置false
void Motor::set_param(int key, uint32_t value)
{
    ValueType v{};
    v.value.uint32Value = value;
    v.isFloat = false;
    param_map[key] = v;
}

// 安全获取float参数：先查键存在性，再校验类型标志，失败返回0
float Motor::get_param_as_float(int key) const
{
    auto it = param_map.find(key);
    if (it != param_map.end())
    {
        if (it->second.isFloat)
        {
            return it->second.value.floatValue;
        }
        else
        {
            return 0;  // 类型不匹配时返回默认值而非抛出异常，保证实时性
        }
    }
    return 0;
}

// 安全获取float类型参数：键不存在/类型不匹配时返回0（保证实时性，不抛异常）
// 参数：key - 要获取的参数键（寄存器ID）
// 返回：float类型参数值（失败返回0）
uint32_t Motor::get_param_as_uint32(int key) const 
{
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (!it->second.isFloat) {
            return it->second.value.uint32Value;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

// 判断指定参数键是否存在于参数映射表中
// 参数：key - 要检查的参数键（寄存器ID）
// 返回：存在返回true，不存在返回false
bool Motor::is_have_param(int key) const
{
    return param_map.find(key) != param_map.end();
}


/****** 一个串口（port-ACM）对应一个Motor_Control实例，管理该串口下的所有电机 ******/
// Motor_Control构造函数：初始化串口、创建电机实例、配置电机模式、启动数据接收线程
// 参数说明：
// - serial_port：串口名（如/dev/ttyACM0）
// - seial_baud：串口波特率
// - data_ptr：电机控制数据指针（存储各电机的指令/状态数据）
Motor_Control::Motor_Control(std::string serial_port, int seial_baud,std::unordered_map<int, DmActData>* data_ptr)
    // 初始化列表：绑定电机控制数据指针
    :  data_ptr_(data_ptr) 
{
    // 初始化线程停止标志（false表示线程不停止）
    stop_thread_ = false;

    // ROS2日志：记录Motor_Control实例创建信息
    RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
        "Created Motor_Control for port: %s, baudrate: %d", serial_port.c_str(), seial_baud);

    // 遍历电机控制数据，为每个电机创建Motor实例并添加到管理列表
    for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it)  // data_ptr_->begin()返回的迭代器指向unordered_map<int, DmActData>中第一个std::pair<int, DmActData>类型的键值对
    {
        // 动态创建Motor对象：参数为电机类型、从CAN ID、主CAN ID
        Motor* motor = new Motor(it->second.motorType,it->second.can_id, it->second.mst_id); // mst_id就是master ID
        // 将电机添加到Motor_Control的管理映射表
        addMotor(motor);
        // ROS2日志：记录电机添加信息
        RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
            "Added motor: Type=%d, Slave_id=%d, Master_id=%d", motor->GetMotorType(), motor->GetSlaveId(), motor->GetMasterId());
    }

    // 串口参数配置：8N1（8位数据位、无校验、1位停止位），无流控，超时20ms（保证实时性）
    serial_.setPort(serial_port);          // 设置串口名
    serial_.setBaudrate(seial_baud);       // 设置串口波特率
    serial_.setFlowcontrol(serial::flowcontrol_none);  // 无流控
    serial_.setParity(serial::parity_none); // 无校验（默认值，显式设置增加可读性）
    serial_.setStopbits(serial::stopbits_one); // 1位停止位
    serial_.setBytesize(serial::eightbits);    // 8位数据位
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);  // 设置20ms超时
    serial_.setTimeout(time_out);            // 应用超时配置

    serial_.open();  // 打开串口
    usleep(1000000); // 延时1秒，等待串口驱动就绪
    // ROS2日志：确认串口打开成功
    RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
        "Serial port %s opened successfully with baudrate %d", serial_port.c_str(), seial_baud);
    
    // 遍历所有电机，切换到MIT控制模式（达妙电机的阻抗控制模式）
    for(auto& it : motors) // 因为addMotor(motor);函数有点问题，会冗余，所以同一个电机的模式转换会连续执行两次
    {
        switchControlMode(*it.second,damiao::MIT_MODE); // 控制模式切换函数，.second代表键值对的第二个元素
        // ROS2日志：记录电机模式切换信息
        RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
            "Switch mode to MIT_MODE for motor: Type=%d, Slave_id=%d, Master_id=%d", it.second->GetMotorType(), it.second->GetSlaveId(), it.second->GetMasterId());
    }
    set_zero_position();  // 执行电机归零操作（将当前位置设为零点）
    enable();             // 使能该串口下的所有电机
    
    usleep(500000); // 延时0.5秒，等待电机完成初始化
    // 重要！ 创建并启动电机数据接收线程：绑定成员函数和this指针
    rec_thread = std::thread(std::bind(&Motor_Control::get_motor_data_thread, this));
    // 注释行：boost版本的线程创建方式（未启用）
    // rec_thread = std::make_unique<std::thread>(boost::bind(&Motor_Control::get_motor_data_thread, this));
    // 控制台输出：初始化成功提示
    std::cerr<<"Motor_Control init success!"<<std::endl;
}

// 析构函数：先停机所有电机(kd=0.3保证阻尼)，再安全join线程，最后关闭串口，防止资源泄漏
Motor_Control::~Motor_Control()
{   
    std::cerr<<"enter ~Motor_Control()"<<std::endl;
   
    for (const auto& pair : motors)
    {
        Motor_id id = pair.first;
        //std::cerr<<"id: "<<id<<std::endl;
        control_mit(*motors[id], 0, 0.3, 0, 0, 0);  // kd=0.3提供阻尼，防止急停震荡
    }
    stop_thread_ = true; // 停止接收
    disable(); // 失能
    //if (serial_.isOpen())
  //  {
  //    serial_.close(); 
  //  }
    if(rec_thread.joinable()) // 检查线程对象是否处于「可以被等待结束」的状态
    {
      rec_thread.join();  // 阻塞等待线程结束，等待接收线程安全退出，避免野指针
    }
    if (serial_.isOpen())
    {
      serial_.close(); 
    }

}

// 批量使能：每个电机重复发送20次0xFC命令，间隔2ms，提高成功率
void Motor_Control::enable()
{
    for(auto& it : motors)
    {   
       for(int j=0;j<20;j++)
       {
        control_cmd(it.second->GetSlaveId(), 0xFC);  // 使能
        usleep(2000);  // 2ms间隔，避免CAN总线拥塞
       }
    }
}


//读电机反馈命令
// 构造0x7FF广播ID，携带目标电机ID与0xCC命令，请求状态帧
void Motor_Control::refresh_motor_status(const Motor& motor) // 这函数没用过
{
    uint32_t id = 0x7FF; // 0x7FF是这款达妙电机底层通信协议里约定好的固定部分
    uint8_t can_low = motor.GetSlaveId() & 0xff; // id low 8 bit
    uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; //id high 8 bit
    std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(id, data_buf.data());
    serial_.write((uint8_t*)&send_data, sizeof(can_send_frame));
}

// 批量禁用：发送0xFD命令，逻辑与enable对称
void Motor_Control::disable()
{
    for(auto& it : motors)
    {   
       for(int j=0;j<20;j++)
       
       {
        control_cmd(it.second->GetSlaveId(), 0xFD);
        usleep(2000);
       }
    }  
}

// 批量归零：发送0xFE命令，电机会把当前位置设为零点
// 重要！设置零点函数
void Motor_Control::set_zero_position()
{
    for(auto& it : motors)
    {   
        // 每个电机重复发送20次归零指令
       for(int j=0;j<20;j++)
       control_cmd(it.second->GetSlaveId(), 0xFE);
       {
        usleep(2000);
       }
    RCLCPP_INFO(rclcpp::get_logger("motor_hw"), 
        "设置零点成功！: Type=%d, Slave_id=%d, Master_id=%d", it.second->GetMotorType(), it.second->GetSlaveId(), it.second->GetMasterId());
    }  
}

// MIT模式：浮点→定点线性映射，位域打包，单帧8字节，满足达妙私有协议
// 函数作用：将浮点型的电机控制参数（刚度kp、阻尼kd、位置q、速度dq、扭矩tau）
// 转换为达妙电机要求的定点数，按私有协议打包成8字节数据帧，通过串口发送给指定电机
void Motor_Control::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
{
    // 1. 定义浮点转无符号定点数的lambda表达式（static：只初始化一次，提升循环效率）
    // 功能：将有物理意义的浮点数（如kp、q），转换为电机可识别的无符号定点整数
    // 参数说明：
    // x：待转换的浮点数；xmin/xmax：该参数的物理取值范围；bits：定点数的有效位数
    // 返回值：转换后的无符号定点整数（uint16_t）
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin; // 计算参数的物理取值范围（最大值-最小值）
        float data_norm = (x - xmin) / span; // 浮点数归一化到[0,1]区间（反推：把物理量映射到0~1）
        uint16_t data_uint = data_norm * ((1 << bits) - 1); // 归一化值映射到定点数范围（0~2^bits-1）
        return data_uint; // 返回转换后的定点数
    };

    // 2. 获取目标电机的从ID（SlaveId），作为后续校验和发送帧的核心标识
    Motor_id id = DM_Motor.GetSlaveId();

    // 3. 校验电机ID的有效性：查找该SlaveId是否在已注册的motors映射表中
    // motors.find(id)：返回指向该ID的迭代器；motors.end()：映射表尾后迭代器（表示未找到）
    if(motors.find(id) == motors.end())  // end()不是指向容器最后一个元素，而是「尾后迭代器」—— 指向容器最后一个元素的 “下一个不存在的位置”，是 find () 找不到元素时的 “默认返回值”。
    {
        // 若ID未注册，抛出运行时异常，终止函数执行（避免访问不存在的电机对象）
        throw std::runtime_error("Motor_Control id not found");
    }

    // 4. 获取该ID对应的Motor对象指针的引用（&），避免拷贝，提升效率
    // motors[id]：通过SlaveId快速查找映射表中的Motor对象指针
    auto& m = motors[id];

    // 5. 浮点转定点：刚度kp（单位：N·m/rad）→ 12位定点数，物理范围0~500
    // 12位定点数范围：0~4095（2^12-1），对应kp的0~500 N·m/rad
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);

    // 6. 浮点转定点：阻尼kd（单位：N·m·s/rad）→ 12位定点数，物理范围0~5
    // 12位定点数范围：0~4095，对应kd的0~5 N·m·s/rad
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);

    // 7. 获取该电机的参数限制（Q_MAX/DQ_MAX/TAU_MAX）：不同电机的物理范围不同（如关节电机Q_MAX=π，舵机Q_MAX=2π）
    Limit_param limit_param_cmd = m->get_limit_param();

    // 8. 浮点转定点：位置q（单位：rad）→ 16位定点数，物理范围[-Q_MAX, Q_MAX]
    // 16位定点数范围：0~65535，对应q的-Q_MAX~Q_MAX（比如-π~π）
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);

    // 9. 浮点转定点：速度dq（单位：rad/s）→ 12位定点数，物理范围[-DQ_MAX, DQ_MAX]
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);

    // 10. 浮点转定点：扭矩tau（单位：N·m）→ 12位定点数，物理范围[-TAU_MAX, TAU_MAX]
    uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

    // 11. 初始化8字节数据帧缓冲区：按达妙私有协议打包所有定点数，无浪费位
    // 协议约定：8字节 = q(16位) + dq(12位) + kp(12位) + kd(12位) + tau(12位) → 总计64位（8字节）
    std::array<uint8_t, 8> data_buf{};

    // 12. 打包位置q（16位）→ 占data_buf[0]（高8位）+ data_buf[1]（低8位）
    data_buf[0] = (q_uint >> 8) & 0xff; // q_uint右移8位，取高8位（0xff确保只保留8位）
    data_buf[1] = q_uint & 0xff;        // q_uint与0xff，取低8位

    // 13. 打包速度dq（12位）+ 刚度kp高4位 → 占data_buf[2] + data_buf[3]高4位
    data_buf[2] = dq_uint >> 4; // dq_uint右移4位，取高8位（12位的前8位）
    // 拆解：
    // (dq_uint & 0xf) << 4：dq_uint的低4位，左移4位（占data_buf[3]高4位）
    // ((kp_uint >> 8) & 0xf)：kp_uint的高4位（12位的前4位），占data_buf[3]低4位
    // | 按位或：合并两部分，填满data_buf[3]的8位
    data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);

    // 14. 打包刚度kp低8位 → 占data_buf[4]（12位的后8位）
    data_buf[4] = kp_uint & 0xff;

    // 15. 打包阻尼kd（12位）高8位 → 占data_buf[5]
    data_buf[5] = kd_uint >> 4;

    // 16. 打包阻尼kd低4位 + 扭矩tau高4位 → 占data_buf[6]
    // 拆解：
    // (kd_uint & 0xf) << 4：kd_uint的低4位，左移4位（占data_buf[6]高4位）
    // ((tau_uint >> 8) & 0xf)：tau_uint的高4位（12位的前4位），占data_buf[6]低4位
    // | 按位或：合并两部分，填满data_buf[6]的8位
    data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);

    // 17. 打包扭矩tau低8位 → 占data_buf[7]（12位的后8位）
    data_buf[7] = tau_uint & 0xff;

    // 18. 调用can_send_frame的modify函数，配置CAN帧：
    // 第一个参数id：目标电机的SlaveId（非广播ID！定向发送给单个电机）
    // 第二个参数data_buf.data()：指向8字节打包数据的指针，填充到CAN帧的数据段
    send_data.modify(id, data_buf.data());

    // 19. 将配置好的CAN帧结构体，转换为字节指针，通过串口发送给电机
    // (uint8_t*)&send_data：结构体转字节指针，串口按字节发送
    // sizeof(can_send_frame)：发送的总字节数=CAN帧结构体大小
    serial_.write((uint8_t*)&send_data, sizeof(can_send_frame));
}

// 位置+速度模式：8字节=pos(float)+vel(float)，ID偏移POS_MODE
void Motor_Control::control_pos_vel(Motor &DM_Motor,float pos,float vel)
{
    Motor_id id = DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
    }
    std::array<uint8_t, 8> data_buf{};
    memcpy(data_buf.data(), &pos, sizeof(float));
    memcpy(data_buf.data() + 4, &vel, sizeof(float));
    id += POS_MODE;
    send_data.modify(id, data_buf.data());
    serial_.write(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
}

// 纯速度模式：4字节float，ID偏移SPEED_MODE，后4字节补0
void Motor_Control::control_vel(Motor &DM_Motor,float vel)
{
    Motor_id id =DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("VEL ERROR : id not found");
    }
    std::array<uint8_t, 8> data_buf = {0};
    memcpy(data_buf.data(), &vel, sizeof(float));
    id=id+SPEED_MODE;
    send_data.modify(id, data_buf.data());
    serial_.write((uint8_t*)&send_data,sizeof(can_send_frame));
}
   

// 解析电机应答帧：校验头0x11+尾0x55，按寄存器ID分派float/uint32
void Motor_Control::receive_param()
{
    //serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

    if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
    {
        auto & data = receive_data.canData;
        if(data[2]==0x33 or data[2]==0x55)
        {
            uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
            uint8_t RID = data[3];
            if (motors.find(slaveID) == motors.end())
            {
                //can not found motor id
                return;
            }
            if(is_in_ranges(RID))
            {
                uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                motors[slaveID]->set_param(RID, data_uint32);
            }
            else
            {
                float data_float = uint8_to_float(data + 4);
                motors[slaveID]->set_param(RID, data_float);
            }
        }
        return ;
    }
}

/**
 * @brief add motor to class 添加电机到Motor_Control管理列表
 * @param DM_Motor : motor object 电机对象指针
 */
// 双ID映射添加电机：主/从ID都映射到同一电机对象，支持主从ID切换
// 没错 就是添加两遍，但是这样确实会导致冗余
void Motor_Control::addMotor(Motor *DM_Motor)
{
    // 从ID映射到电机对象
    motors.insert({DM_Motor->GetSlaveId(), DM_Motor}); // std::unordered_map的insert函数，本质是向映射表中插入一组新的 键值对
    // 主ID映射到同一电机对象
    motors.insert({DM_Motor->GetMasterId(), DM_Motor});
}
/*
    * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
    * @param DM_Motor: motor object 电机对象
    * @param RID: register id 寄存器ID  example: damiao::UV_Value
    * @return: motor param 电机参数 如果没查询到返回的参数为0
    */
// 带重试的读寄存器：先发0x33命令，轮询receive_param()直到指定RID出现或超时
float Motor_Control::read_motor_param(Motor &DM_Motor,uint8_t RID)
{
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(0x7FF, data_buf.data());
    serial_.write((uint8_t*)&send_data,sizeof(can_send_frame));
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            if (is_in_ranges(RID))
            {
                return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
            }
            else
            {
                return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
            }
        }
    }

    return 0;
}

/*
    * @description: switch control mode 切换电机控制模式
    * @param DM_Motor: motor object 电机对象
    * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
    */
// 写模式寄存器RID=10，再读回验证，带重试，保证模式切换成功
bool Motor_Control::switchControlMode(Motor &DM_Motor,Control_Mode mode)
{
    uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
    uint8_t RID = 10;
    write_motor_param(DM_Motor,RID,write_data);
    if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
    {
        return false;
    }
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
        }
    }
    return false;
}

/*
    * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
    * @param DM_Motor: motor object 电机对象
    * @param RID: register id 寄存器ID
    * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
    * @return: bool true or false  是否修改成功
    */
// 自动识别整型/浮点型：is_in_ranges(RID)决定序列化方式，再读回验证
bool Motor_Control::change_motor_param(Motor &DM_Motor,uint8_t RID,float data)
{
    if(is_in_ranges(RID)) {
        //居然传进来的是整型的范围 救一下
        uint32_t data_uint32 = float_to_uint32(data);
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data_uint32;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    else
    {
        //is float
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
    {
        return false;
    }
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            if (is_in_ranges(RID))
            {
                return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
            }
            else
            {
                return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) - data)<0.1f;
            }
        }
    }
    return false;
}

/*
    * @description: save all param to motor flash 保存电机的所有参数到flash里面
    * @param DM_Motor: motor object 电机对象
    * 电机默认参数不会写到flash里面，需要进行写操作
    */
// 先停机，再发0xAA命令，100ms等待Flash写入完成
void Motor_Control::save_motor_param(Motor &DM_Motor)
{
    disable();
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(0x7FF, data_buf.data());
    serial_.write((uint8_t*)&send_data,sizeof(can_send_frame));
    usleep(100000);//100ms wait for save
}

/*
    * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
    * @param DM_Motor: motor object 电机对象
    * @param P_MAX: position max 位置最大值
    * @param Q_MAX: velocity max 速度最大值
    * @param T_MAX: torque max 扭矩最大值
    */
// 运行时修改软件限幅表，立即生效，不写入电机Flash
void Motor_Control::changeMotorLimit(Motor &DM_Motor,float P_MAX,float Q_MAX,float T_MAX)
{
    limit_param[DM_Motor.GetMotorType()]={P_MAX,Q_MAX,T_MAX};
}

// 底层命令封装：8字节数据仅最后一个字节有效，其余0xFF
void Motor_Control::control_cmd(Motor_id id , uint8_t cmd)
{
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd}; // 数据手册就这么规定的
    send_data.modify(id, data_buf.data());
    serial_.write((uint8_t*)&send_data,sizeof(can_send_frame));
}

// 写寄存器底层：固定0x55命令，4字节数据按小端序填入data_buf[4..7]
void Motor_Control::write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
{
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
    data_buf[4] = data[0];
    data_buf[5] = data[1];
    data_buf[6] = data[2];
    data_buf[7] = data[3];
    send_data.modify(0x7FF, data_buf.data()); // 给uart底层框架最终要发的8字节内容赋值
    serial_.write((uint8_t*)&send_data,sizeof(can_send_frame));
}

// 批量下发MIT命令：遍历data_ptr_中所有电机，使用最新kp/kd/pos/vel/tau
void Motor_Control::write()
{
    for(const auto& m : *data_ptr_)
    {
        int motor_id = m.first;//这里指的是can_id
        if(motors.find(motor_id) == motors.end())
        {
           throw std::runtime_error("read ERROR : Motor_Control id not found");
        }
        auto& it = motors[motor_id];

      /* std::cerr<<"write"<<std::endl;
       std::cerr<<"motor_id: "<<motor_id<<std::endl;
       std::cerr<<"pos: "<<m.second.cmd_pos<<" vel: "<<m.second.cmd_vel<<" effort: "<<m.second.cmd_effort<<std::endl;
       std::cerr<<"kp: "<<m.second.kp<<" kd: "<<m.second.kd<<std::endl;
       std::cerr<<"finishe write"<<std::endl;*/
       control_mit(*it, m.second.kp, m.second.kd,m.second.cmd_pos, m.second.cmd_vel, m.second.cmd_effort); // control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
    }
}




// 接收线程主循环：阻塞read -> 校验头尾 -> 解析定点数 -> 更新电机缓存
// 在 get_motor_data_thread() 中更新数据时加锁
// 接收线程主循环：阻塞read -> 校验头尾 -> 解析定点数 -> 更新电机缓存
// 在 get_motor_data_thread() 中更新数据时加锁
// 函数作用：独立线程持续读取串口的电机反馈数据，解析后更新电机对象缓存，保证主线程不被阻塞
void Motor_Control::get_motor_data_thread() 
{      
    // 线程主循环：stop_thread_是线程停止标志（bool类型，初始false）
    // 只要stop_thread_为false，线程就持续运行，直到外部设置为true才退出
    while (!stop_thread_)
    {  
        // try-catch块：捕获串口读写可能抛出的异常（如串口断开、超时），避免线程崩溃
        try 
        {  
            // 1. 阻塞读取串口数据，填充到receive_data结构体中
            // (uint8_t*)&receive_data：将结构体指针转为字节指针，串口按字节读取数据并填充
            // sizeof(CAN_Receive_Frame)：读取的总字节数=接收帧结构体的大小，保证读取完整帧
            serial_.read((uint8_t*)&receive_data,sizeof(CAN_Receive_Frame)); 
            
            // 2. 校验接收帧的有效性：
            // CMD=0x11：协议约定的“电机反馈数据帧”指令码；frameEnd=0x55：帧尾校验位（表示帧完整）
            // 只有校验通过，才解析后续数据，避免处理无效/损坏的帧
            if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
            {
                // 3. 定义定点数转浮点数的lambda表达式（static：只初始化一次，提升效率）
                // 功能：将电机返回的uint16_t定点数，转为有物理意义的浮点数（位置/速度/扭矩）
                // 参数说明：
                // x：待转换的定点数；xmin/xmax：物理量的最小/最大值；bits：定点数的有效位数
                static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                    float span = xmax - xmin; // 计算物理量的取值范围（最大值-最小值）
                    float data_norm = float(x) / ((1 << bits) - 1); // 定点数归一化到[0,1]（1<<bits等价于2^bits）
                    float data = data_norm * span + xmin; // 归一化值映射到实际物理量范围
                    return data; // 返回转换后的浮点数
                };

                // 4. 定义receive_data.canData的引用（别名），简化后续代码书写
                // canData是存储电机反馈原始数据的数组（6字节有效载荷），引用避免拷贝，提升效率
                auto & data = receive_data.canData;

                // 5. 解析位置(q)的16位定点数：
                // data[1]：位置数据高8位，左移8位；data[2]：位置数据低8位；| 按位或合并为16位整数
                uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                
                // 6. 解析速度(dq)的12位定点数：
                // data[3]：速度数据高8位，左移4位；data[4]>>4：data[4]的高4位（速度低4位）；| 合并为12位整数
                uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                
                // 7. 解析扭矩(tau)的12位定点数：
                // data[4]&0xf：data[4]的低4位（扭矩高4位），左移8位；data[5]：扭矩低8位；| 合并为12位整数
                uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
        
                // 8. 校验电机ID有效性：查找当前帧的canId是否在已注册的motors映射表中
                // motors.find(canId)：返回迭代器；motors.end()：映射表尾后迭代器（表示未找到）
                if(motors.find(receive_data.canId) == motors.end())
                {
                    return;  // 未注册的ID：直接退出当前循环，丢弃该帧，防止访问不存在的电机对象崩溃
                }
                
                // 9. 获取该canId对应的Motor对象指针：motors是<电机ID, Motor*>映射表，[]快速查找
                auto m = motors[receive_data.canId];
                
                // 10. 获取该电机的参数限制：Q_MAX/DQ_MAX/TAU_MAX是位置/速度/扭矩的最大取值（正负对称）
                Limit_param limit_param_receive = m->get_limit_param();
                
                // 11. 位置定点数转浮点数：16位→实际位置值（范围[-Q_MAX, Q_MAX]）
                float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                
                // 12. 速度定点数转浮点数：12位→实际速度值（范围[-DQ_MAX, DQ_MAX]）
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                
                // 13. 扭矩定点数转浮点数：12位→实际扭矩值（范围[-TAU_MAX, TAU_MAX]）
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                
                // 14. 加锁保护数据更新（RAII机制）：
                // {} 是局部作用域，lock_guard超出作用域自动解锁，避免死锁
                // motor_data_mutex_：互斥锁，防止主线程读取数据时，线程正在更新（竞态条件）
                {
                    std::lock_guard<std::mutex> lock(motor_data_mutex_);
                    // 15. 更新电机对象的内部缓存数据：将解析后的位置/速度/扭矩存入Motor对象，供主线程read()调用
                    m->receive_data(receive_q, receive_dq, receive_tau);  // 更新电机对象内部缓存，供read()使用
                }
            }
        } 
        // 捕获串口相关异常（如串口断开、权限不足）
        catch (const serial::SerialException& e) 
        {
            // 打印异常信息，便于调试，线程不退出（继续循环）
            std::cerr << "Serial exception: " << e.what() << std::endl;
        } 
        // 捕获IO异常（如串口读取超时、设备不存在）
        catch (const serial::IOException& e)
        {
            // 打印IO异常信息，线程不退出
            std::cerr << "IO exception: " << e.what() << std::endl;
        }
    }
}

// 在 read() 函数中读取数据时加锁
void Motor_Control::read()
{
    std::lock_guard<std::mutex> lock(motor_data_mutex_);  // 加锁保护数据读取
    for(auto& m : *data_ptr_)
    {
        int motor_id = m.first;//这里指的是can_id
        if(motors.find(motor_id) == motors.end())
        {
           RCLCPP_ERROR(rclcpp::get_logger("motor_hw"), "Motor with ID %d not found", motor_id);
           continue;
        }
        auto& it = motors[motor_id];

        m.second.pos = it->Get_Position();
        m.second.vel = it->Get_Velocity();
        m.second.effort = it->Get_tau();

        // 注释掉调试输出或根据需要保留
        // std::cerr<<"MotorType: "<<motor_id<<std::endl;
        // std::cerr<<"pos: "<<m.second.pos<<" vel: "<<m.second.vel<<" effort: "<<m.second.effort<<std::endl;  
    }
}



}