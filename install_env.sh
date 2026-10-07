#!/bin/bash
###############################################################################
# ROS2 四足机器人项目环境配置脚本（ROS2 已安装，跳过步骤2）
# 适用于 Ubuntu 22.04 + ROS2 humble
###############################################################################
# 不使用 set -e，按步骤独立容忍失败


WORKSPACE_DIR="/home/sysu219/219_ws"
SERIAL_DIR="/home/sysu219/serial_lib_ws"
LOG_FILE="/tmp/ros2_install.log"

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log() { echo -e "${GREEN}[$(date +%H:%M:%S)]${NC} $1" | tee -a "$LOG_FILE"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)]${NC} $1" | tee -a "$LOG_FILE"; }
err() { echo -e "${RED}[$(date +%H:%M:%S)]${NC} $1" | tee -a "$LOG_FILE"; }

# 检查 ROS2 humble
if [ ! -f /opt/ros/humble/setup.bash ]; then
    err "未检测到 ROS2 humble，请先安装 ROS2 humble"
    exit 1
fi
log "检测到 ROS2 humble，跳过安装"

###############################################################################
# 步骤1：安装基础工具
###############################################################################
log "============================================================"
log "步骤 1/7：安装基础工具"
log "============================================================"
sudo apt-get update || warn "apt-get update 部分失败，继续"
sudo apt-get install -y \
    git curl wget gnupg software-properties-common \
    build-essential cmake g++ gcc libudev-dev \
    python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete \
    || warn "部分基础工具安装失败，继续"

# 初始化 rosdep（如果未初始化）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "初始化 rosdep"
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>&1 | tail -5 || warn "rosdep update 失败，继续"

###############################################################################
# 步骤2：安装 ROS2 相关依赖包（跳过 ROS2 本体）
###############################################################################
log "============================================================"
log "步骤 2/7：安装 ROS2 相关依赖包"
log "============================================================"
sudo apt-get install -y \
    ros-humble-hardware-interface \
    ros-humble-controller-interface \
    ros-humble-realtime-tools \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-imu-sensor-broadcaster \
    ros-humble-controller-manager \
    ros-humble-ros2controlcli || warn "部分 ROS2 依赖安装可能失败，请检查日志"

###############################################################################
# 步骤3：安装 ONNX Runtime C++
###############################################################################
log "============================================================"
log "步骤 3/7：安装 ONNX Runtime C++"
log "============================================================"
ORT_VERSION="1.18.1"
if [ ! -d "/opt/onnxruntime" ]; then
    log "下载 ONNX Runtime ${ORT_VERSION}..."
    cd /tmp
    if [ ! -f "onnxruntime-linux-x64-${ORT_VERSION}.tgz" ]; then
        wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz
    fi

    log "解压..."
    tar -xzf onnxruntime-linux-x64-${ORT_VERSION}.tgz

    log "安装到 /opt/onnxruntime ..."
    sudo mkdir -p /opt/onnxruntime
    sudo cp -r onnxruntime-linux-x64-${ORT_VERSION}/* /opt/onnxruntime/

    log "配置动态库路径..."
    echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
    sudo ldconfig
    log "ONNX Runtime 安装完成"
else
    log "ONNX Runtime 已安装，跳过"
fi

###############################################################################
# 步骤4：安装串口相关库
###############################################################################
log "============================================================"
log "步骤 4/7：安装串口相关库"
log "============================================================"
sudo apt-get install -y \
    ros-humble-serial-driver \
    libserial-dev \
    cutecom || warn "部分串口库安装失败，请检查日志"

###############################################################################
# 步骤5：源码编译安装 serial 库
###############################################################################
log "============================================================"
log "步骤 5/7：源码编译安装 serial 库"
log "============================================================"

# 加载 ROS2 环境（serial 是 ament_cmake 项目，需要 ROS2 环境）
source /opt/ros/humble/setup.bash

if [ ! -f "/usr/local/lib/libserial.so" ] || [ ! -f "/usr/lib/x86_64-linux-gnu/libserial.so" ]; then
    log "克隆 serial 库..."
    if [ ! -d "$SERIAL_DIR/serial" ]; then
        mkdir -p "$SERIAL_DIR"
        cd "$SERIAL_DIR"
        git clone https://github.com/ZhaoXiangBox/serial.git
    fi

    cd "$SERIAL_DIR/serial"

    # 清理旧构建
    rm -rf build install log

    # 在 ROS2 环境下用 colcon 编译
    log "使用 colcon 在 ROS2 环境下编译 serial..."
    source /opt/ros/humble/setup.bash
    colcon build \
        --packages-select serial \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+ > /tmp/serial_colcon.log 2>&1

    if [ $? -ne 0 ]; then
        err "colcon 编译 serial 失败，查看 /tmp/serial_colcon.log"
        tail -30 /tmp/serial_colcon.log
        exit 1
    fi

    # 安装到系统目录
    log "复制编译产物到 /usr/local/lib 和 /usr/local/include..."
    sudo cp install/serial/lib/libserial.so /usr/local/lib/ 2>/dev/null || \
        find install -name "libserial.so" -exec sudo cp {} /usr/local/lib/ \;

    sudo mkdir -p /usr/local/include/serial
    sudo cp include/serial/serial.h /usr/local/include/serial/ 2>/dev/null || true
    sudo cp include/serial/v8stdint.h /usr/local/include/serial/ 2>/dev/null || true

    sudo ldconfig
    log "serial 库源码编译安装完成"
    log "库文件：$(ls /usr/local/lib/libserial.so 2>/dev/null && echo 'OK' || echo '未找到')"
else
    log "serial 库已安装，跳过"
fi

###############################################################################
# 步骤6：配置环境变量
###############################################################################
log "============================================================"
log "步骤 6/7：配置环境变量"
log "============================================================"

BASHRC="$HOME/.bashrc"
BACKUP="$HOME/.bashrc.bak.$(date +%Y%m%d_%H%M%S)"
cp "$BASHRC" "$BACKUP" 2>/dev/null || true

# 添加 ROS2 环境
grep -q "source /opt/ros/humble/setup.bash" "$BASHRC" || cat >> "$BASHRC" << 'EOF'

# >>> ROS2 humble 环境 >>>
source /opt/ros/humble/setup.bash
# <<< ROS2 humble 环境 <<<

EOF

# 添加 ONNX Runtime 环境变量
grep -q "ONNXRUNTIME_ROOT" "$BASHRC" || cat >> "$BASHRC" << 'EOF'

# >>> ONNX Runtime 环境变量 >>>
export ONNXRUNTIME_ROOT=/opt/onnxruntime
export LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH
# <<< ONNX Runtime 环境变量 <<<

EOF

log "环境变量已配置到 $BASHRC"

###############################################################################
# 步骤7：编译项目
###############################################################################
log "============================================================"
log "步骤 7/7：编译 ROS2 项目"
log "============================================================"
cd "$WORKSPACE_DIR"

if [ ! -d "src" ]; then
    err "未找到 src 目录，请确认项目路径"
    exit 1
fi

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

log "运行 rosdep install 安装项目依赖..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble 2>&1 | tail -20 || warn "rosdep install 部分失败，继续编译"

log "开始 colcon build..."
colcon build \
    --packages-up-to ocs2_core leg_pd_controller sysu219_guide_controller sysu219_description keyboard_input hardware_sysu219 \
    --symlink-install \
    --event-handlers console_direct+ \
    --continue-on-error \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install 2>&1 | tail -80

log "============================================================"
log "环境配置与编译全部完成！"
log "============================================================"
log "激活环境："
log "  source /opt/ros/humble/setup.bash"
log "  source $WORKSPACE_DIR/install/setup.bash"
log ""
log "启动 gazebo 仿真："
log "  ros2 launch sysu219_guide_controller gazebo.launch.py"
log "============================================================"
