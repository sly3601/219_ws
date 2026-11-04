"""独立 IMU 读取节点：读 hipnuc（超核电子）串口，发布 sensor_msgs/Imu 到 /imu/data_raw。

与硬件接口 / 电机完全解耦，可单独启动，供 imu_transform_tuner 等调试工具使用。
只做最小编码解析（hi91 / hi83 两个协议分支），发布 orientation / angular_velocity /
linear_acceleration，不做任何坐标系旋转——原始数据交给调试工具去标定轴映射。

串口读写用 Python 标准库（os + termios + select），不依赖 pyserial。
"""

import math
import os
import select
import struct
import termios
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

CHSYNC1 = 0x5A
CHSYNC2 = 0xA5
CH_HDR_SIZE = 6
DEG_TO_RAD = 0.017453292519943295
GRA_ACC = 9.8

# hi83 data_bitmap 位定义（与 hipnuc_dec.h 一致）
HI83_BMAP_ACC_B = 1 << 0
HI83_BMAP_GYR_B = 1 << 1
HI83_BMAP_MAG_B = 1 << 2
HI83_BMAP_RPY = 1 << 3
HI83_BMAP_QUAT = 1 << 4


def _crc16(data, crc=0):
    crc &= 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            temp = crc << 1
            if crc & 0x8000:
                temp ^= 0x1021
            crc = temp
        crc &= 0xFFFF
    return crc


def _open_serial(port, baud):
    baud_const = getattr(termios, 'B%d' % baud, None)
    if baud_const is None:
        raise ValueError(f'不支持的波特率 {baud}')

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                          # iflag
        attrs[1] = 0                          # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1
        attrs[3] = 0                          # lflag（raw 模式）
        attrs[4] = baud_const                 # ispeed
        attrs[5] = baud_const                 # ospeed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1           # 0.1s
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def _parse_hi91(buf):
    """返回 (quat(w,x,y,z), gyr(rad/s), acc(m/s^2))；buf 是完整 hi91 子包。"""
    if len(buf) < 76:
        return None
    vals = struct.unpack_from('<BHBfI3f3f3f3f4f', buf, 0)
    acc = vals[5:8]     # g
    gyr = vals[8:11]    # deg/s
    quat = vals[17:21]  # (w, x, y, z)
    gyr_rad = [g * DEG_TO_RAD for g in gyr]
    acc_ms2 = [a * GRA_ACC for a in acc]
    return (quat, gyr_rad, acc_ms2)


def _parse_hi83(buf):
    """返回 (quat(w,x,y,z), gyr(rad/s), acc(m/s^2))；buf 是完整 hi83 子包。"""
    if len(buf) < 8 or buf[0] != 0x83:
        return None
    bm = struct.unpack_from('<I', buf, 4)[0]
    idx = 8
    quat = None
    gyr = None
    acc = None
    if bm & HI83_BMAP_ACC_B:
        acc = struct.unpack_from('<3f', buf, idx)
        idx += 12
    if bm & HI83_BMAP_GYR_B:
        gyr = struct.unpack_from('<3f', buf, idx)
        idx += 12
    if bm & HI83_BMAP_MAG_B:
        idx += 12
    if bm & HI83_BMAP_RPY:
        idx += 12
    if bm & HI83_BMAP_QUAT:
        quat = struct.unpack_from('<4f', buf, idx)
        idx += 16
    # hi83 的 gyr_b/acc_b 已经是 rad/s、m/s^2，quat 是 (w,x,y,z)
    return (quat, gyr, acc)


class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.declare_parameter('serial_port', '/dev/sysu219/ttyACM4')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('frame_id', 'imu_link')

        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Imu, self._imu_topic, qos)

        self._fd = None
        try:
            self._fd = _open_serial(self._serial_port, self._baud_rate)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'无法打开串口 {self._serial_port}: {e}')
        else:
            self.get_logger().info(
                f'串口已打开 {self._serial_port} @ {self._baud_rate}，'
                f'发布 {self._imu_topic}')

        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        buf = bytearray()
        while rclpy.ok() and not self._stop:
            if self._fd is None:
                break
            try:
                r, _, _ = select.select([self._fd], [], [], 0.2)
            except OSError as e:
                self.get_logger().error(f'select 失败: {e}')
                break
            if not r:
                continue
            try:
                data = os.read(self._fd, 1024)
            except OSError as e:
                self.get_logger().error(f'串口读取失败: {e}')
                break
            if not data:
                continue
            for byte in data:
                buf.append(byte)
                # 同步帧头 0x5A 0xA5
                if len(buf) < 2:
                    if buf[-1] != CHSYNC1:
                        buf.clear()
                    continue
                if buf[0] != CHSYNC1 or buf[1] != CHSYNC2:
                    del buf[0]
                    continue
                if len(buf) < CH_HDR_SIZE:
                    continue
                payload_len = struct.unpack_from('<H', buf, 2)[0]
                total = CH_HDR_SIZE + payload_len
                if len(buf) < total:
                    continue
                frame = bytes(buf[:total])
                del buf[:total]
                self._handle_frame(frame)

    def _handle_frame(self, frame):
        payload = frame[CH_HDR_SIZE:]
        crc = _crc16(frame[:4] + payload)
        crc_field = struct.unpack_from('<H', frame, 4)[0]
        if crc != crc_field:
            return

        i = 0
        while i < len(payload):
            tag = payload[i]
            if tag == 0x91:
                sub = payload[i:i + 76]
                if len(sub) < 76:
                    break
                parsed = _parse_hi91(sub)
                if parsed:
                    self._publish(*parsed)
                i += 76
            elif tag == 0x83:
                parsed = _parse_hi83(payload[i:])
                if parsed and parsed[0] is not None and parsed[1] is not None \
                        and parsed[2] is not None:
                    self._publish(*parsed)
                break
            else:
                break

    def _publish(self, quat, gyr, acc):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.w = float(quat[0])
        msg.orientation.x = float(quat[1])
        msg.orientation.y = float(quat[2])
        msg.orientation.z = float(quat[3])
        msg.angular_velocity.x = float(gyr[0])
        msg.angular_velocity.y = float(gyr[1])
        msg.angular_velocity.z = float(gyr[2])
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        self._pub.publish(msg)

    def shutdown(self):
        self._stop = True
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None


def main(args=None):
    rclpy.init(args=args)
    node = ImuReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
