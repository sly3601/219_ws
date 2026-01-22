* 该版本正式成为云端版本V1系列
* 2026.01.14 V1.1 完成了新硬件接口的初步对接！
* 2026.01.16 V1.2 补充了imu的硬件底层（未测试）
* colcon build   --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco   --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release   --event-handlers console_direct+   --continue-on-error   --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install  # 仅加这行，确保路径统一
* 2026.01.17 V1.3 完全完成了imu和电机底层移植 在rviz中显示
* 2026.01.17 V1.4 

* LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
