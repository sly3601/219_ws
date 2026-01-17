cmake_minimum_required(VERSION 3.8)
project(hardware_unitree_mujoco)

# 编译选项（保留）
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 依赖项（保留核心，仅新增serial）
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    serial  # ========== 微改1：添加serial到依赖列表 ==========
    tf2_ros
    geometry_msgs
)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)  # ========== 微改2：新增find_package serial ==========
find_package(tf2_ros REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach ()

# ====================== 核心修复：包含src目录下所有.c/.cpp文件 ======================
# 自动收集src目录下所有.c/.cpp文件（无需手动列，适配你新增的文件）
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)
# ===============================================================================

# 编译库：包含所有src下的文件（核心！解决未定义符号）
include(GNUInstallDirs)
add_library(
    ${PROJECT_NAME}
    SHARED
    ${SRC_FILES}  # 用收集到的所有源文件替代单个文件
)

# RPATH配置（保留，确保库路径正确）
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

# 头文件路径（包含src目录，适配你所有本地头文件）
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src  # 关键：包含src目录，找到所有本地头文件
    ${EIGEN3_INCLUDE_DIR}
    ${serial_INCLUDE_DIRS}  # ========== 微改3：添加serial头文件路径 ==========
)

# 链接依赖（保留，仅新增serial链接）
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp
  Eigen3::Eigen
  ${serial_LIBRARIES}  # ========== 微改4：链接serial库 ==========
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 插件索引（ROS2标准自动配置，无需手动同步）
pluginlib_export_plugin_description_file(hardware_interface HardwareUnitree.xml)

# 基础安装配置（仅保留必要项，无多余）
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/HardwareUnitree.xml
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# 测试/导出（保留）
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()