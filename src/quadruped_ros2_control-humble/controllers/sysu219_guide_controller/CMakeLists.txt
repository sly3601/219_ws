cmake_minimum_required(VERSION 3.8)
project(sysu219_guide_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        realtime_tools
        std_msgs
        control_input_msgs
        controller_common
        kdl_parser

        rclcpp
        visualization_msgs
        geometry_msgs
        tf2_ros

        hpipm_catkin
        blasfeo_catkin
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

set(ONNXRUNTIME_ROOT /opt/onnxruntime)
include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_ROOT}/lib)

foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED

        src/Sysu219GuideController.cpp

        src/FSM/StateFixedStand.cpp
        src/FSM/StateSwingTest.cpp
        src/FSM/StateFreeStand.cpp
        src/FSM/StateBalanceTest.cpp
        src/FSM/StateTrotting.cpp

        src/robot/QuadrupedRobot.cpp
        src/robot/RobotLeg.cpp

        src/control/Estimator.cpp
        src/control/LowPassFilter.cpp
        src/control/BalanceCtrl.cpp
        src/control/ConvexMpcSolver.cpp

        src/quadProgpp/Array.cc
        src/quadProgpp/QuadProg++.cc

        src/gait/WaveGenerator.cpp
        src/gait/FeetEndCalc.cpp
        src/gait/GaitGenerator.cpp

        src/debug/foot_marker_publisher.cpp

        src/control/RLPolicy.cpp
        src/FSM/StateRLWalk.cpp

)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        ${BLASFEO_INCLUDE_DIR}
        PRIVATE
        src)
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${CONTROLLER_INCLUDE_DEPENDS}
        ocs2_core
)


# --disable-new-dtags:
#   让 INSTALL_RPATH 更倾向写成传统 RPATH，而不是 RUNPATH。
#   RPATH 对 hpipm -> blasfeo 这种间接依赖更稳。
#
# --no-as-needed:
#   防止链接器因为当前库没有直接调用 blasfeo 符号，就把 libblasfeo.so 依赖裁掉。
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        "-Wl,--disable-new-dtags"
        "-Wl,--no-as-needed"
        PUBLIC
        hpipm
        blasfeo

        onnxruntime
)

# 让 ROS2 pluginlib 加载 libsysu219_guide_controller.so 时，
# 能从同一个 workspace 的 install 目录里找到 hpipm / blasfeo 动态库。
# 不依赖用户手动 export LD_LIBRARY_PATH 或 sudo ldconfig。
set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_WITH_INSTALL_RPATH FALSE
        SKIP_BUILD_RPATH FALSE
        SKIP_INSTALL_RPATH FALSE
        INSTALL_RPATH "$ORIGIN;$ORIGIN/..;$ORIGIN/../../../hpipm_catkin/lib;$ORIGIN/../../../blasfeo_catkin/lib"
)




pluginlib_export_plugin_description_file(controller_interface sysu219_guide_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
