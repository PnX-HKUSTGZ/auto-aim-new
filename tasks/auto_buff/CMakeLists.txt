cmake_minimum_required(VERSION 3.16)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

add_library(auto_buff OBJECT 
    rune_camera.cpp
    rune_detector.cpp
    rune_observation.cpp
    rune_observation_refiner.cpp
    rune_pose_optimizer.cpp
    power_rune_plane.cpp
    small_rune_motion_estimator.cpp
    big_rune_motion_estimator.cpp
    phase_motion_estimator.cpp
    rune_tracker.cpp
    rune_decision_state_machine.cpp
    buff_aimer.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime Ceres::ceres)
