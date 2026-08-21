cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    ba_solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim PUBLIC io openvino::runtime tinympcstatic
    g2o::core g2o::types_slam3d g2o::solver_dense Sophus::Sophus)

# types_slam3d depends on opengl_helper, but --as-needed otherwise removes it
# from the executable's direct dependencies.  Keeping it direct allows the
# executable's RUNPATH to find it without changing the lookup priority of
# unrelated libraries (notably the HikRobot SDK selected by LD_LIBRARY_PATH).
if(UNIX AND NOT APPLE)
    target_link_libraries(auto_aim INTERFACE
        "-Wl,--push-state,--no-as-needed"
        g2o::opengl_helper
        "-Wl,--pop-state")
endif()
