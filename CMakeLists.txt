cmake_minimum_required(VERSION 3.16)

project(MyTracker LANGUAGES CXX)

include(CTest)

option(BUILD_TRACKER_VISUALIZER "Build the OpenCV image-sequence visualizer" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(tracking_core STATIC
    src/geometry.cpp
    src/hungarian.cpp
    src/kalman_box_tracker.cpp
    src/ocsort_association.cpp
    src/ocsort_reupdate.cpp
    src/ocsort_tracker.cpp
    src/sort_tracker.cpp
)

target_compile_features(tracking_core PUBLIC cxx_std_17)
target_include_directories(tracking_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tracking_core PRIVATE Eigen3::Eigen)

add_executable(tracker_replay
    tools/tracker_replay.cpp
    tools/replay_input.cpp
)
target_include_directories(tracker_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(tracker_replay PRIVATE tracking_core)

if(BUILD_TRACKER_VISUALIZER)
    find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc)

    add_executable(tracker_visualizer
        tools/tracker_visualizer.cpp
        tools/replay_input.cpp
    )
    target_include_directories(tracker_visualizer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(tracker_visualizer PRIVATE tracking_core ${OpenCV_LIBS})
endif()

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(tracker_tests
        tests/test_public_api.cpp
        tests/test_geometry.cpp
        tests/test_hungarian.cpp
        tests/test_kalman.cpp
        tests/test_ocsort_association.cpp
        tests/test_ocsort_tracker.cpp
        tests/test_tracker.cpp
    )
    target_include_directories(tracker_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(tracker_tests PRIVATE tracking_core GTest::gtest_main Eigen3::Eigen)

    include(GoogleTest)
    gtest_discover_tests(tracker_tests)

    add_test(
        NAME TrackerReplaySuccess
        COMMAND ${CMAKE_COMMAND}
            -DTRACKER_REPLAY=$<TARGET_FILE:tracker_replay>
            -DTESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/testdata/replay
            -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_replay_success.cmake
    )
    add_test(
        NAME TrackerReplayFailures
        COMMAND ${CMAKE_COMMAND}
            -DTRACKER_REPLAY=$<TARGET_FILE:tracker_replay>
            -DTESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/testdata/replay
            -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_replay_failures.cmake
    )

    if(BUILD_TRACKER_VISUALIZER)
        add_test(
            NAME TrackerVisualizerMissingImage
            COMMAND ${CMAKE_COMMAND}
                -DTRACKER_VISUALIZER=$<TARGET_FILE:tracker_visualizer>
                -DFRAMES=${CMAKE_CURRENT_SOURCE_DIR}/testdata/replay/frames_empty.csv
                -DDETECTIONS=${CMAKE_CURRENT_SOURCE_DIR}/testdata/replay/empty_detections.csv
                -DIMAGE_DIR=${CMAKE_CURRENT_BINARY_DIR}/visualizer-missing-images
                -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_visualizer_missing_image.cmake
        )
        add_test(
            NAME TrackerVisualizerAutoPlay
            COMMAND ${CMAKE_COMMAND}
                -DTRACKER_VISUALIZER=$<TARGET_FILE:tracker_visualizer>
                -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/visualizer-autoplay
                -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_visualizer_autoplay.cmake
        )
    endif()
endif()
