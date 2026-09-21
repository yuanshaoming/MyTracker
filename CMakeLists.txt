cmake_minimum_required(VERSION 3.16)

project(MyTracker LANGUAGES CXX)

include(CTest)

option(BUILD_TRACKER_VISUALIZER "Build the OpenCV image-sequence visualizer" OFF)

if(BUILD_TRACKER_VISUALIZER)
    message(FATAL_ERROR "tracker_visualizer is unavailable on the C++14/VS2015 compatibility branch")
endif()

find_package(Eigen3 3.3.9 EXACT REQUIRED NO_MODULE)

add_library(tracking_core STATIC
    src/geometry.cpp
    src/hungarian.cpp
    src/kalman_box_tracker.cpp
    src/ocsort_association.cpp
    src/ocsort_reupdate.cpp
    src/ocsort_tracker.cpp
    src/sort_tracker.cpp
)

target_compile_features(tracking_core PUBLIC cxx_std_14)
target_include_directories(tracking_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tracking_core PRIVATE Eigen3::Eigen)
set_target_properties(tracking_core PROPERTIES OUTPUT_NAME tracking_core_vs2015)

add_executable(tracker_replay
    tools/tracker_replay.cpp
    tools/replay_input.cpp
)
target_include_directories(tracker_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(tracker_replay PRIVATE tracking_core)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    if(TARGET GTest::gtest AND TARGET GTest::gtest_main)
        set(TRACKER_GTEST_TARGETS GTest::gtest_main)
    elseif(TARGET GTest::GTest AND TARGET GTest::Main)
        set(TRACKER_GTEST_TARGETS GTest::GTest GTest::Main)
    else()
        message(FATAL_ERROR "GoogleTest package does not provide gtest and gtest_main targets")
    endif()

    if(MSVC AND MSVC_VERSION LESS 1910)
        if(GTest_VERSION VERSION_GREATER_EQUAL "1.17" OR GTEST_VERSION VERSION_GREATER_EQUAL "1.17")
            message(FATAL_ERROR "This GoogleTest version requires C++17; use a v140-compatible version")
        endif()
        foreach(gtest_target IN LISTS TRACKER_GTEST_TARGETS)
            get_target_property(gtest_features ${gtest_target} INTERFACE_COMPILE_FEATURES)
            if(gtest_features MATCHES "cxx_std_(17|20|23|26)")
                message(FATAL_ERROR "This GoogleTest package requires C++17; use a v140-compatible version")
            endif()
        endforeach()
    endif()

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
    target_link_libraries(tracker_tests PRIVATE tracking_core ${TRACKER_GTEST_TARGETS} Eigen3::Eigen)

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
endif()
