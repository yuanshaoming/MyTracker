function(expect_failure frames detections expected_location)
    set(output "${OUTPUT_DIR}/tracker_replay_invalid.csv")
    file(REMOVE "${output}")
    execute_process(
        COMMAND "${TRACKER_REPLAY}" --frames "${frames}" --detections "${detections}" --output "${output}"
        RESULT_VARIABLE result
        ERROR_VARIABLE error
    )
    if(result EQUAL 0)
        message(FATAL_ERROR "invalid replay unexpectedly succeeded for ${frames} and ${detections}")
    endif()
    string(FIND "${error}" "${expected_location}" location_index)
    if(location_index EQUAL -1)
        message(FATAL_ERROR "failure did not include ${expected_location}: ${error}")
    endif()
    if(EXISTS "${output}")
        message(FATAL_ERROR "invalid input created an output file")
    endif()
endfunction()

set(DATA "${TESTDATA_DIR}")
expect_failure("${DATA}/invalid_column_frames.csv" "${DATA}/empty_detections.csv" "invalid_column_frames.csv:2")
expect_failure("${DATA}/invalid_number_frames.csv" "${DATA}/empty_detections.csv" "invalid_number_frames.csv:2")
expect_failure("${DATA}/duplicate_timestamp_frames.csv" "${DATA}/empty_detections.csv" "duplicate_timestamp_frames.csv:3")
expect_failure("${DATA}/unordered_frames.csv" "${DATA}/empty_detections.csv" "unordered_frames.csv:3")
expect_failure("${DATA}/frames_empty.csv" "${DATA}/invalid_column_detections.csv" "invalid_column_detections.csv:2")
expect_failure("${DATA}/frames_empty.csv" "${DATA}/invalid_number_detections.csv" "invalid_number_detections.csv:2")
expect_failure("${DATA}/frames_empty.csv" "${DATA}/invalid_box_detections.csv" "invalid_box_detections.csv:2")
expect_failure("${DATA}/frames_empty.csv" "${DATA}/unknown_frame_detections.csv" "unknown_frame_detections.csv:2")
expect_failure("${DATA}/frames_two.csv" "${DATA}/unordered_detections.csv" "unordered_detections.csv:3")

set(INVALID_TRACKER_OUTPUT "${OUTPUT_DIR}/tracker_replay_invalid_tracker.csv")
file(REMOVE "${INVALID_TRACKER_OUTPUT}")
execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${DATA}/frames.csv" --detections "${DATA}/detections.csv" --output "${INVALID_TRACKER_OUTPUT}" --tracker invalid
    RESULT_VARIABLE INVALID_TRACKER_RESULT
    ERROR_VARIABLE INVALID_TRACKER_ERROR
)
if(INVALID_TRACKER_RESULT EQUAL 0)
    message(FATAL_ERROR "invalid tracker unexpectedly succeeded")
endif()
if(EXISTS "${INVALID_TRACKER_OUTPUT}")
    message(FATAL_ERROR "invalid tracker created an output file")
endif()
