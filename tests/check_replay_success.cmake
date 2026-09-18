set(FRAMES "${TESTDATA_DIR}/frames.csv")
set(DETECTIONS "${TESTDATA_DIR}/detections.csv")
set(EMPTY_DETECTIONS "${TESTDATA_DIR}/empty_detections.csv")
set(OUTPUT_ONE "${OUTPUT_DIR}/tracker_replay_one.csv")
set(OUTPUT_TWO "${OUTPUT_DIR}/tracker_replay_two.csv")
set(OUTPUT_SORT "${OUTPUT_DIR}/tracker_replay_sort.csv")
set(OUTPUT_OCSORT "${OUTPUT_DIR}/tracker_replay_ocsort.csv")
set(EMPTY_OUTPUT "${OUTPUT_DIR}/tracker_replay_empty.csv")

file(REMOVE "${OUTPUT_ONE}" "${OUTPUT_TWO}" "${OUTPUT_SORT}" "${OUTPUT_OCSORT}" "${EMPTY_OUTPUT}")
execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${FRAMES}" --detections "${DETECTIONS}" --output "${OUTPUT_ONE}"
    RESULT_VARIABLE RESULT_ONE
    ERROR_VARIABLE ERROR_ONE
)
if(NOT RESULT_ONE EQUAL 0)
    message(FATAL_ERROR "first replay failed: ${ERROR_ONE}")
endif()
execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${FRAMES}" --detections "${DETECTIONS}" --output "${OUTPUT_TWO}"
    RESULT_VARIABLE RESULT_TWO
    ERROR_VARIABLE ERROR_TWO
)
if(NOT RESULT_TWO EQUAL 0)
    message(FATAL_ERROR "second replay failed: ${ERROR_TWO}")
endif()
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUTPUT_ONE}" "${OUTPUT_TWO}"
    RESULT_VARIABLE COMPARE_RESULT
)
if(NOT COMPARE_RESULT EQUAL 0)
    message(FATAL_ERROR "replay output is not byte-for-byte deterministic")
endif()
execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${FRAMES}" --detections "${DETECTIONS}" --output "${OUTPUT_SORT}" --tracker sort
    RESULT_VARIABLE SORT_RESULT
    ERROR_VARIABLE SORT_ERROR
)
if(NOT SORT_RESULT EQUAL 0)
    message(FATAL_ERROR "explicit SORT replay failed: ${SORT_ERROR}")
endif()
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUTPUT_ONE}" "${OUTPUT_SORT}"
    RESULT_VARIABLE SORT_COMPARE_RESULT
)
if(NOT SORT_COMPARE_RESULT EQUAL 0)
    message(FATAL_ERROR "explicit SORT changed the default replay output")
endif()
execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${FRAMES}" --detections "${DETECTIONS}" --output "${OUTPUT_OCSORT}" --tracker ocsort
    RESULT_VARIABLE OCSORT_RESULT
    ERROR_VARIABLE OCSORT_ERROR
)
if(NOT OCSORT_RESULT EQUAL 0)
    message(FATAL_ERROR "OC-SORT replay failed: ${OCSORT_ERROR}")
endif()
file(READ "${OUTPUT_OCSORT}" OCSORT_TRACKS)
if(NOT OCSORT_TRACKS MATCHES "frame_id,timestamp_ms,track_id")
    message(FATAL_ERROR "OC-SORT replay output has an unexpected header")
endif()

file(READ "${OUTPUT_ONE}" TRACKS)
string(FIND "${TRACKS}" "frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames" HEADER_INDEX)
if(NOT HEADER_INDEX EQUAL 0)
    message(FATAL_ERROR "tracks output has an unexpected header")
endif()
if(NOT TRACKS MATCHES "3,625,1,[^\n]*,Lost,[^\n]*,4,1")
    message(FATAL_ERROR "short miss did not emit Track 1 as Lost with age 4 and lost_frames 1")
endif()
if(NOT TRACKS MATCHES "4,750,1,[^\n]*,Confirmed,[^\n]*,5,0")
    message(FATAL_ERROR "Track 1 did not recover with its original ID")
endif()
if(NOT TRACKS MATCHES "6,1000,1,")
    message(FATAL_ERROR "Track 1 is not stable through the sample sequence")
endif()
if(NOT TRACKS MATCHES "6,1000,2,")
    message(FATAL_ERROR "far second person is missing from the sample output")
endif()

execute_process(
    COMMAND "${TRACKER_REPLAY}" --frames "${TESTDATA_DIR}/frames_empty.csv" --detections "${EMPTY_DETECTIONS}" --output "${EMPTY_OUTPUT}"
    RESULT_VARIABLE EMPTY_RESULT
    ERROR_VARIABLE EMPTY_ERROR
)
if(NOT EMPTY_RESULT EQUAL 0)
    message(FATAL_ERROR "empty-detection replay failed: ${EMPTY_ERROR}")
endif()
file(READ "${EMPTY_OUTPUT}" EMPTY_TRACKS)
if(NOT EMPTY_TRACKS STREQUAL "frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames\n")
    message(FATAL_ERROR "empty-detection replay should contain only the header")
endif()
