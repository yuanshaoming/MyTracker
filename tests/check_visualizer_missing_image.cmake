execute_process(
    COMMAND "${TRACKER_VISUALIZER}" --frames "${FRAMES}" --detections "${DETECTIONS}" --images "${IMAGE_DIR}"
    RESULT_VARIABLE result
    ERROR_VARIABLE error
)

if(result EQUAL 0)
    message(FATAL_ERROR "visualizer unexpectedly accepted a missing image")
endif()
string(FIND "${error}" "frame 0: missing image" error_index)
if(error_index EQUAL -1)
    message(FATAL_ERROR "missing-image failure did not name frame 0: ${error}")
endif()
