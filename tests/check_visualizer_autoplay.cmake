set(FRAMES "${OUTPUT_DIR}/frames.csv")
set(DETECTIONS "${OUTPUT_DIR}/detections.csv")
set(IMAGES "${OUTPUT_DIR}/images")

file(MAKE_DIRECTORY "${IMAGES}")
file(WRITE "${FRAMES}" "frame_id,timestamp_ms\n0,0\n1,125\n")
file(WRITE "${DETECTIONS}" "frame_id,x1,y1,x2,y2,confidence,class_id\n")
file(WRITE "${IMAGES}/000000.jpg" "P3\n2 2\n255\n0 0 0 0 0 0 0 0 0 0 0 0\n")
file(WRITE "${IMAGES}/000001.jpg" "P3\n2 2\n255\n255 255 255 255 255 255 255 255 255 255 255 255\n")

execute_process(
    COMMAND "${TRACKER_VISUALIZER}" --frames "${FRAMES}" --detections "${DETECTIONS}" --images "${IMAGES}" --auto-play
    RESULT_VARIABLE result
    ERROR_VARIABLE error
)

if(NOT result EQUAL 0)
    message(FATAL_ERROR "auto-play did not process consecutive timestamps: ${error}")
endif()

execute_process(
    COMMAND "${TRACKER_VISUALIZER}" --frames "${FRAMES}" --detections "${DETECTIONS}" --images "${IMAGES}" --tracker ocsort --auto-play
    RESULT_VARIABLE ocsort_result
    ERROR_VARIABLE ocsort_error
)
if(NOT ocsort_result EQUAL 0)
    message(FATAL_ERROR "OC-SORT auto-play failed: ${ocsort_error}")
endif()
