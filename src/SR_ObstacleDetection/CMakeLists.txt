# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
# declare our filter
set(FILTER_NAME sr_obstacledetection)

adtf_add_filter(${FILTER_NAME}
	SR_ObstacleDetection.h
	SR_ObstacleDetection.cpp
        ../misc/PinWrapper.cpp
        ../misc/PinWrapper.h
)

# Specify where it should be installed to
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
