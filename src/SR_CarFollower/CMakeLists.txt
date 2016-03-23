# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
set(FILTER_NAME sr_carfollower)

adtf_add_filter(${FILTER_NAME}
	SR_CarFollower.h
	SR_CarFollower.cpp
        ../misc/SR_Logger.h
        ../misc/SR_Logger.cpp
        ../misc/PinWrapper.h
        ../misc/PinWrapper.cpp
        ../misc/LineHelpers.h
        ../misc/LineHelpers.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
