# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
# declare our filter
set(FILTER_NAME SR_ParkingFinder)

adtf_add_filter(${FILTER_NAME}
	../misc/PinWrapper.cpp
	../misc/PinWrapper.h
        ../misc/SR_Logger.cpp
        ../misc/SR_Logger.h
	SR_ParkingFinder.h
	SR_ParkingFinder.cpp
)
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
