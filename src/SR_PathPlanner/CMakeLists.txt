# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
# declare our filter
set(FILTER_NAME sr_pathPlanner)

adtf_add_filter(${FILTER_NAME}
	../misc/PinWrapper.h
	../misc/PinWrapper.cpp
	../misc/SR_Logger.h
	../misc/SR_Logger.cpp
	../misc/LineHelpers.h
	../misc/LineHelpers.cpp
	../misc/CrossingHelpers.h
	../misc/CrossingHelpers.cpp
	../misc/OdometryHelpers.h
	../misc/OdometryHelpers.cpp
	PathPlanner.h
	PathPlanner.cpp
	DrivingPath.h
	DrivingPath.cpp
)
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
