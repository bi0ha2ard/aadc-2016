# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
# declare our filter
set(FILTER_NAME sr_imageProcessing)

adtf_add_filter(${FILTER_NAME}
	IPM.cpp
	IPM.h
	../misc/PinWrapper.h
	../misc/PinWrapper.cpp
	../misc/SR_Logger.h
	../misc/SR_Logger.cpp
	SR_ImageProcessing.h
	SR_ImageProcessing.cpp
	ProcessingThread.h
	ProcessingThread.cpp
	../misc/FastRot.h
	../misc/OdometryHelpers.h
	../misc/OdometryHelpers.cpp
	../misc/CrossingHelpers.h
	../misc/CrossingHelpers.cpp

	#debug
	../SR_PathPlanner/DrivingPath.h
	../SR_PathPlanner/DrivingPath.cpp
	../misc/LineHelpers.h
	../misc/LineHelpers.cpp
)
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
