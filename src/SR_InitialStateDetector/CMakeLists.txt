# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME sr_initialStateDetector)

adtf_add_filter(${FILTER_NAME}
	../misc/PinWrapper.cpp
	InitialStateDetector.h
	InitialStateDetector.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
