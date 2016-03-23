# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME SR_SignalClamper)

adtf_add_filter(${FILTER_NAME}
	SR_SignalClamper.h
	SR_SignalClamper.cpp
	../misc/PinWrapper.h
	../misc/PinWrapper.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
