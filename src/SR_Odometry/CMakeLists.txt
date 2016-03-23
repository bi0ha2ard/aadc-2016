# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME SR_Odometry)

adtf_add_filter(${FILTER_NAME}
        SR_Odometry.h
        SR_Odometry.cpp
        SR_Calculations.h
        SR_Calculations.cpp
        EulerAngles.hpp
        EulerAngles.cpp
        QuatTypes.h
        ../misc/PinWrapper.h
        ../misc/PinWrapper.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
