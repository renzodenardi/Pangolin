###############################################################################
# Find Arducam
#
# This sets the following variables:
# ArduCam_FOUND - True if ArduCam was found.
# ArduCam_INCLUDE_DIRS - Directories containing the ArduCam include files.
# ArduCam_LIBRARIES - Libraries needed to use ArduCam.

find_path(
    ArduCam_INCLUDE_DIR ArduCamLib.h
    PATHS
        /usr/include
        /user/include
	PATH_SUFFIXES ArduCam
)

find_library(
    ArduCamLib_LIBRARY
    NAMES ArduCamLib
    PATHS
        /usr/lib
        /user/lib
    PATH_SUFFIXES ${ARDUCAM_PATH_SUFFIXES}
)

find_library(
    ArduCamConfigParser_LIBRARY
    NAMES arducam_config_parser
    PATHS
        /usr/lib
        /user/lib
    PATH_SUFFIXES ${ARDUCAM_PATH_SUFFIXES}
)

find_library(libusb-1.0_LIBRARY
    NAMES libusb-1.0.so
    HINTS /usr/lib /usr/local/lib /lib64)

set(ArduCam_INCLUDE_DIRS ${ArduCam_INCLUDE_DIR})
set(ArduCam_LIBRARY "${ArduCamLib_LIBRARY}" "${ArduCamConfigParser_LIBRARY}" "${libusb-1.0_LIBRARY}")
set(ArduCam_LIBRARIES ${ArduCam_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ArduCam
  FOUND_VAR ArduCam_FOUND
  REQUIRED_VARS ArduCam_LIBRARY ArduCam_INCLUDE_DIR
)
