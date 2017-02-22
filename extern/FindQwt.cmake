find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

find_path(Qwt_INCLUDE_DIR qwt)
find_library(Qwt_LIBRARY qwt qwtmathml)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qwt DEFAULT_MSG Qwt_LIBRARY Qwt_INCLUDE_DIR)
mark_as_advanced(Qwt_INCLUDE_DIR Qwt_LIBRARY)
