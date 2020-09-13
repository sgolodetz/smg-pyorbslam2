#####################
# UseORBSLAM2.cmake #
#####################

#SET(spaint_ROOT_DIR "${PROJECT_SOURCE_DIR}/../spaint" CACHE PATH "The spaint root directory")
#SET(spaint_MODULES_DIR "${spaint_ROOT_DIR}/modules")
#SET(spaint_LIB_DIR "${spaint_ROOT_DIR}/build/lib")

SET(ORBSLAM2_ROOT_DIR "${PROJECT_SOURCE_DIR}/../orbslam" CACHE PATH "The ORB-SLAM root directory")

#SET(spaint_itmx_INCLUDE_DIR "${spaint_MODULES_DIR}/itmx/include")
#SET(spaint_orx_INCLUDE_DIR "${spaint_MODULES_DIR}/orx/include")
#SET(spaint_rigging_INCLUDE_DIR "${spaint_MODULES_DIR}/rigging/include")
#SET(spaint_tvgutil_INCLUDE_DIR "${spaint_MODULES_DIR}/tvgutil/include")
#...

#INCLUDE_DIRECTORIES(${spaint_itmx_INCLUDE_DIR} ${spaint_orx_INCLUDE_DIR} ${spaint_rigging_INCLUDE_DIR} ${spaint_tvgutil_INCLUDE_DIR})
INCLUDE_DIRECTORIES("${ORBSLAM2_ROOT_DIR}" "${ORBSLAM2_ROOT_DIR}/include" "${ORBSLAM2_ROOT_DIR}/Thirdparty/Pangolin/include" "${ORBSLAM2_ROOT_DIR}/Thirdparty/Pangolin/build/src/include")

#FIND_LIBRARY(spaint_itmx_LIBRARY_DEBUG itmx_d HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_orx_LIBRARY_DEBUG orx_d HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_rigging_LIBRARY_DEBUG rigging_d HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_tvgutil_LIBRARY_DEBUG tvgutil_d HINTS ${spaint_LIB_DIR})
#...

#FIND_LIBRARY(spaint_itmx_LIBRARY_RELEASE itmx HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_orx_LIBRARY_RELEASE orx HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_rigging_LIBRARY_RELEASE rigging HINTS ${spaint_LIB_DIR})
#FIND_LIBRARY(spaint_tvgutil_LIBRARY_RELEASE tvgutil HINTS ${spaint_LIB_DIR})
#...

#SET(spaint_itmx_LIBRARY debug ${spaint_itmx_LIBRARY_DEBUG} optimized ${spaint_itmx_LIBRARY_RELEASE})
#SET(spaint_orx_LIBRARY debug ${spaint_orx_LIBRARY_DEBUG} optimized ${spaint_orx_LIBRARY_RELEASE})
#SET(spaint_rigging_LIBRARY debug ${spaint_rigging_LIBRARY_DEBUG} optimized ${spaint_rigging_LIBRARY_RELEASE})
#SET(spaint_tvgutil_LIBRARY debug ${spaint_tvgutil_LIBRARY_DEBUG} optimized ${spaint_tvgutil_LIBRARY_RELEASE})
#...

#SET(spaint_LIBRARIES ${spaint_itmx_LIBRARY} ${spaint_orx_LIBRARY} ${spaint_rigging_LIBRARY} ${spaint_tvgutil_LIBRARY})
