##################
# UseEigen.cmake #
##################

FIND_PATH(EIGEN_INCLUDE_DIR eigen3.pc.in HINTS "$ENV{SMGLIB_EIGEN_INCLUDE_DIR}")
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN_INCLUDE_DIR})
