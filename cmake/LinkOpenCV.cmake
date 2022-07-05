####################
# LinkOpenCV.cmake #
####################

TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${OpenCV_LIBS})

IF(WIN32)
  STRING(REPLACE "\\" "/" _OpenCV_LIB_PATH "${_OpenCV_LIB_PATH}")
  FILE(GLOB OpenCV_RUNTIMELIBS "${_OpenCV_LIB_PATH}/*.dll")
ENDIF()

FOREACH(RUNTIMELIB ${OpenCV_RUNTIMELIBS})
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RUNTIMELIB} "$<TARGET_FILE_DIR:${targetname}>")
ENDFOREACH()
