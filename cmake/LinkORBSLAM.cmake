#####################
# LinkORBSLAM.cmake #
#####################

TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${ORBSLAM_LIBRARIES})

IF(MediaFoundation_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${MediaFoundation_LIBRARIES})
ENDIF()

IF(TIFF_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${TIFF_LIBRARY})
ENDIF()

IF(zstd_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${zstd_LIBRARY})
ENDIF()

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Pangolin_ROOT_DIR}/install/lib/glew.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Pangolin_ROOT_DIR}/install/lib/glewd.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
