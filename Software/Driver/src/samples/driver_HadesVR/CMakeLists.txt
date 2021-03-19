set(TARGET_NAME driver_sample)

add_library(${TARGET_NAME} SHARED
  driverlog.cpp
  driverlog.h
  driver_sample.cpp
  dataHandler.cpp
  dataHandler.h
)

add_definitions(-DDRIVER_SAMPLE_EXPORTS)

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

setTargetOutputDirectory(${TARGET_NAME})
