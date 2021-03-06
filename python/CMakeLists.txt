
# Find the wifibroadcast paths
find_path(WFB_INCLUDE_DIR wifibroadcast/fec.hh)

# Find the python library.
find_package(PythonLibs REQUIRED)
include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${PROJECT_SOURCE_DIR}/python
  ${WFB_INCLUDE_DIR}/wifibroadcast
  ${PROJECT_SOURCE_DIR}/modules
  )

# Compile with cython
include(UseCython)

# Build the FEC cython interface
cython_add_module(fec)
link_directories(${PROJECT_BINARY_DIR})
target_link_libraries(fec ${PYTHON_LIBRARY} ${Extra_Libraries} wifibroadcast)

# Build the py_v4l2 cython interface
cython_add_module(py_v4l2)
target_link_libraries(py_v4l2 ${PYTHON_LIBRARY} ${Extra_Libraries} v4l2)

# Build the mavlink cython interface
cython_add_module(mavlink)
target_link_libraries(mavlink ${PYTHON_LIBRARY} ${Extra_Libraries})

# Build the mavlink cython interface
#cython_add_module(msp)
#target_link_libraries(msp ${PYTHON_LIBRARY} ${Extra_Libraries})

# Install the cython targets.
install(TARGETS fec py_v4l2 mavlink DESTINATION ${RELATIVE_PYTHON_DIR})

# Install the python executable scripts
install(PROGRAMS openhd_camera DESTINATION ${BinDir})
install(PROGRAMS openhd_telemetry DESTINATION ${BinDir})
install(PROGRAMS openhd_controller DESTINATION ${BinDir})
install(PROGRAMS openhd_mavlink_air DESTINATION ${BinDir})
install(PROGRAMS openhd_mavlink_ground DESTINATION ${BinDir})
install(PROGRAMS openhd_mavlink_transmitter DESTINATION ${BinDir})
install(PROGRAMS openhd_crsf_air DESTINATION ${BinDir})

# Install the python libraries
install(FILES
  format_as_table.py
  camera.py
  telemetry.py
  MultiWii.py
  MavlinkTelemetry.py
  transmitter.py
  video_player.py
  crsf.py
  DESTINATION ${RELATIVE_PYTHON_DIR})
