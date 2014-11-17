# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

if(${PLATFORM} MATCHES "Arduino")
  generate_arduino_firmware(test_bcomms_ino
      SRCS test_arduino.cpp
      LIBS blob_comms
      PORT /dev/ttyACM0
      BOARD leonardo
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
  add_executable(test_bcomms_lnx test_linux) # build executable
  target_link_libraries(test_bcomms_lnx blob_comms) # link libraries
endif(${PLATFORM} MATCHES "Arduino")

