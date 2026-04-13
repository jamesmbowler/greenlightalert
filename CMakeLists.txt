cmake_minimum_required(VERSION 3.16)

project(esp32_camera_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EI_ARDUINO_LIB_DIR "/Users/jamesbowler/Documents/Arduino/libraries/jbowler-project-1_inferencing/src")

file(GLOB_RECURSE EI_TFLM_SOURCES
  "${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/tensorflow/**/*.cc"
  "${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/tensorflow/**/*.cpp"
  "${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/tensorflow/**/*.c"
)

find_package(OpenCV REQUIRED)

add_executable(esp32_camera_viewer main.cpp)
target_link_libraries(esp32_camera_viewer PRIVATE ${OpenCV_LIBS})
target_include_directories(esp32_camera_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(esp32_serial_camera_viewer serial_viewer.cpp)
target_link_libraries(esp32_serial_camera_viewer PRIVATE ${OpenCV_LIBS})
target_include_directories(esp32_serial_camera_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(serial_saver serial_saver.cpp)
target_link_libraries(serial_saver PRIVATE ${OpenCV_LIBS})
target_include_directories(serial_saver PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(test_find_light
  tests/find_light/test_find_light.cpp
  arduino/esp32_serial_camera_sender/traffic_light_detector_shared.cpp
)
target_link_libraries(test_find_light PRIVATE ${OpenCV_LIBS})
target_include_directories(test_find_light PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(red_light_box_center_test
  tests/red_light_box_center/red_light_box_center.cpp
  arduino/esp32_serial_camera_sender/red_light_box_shared.cpp
)
target_link_libraries(red_light_box_center_test PRIVATE ${OpenCV_LIBS})
target_include_directories(red_light_box_center_test PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(end_to_end
  tests/end_to_end/end_to_end.cpp
  arduino/esp32_serial_camera_sender/red_light_box_shared.cpp
  ${EI_ARDUINO_LIB_DIR}/tflite-model/tflite_learn_790184_19_compiled.cpp
  ${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/dsp/memory.cpp
  ${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/dsp/kissfft/kiss_fft.cpp
  ${EI_ARDUINO_LIB_DIR}/edge-impulse-sdk/dsp/kissfft/kiss_fftr.cpp
  ${EI_TFLM_SOURCES}
)
target_link_libraries(end_to_end PRIVATE ${OpenCV_LIBS})
target_include_directories(end_to_end PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/end_to_end
  ${EI_ARDUINO_LIB_DIR}
)
