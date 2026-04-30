cmake_minimum_required(VERSION 3.8)
project(teensy_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED )
find_package(Threads REQUIRED)
# find_package(libgpiodcxx REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${dlib_INCLUDE_DIR} /usr/include)
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 and 4
if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
   message(STATUS "Is a RASPBERRY; CPU=${CPU} (Pi3=armv7l, pi4=aarch64)")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
   set(GPIO_VERSION "src/sgpiod2.cpp") # newer version than in Kubuntu
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message(STATUS "Not a RASPBERRY; CPU=${CPU}")
   find_package(PahoMqttCpp REQUIRED)
   set(GPIO_VERSION "src/sgpiod.cpp") # old version - but not used on this platform
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

add_executable(teensy_interface
      src/cmixer.cpp
      src/cmotor.cpp
      src/cservo.cpp
      ${GPIO_VERSION}
      src/main.cpp
      src/mjoy.cpp
      src/mvelocity.cpp
      src/scurrent.cpp
      src/sdistforce.cpp
      src/sedge.cpp
      src/sencoder.cpp
      src/scurrent.cpp
      src/simu.cpp
      src/sjoylogitech.cpp
      src/srobot.cpp
      src/steensy.cpp
      src/umqtt.cpp
      src/umqttin.cpp
      src/upid.cpp
      src/uservice.cpp
      src/utime.cpp
      )

if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
  # target_link_libraries(teensy_interface ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} PahoMqttCpp::paho-mqttpp3 paho-mqtt3c readline gpiod rt)
  target_link_libraries(teensy_interface ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} paho-mqtt3c readline gpiod rt)
else()
  target_link_libraries(teensy_interface ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} paho-mqttpp3 paho-mqtt3as paho-mqtt3c readline gpiod)
  #target_link_libraries(teensy_interface ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} PahoMqttCpp::paho-mqttpp3 paho-mqtt3c readline gpiod)
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
