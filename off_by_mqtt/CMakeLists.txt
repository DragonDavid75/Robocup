cmake_minimum_required(VERSION 3.8)
project(off_by_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS} ${dlib_INCLUDE_DIR})
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 only ??
if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
   message("# Is a RASPBERRY CPU=${CPU} (Pi3=armv7l, pi4=aarch64)")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("# Not a RASPBERRY ${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")


add_executable(off_by_mqtt
      src/main.cpp
      src/umqtt.cpp
      src/uservice.cpp
      src/utime.cpp
      )

if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
  target_link_libraries(off_by_mqtt ${CMAKE_THREAD_LIBS_INIT} paho-mqtt3c readline gpiod rt gpiodcxx)
  #target_link_libraries(off_by_mqtt ${CMAKE_THREAD_LIBS_INIT} paho-mqtt3c readline gpiod rt gpiodcxx)
else()
  target_link_libraries(off_by_mqtt ${CMAKE_THREAD_LIBS_INIT} paho-mqttpp3 paho-mqtt3as paho-mqtt3c readline gpiod gpiodcxx)
endif()

