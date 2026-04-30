/* #***************************************************************************
 #*   Copyright (C) 2025 by DTU
 #*   jcan@dtu.dk
 #*
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */


#include <stdio.h>
#include <signal.h>
#include "CLI/CLI.hpp"
#include <filesystem>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <netpacket/packet.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "uservice.h"
#include "umqtt.h"
#include <mutex>

#define REV "$Id: uservice.cpp 918 2025-01-20 13:21:16Z jcan $"
// define the service class
UService service;
// make a configuration structure

void signal_callback_handler(int signum)
{ // called when pressing ctrl-C
  UTime t("now");
  const int MSL = 100;
  char s[MSL];
  printf("# at %s Interrupted by signal %d\n", t.getDateTimeAsString(s, true), signum);
  service.terminate();
  exit(signum);
}

bool UService::setup(int argc,char **argv)
{ // Interrupt signal handler
  signal(SIGINT, signal_callback_handler);
  signal(SIGQUIT, signal_callback_handler); // 3
  signal(SIGHUP, signal_callback_handler); // 1
  signal(SIGPWR, signal_callback_handler); // 30
  signal(SIGTERM, signal_callback_handler); // 15 (pkill default)
  //
  CLI::App cli{"off_by_mqtt app"};
  // cli.add_option("-d,--device", service.usbDev, "USB device name for Teensy (default is /dev/ttyACM0)");
  // add reply to version request
  bool version{false};
  cli.add_flag("-v,--version", version, "Latest SVN version (for uservice.cpp)");
  cli.allow_windows_style_options();
  cli.add_option("-m,--mqtthost", mqtthost, "Set mosquitto host (default is 'localhost') see /etc/hosts");
  cli.add_option("-n,--name", mqttclient, "Set mosquitto client name (default is 'off_by_mqtt') no two alike.");
  CLI11_PARSE(cli, argc, argv);
  //
  bool theEnd = false;
  //
  if (version)
  {
    printf("off_by_mqtt SVN version%s\n", getVersionString().c_str());
    theEnd = true;
  }
  if (not theEnd)
  { // open the main data source/destination
    const int MSL = 100;
    char s[MSL];
    UTime t("now");
    printf("# %s UService::setup: Started\n", t.getForFilename(s, true));
    mqtt.setup();
    setupComplete = true;
    usleep(2000);
    //
  }
  return not theEnd;
}


bool UService::mqttDecode(const char* topic, const char* msg, UTime t)
{ // decode messages from Teensy
  bool used = true;
  if (strcmp(topic, "robobot/cmd/shutdown") == 0)
  { // wait for others to react
    // toLog(topic);
    printf("# UService::mqttDecode: got shutdown topic: %s, msg %s; shutdown in 700ms\n", topic, msg);
    usleep(3000000);
    // toLog(topic);
    printf("# UService::mqttDecode: got shutdown topic: %s, msg %s\n", topic, msg);
    std::string aa = exec("shutdown now");
    printf("# shutdown returned %s\n", aa.c_str());
    usleep(100000);
    const int MSL = 100;
    char s[MSL];
    printf("# %s UService::mqttDecode: got %s %s shutting down\n", t.getForFilename(s, true), topic, msg);
    terminate();
  }
  else
  {
    used = false;
    printf("# UService::mqttDecode: got unused topic: %s, msg %s\n", topic, msg);
  }
  return used;
}


void UService::terminate()
{ // Terminate modules (especially threads and log files)
  if (terminating or not setupComplete)
    return;
  printf("# --------- terminating -----------\n");
  terminating = true;
  stop = true; // stop all threads, when finished current activity
  // terminate sensors before Teensy
  mqtt.terminate();
  // service must be the last to close
  printf("# end of service terminate\n");
}

std::string UService::getVersionString()
{
  // #define REV "$Id: uservice.cpp 918 2025-01-20 13:21:16Z jcan $"
  std::string ver = REV;
  int n1 = ver.find(' ', 10);
  int n2 = ver.rfind("Z ");
  std::string part = ver.substr(n1, n2-n1);
  return part;
}


std::string UService::exec(std::string cmd)
{
  // toLog(cmd.c_str());
  std::array<char, 100> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose) > pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  // toLog(result.c_str());
  return result;
}

int UService::isThisProcessRunning(std::string name)
{
  std::string cmd = "pgrep " + name;
  std::string s = exec(cmd);
  // printf("# isThisProcessRunning found '%s' times %s using %s\n", s.c_str(), name.c_str(), cmd.c_str());
  int cnt = 0;
  int pos = 0;
  while (true)
  {
    auto n = s.find('\n', pos);
    if (n == std::string::npos)
      break;
    else
    {
      pos = n+1;
      cnt ++;
    }
  }
  return cnt;
}



void UService::shutdownRequest()
{
  UTime t("now");
  mqtt.publish("robobot/cmd/shutdown","GPIO button", t, 1);
}
