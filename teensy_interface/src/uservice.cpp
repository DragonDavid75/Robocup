/* #***************************************************************************
 #*   Copyright (C) 2023 by DTU
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
//#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include "CLI/CLI.hpp"
#include <filesystem>
#include <string.h>
#include <future>

#include "uini.h"
#include "cmotor.h"
#include "cmixer.h"
#include "cservo.h"
#include "mjoy.h"
#include "mvelocity.h"
#include "scurrent.h"
#include "sdistforce.h"
#include "sedge.h"
#include "sencoder.h"
#include "sgpiod.h"
#include "simu.h"
#include "sjoylogitech.h"
#include "srobot.h"
#include "steensy.h"
#include "umqtt.h"
#include "umqttin.h"
#include "uservice.h"

#define REV "$Id: uservice.cpp 1234 2026-02-01 08:46:32Z jcan $"
// define the service class
UService service;
// make a configuration structure
mINI::INIStructure ini;

void signal_callback_handler(int signum)
{ // called when pressing ctrl-C
  if (service.stop)
    cout << "\nCaught signal " << signum << endl << "# Press quit and enter to finish keyboard read\n";
  else
  {
    printf("# signal_callback_handler: setting stopNowRequest to true\r\n");
    service.stopNowRequest = true;
  }
  // service.terminate();
  // exit(signum);
}

bool UService::setup(int argc,char **argv)
{ // Interrupt signal handler for most common signals
  signal(SIGINT, signal_callback_handler); // 2 normal ctrl-C
//   signal(SIGKILL, signal_callback_handler); // 9
  signal(SIGQUIT, signal_callback_handler); // 3
  signal(SIGHUP, signal_callback_handler); // 1
  signal(SIGPWR, signal_callback_handler); // 30
  signal(SIGTERM, signal_callback_handler); // 15 (pkill default)
  //
  CLI::App cli{"ROBOBOT app"};
  // cli.add_option("-d,--device", service.usbDev, "USB device name for Teensy (default is /dev/ttyACM0)");
  // add reply to version request
  bool version{false};
  cli.add_flag("-v,--version", version, "Compiled SVN version (for uservice.cpp)");
  cli.add_flag("-d,--daemon", asDaemon, "Do not listen to the keyboard (daemon mode)");
  bool nologging = false;
  cli.add_flag("-l,--logging-off", nologging, "Do not start logging right away (wait for MQTT log message)");
  // gyro offset
  bool calibGyro = false;
  cli.add_flag("-g,--gyro", calibGyro, "Calibrate gyro offset");
  float testSec = 0.0;
  cli.add_option("-t,--time", testSec, "Open all sensors for some time (seconds)");
  // rename feature
  int regbotNumber{-1};
  cli.add_option("-n,--number", regbotNumber, "Set name number to Teensy part [0..150] use with --interface.");
  int regbotInterface{0};
  cli.add_option("-i,--interface", regbotInterface, "Set interface number to Teensy 0.." + std::to_string(NUM_TEENSY_MAX-1) + " (default is 0)");
  // string subscribe;
  // cli.add_option("-m --mqtt-sub", subscribe, "Subscribe to a topic, e.g. 'robobot/drive/t1/mot");
  // rename feature
  int regbotHardware{-1};
  cli.add_option("-H,--hardware", regbotHardware, "Set robot hardware type (most likely 8) use with --interface.");
  // ini-file name
  //string ifn{"robot.ini"};
  cli.add_option("-z,--initialization", iniFileName, "Specify which initialization file to use, default is robot.ini");
  //
  // Parse for command line options
  cli.allow_windows_style_options();
  theEnd = true;
  CLI11_PARSE(cli, argc, argv);
  // if we get here, then command line parameters are OK to continue
  theEnd = false;
  //
  // create an ini-file structure
  iniFile = new mINI::INIFile(iniFileName);
  // and read the file (if any, else just create the 'ini' structure)
  iniFile->read(ini);
  // now ini structure is populated
  //
  if (not ini.has("service"))
  { // no data yet, so generate some default values
    ini["service"]["use_robot_hardware"] = "true";
    ini["service"]["logpath"] = "log_%d/";
    ini["service"]["; The '%d' will be replaced with date and timestamp (Must end with a '/')."] = "";
    ini["service"]["max_logging_minutes"] = "60.0";
    ini["service"]["log_service"] = "true";
  }
  bool teensyConnect = ini["service"]["use_robot_hardware"] == "true";
  if (ini["service"].has("max_logging_minutes"))
    maxLogMinutes = strtod(ini["service"]["max_logging_minutes"].c_str(), nullptr);
  // Check for selected values
  if (version)
  {
    printf("SVN service version%s\n", getVersionString().c_str());
    theEnd = true;
  }
  // gyro
  if (calibGyro and regbotInterface >=0 and regbotInterface < NUM_TEENSY_MAX)
  {
    if (teensyConnect)
      imu[regbotInterface].calibrateGyro();
    cliAction = true;
  }
  // rename
  if (regbotNumber >= 0 and regbotNumber <= 150 and regbotInterface >=0 and regbotInterface < NUM_TEENSY_MAX)
  { // save this number to the Teensy (Robobot) and exit
    teensy[regbotInterface].saveRegbotNumber = regbotNumber;
    cliAction = true;
  }
  if (regbotHardware >= 5 and regbotHardware <= 15 and regbotInterface >=0 and regbotInterface < NUM_TEENSY_MAX)
  { // save this number to the Teensy (Robobot) and exit
    teensy[regbotInterface].regbotHardware = regbotHardware;
    cliAction = true;
  }
  if (cliAction and not teensyConnect)
  {
    printf("# NB!NB! hardware is disabled in robot.ini, no Teensy settings available\n");
    theEnd = true;
  }
  topicMaster = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "master";
  //
  //
  // for setup timing
  UTime t("now");
  if (not theEnd)
  { // initialize all elements
    // logging
    logPath = ini["service"]["logpath"];
    int n = logPath.find("%d");
    if (n > 0)
    { // date should be added to path
      UTime t("now");
      std::string dpart = t.getForFilename();
      logPath.replace(n, 2, dpart);
    }
    std::error_code e;
    bool ok = filesystem::create_directory(logPath, e);
    if (ok)
    {
      printf("# UService:: created directory %s\n", logPath.c_str());
      string s = "cp robot.ini " + logPath;
      system(s.c_str());
      // printf("# UService:: robot.ini copied to %s/robot.ini\n", logPath.c_str());
      startedLogging.now();
    }
    else if (n > 0)
    { // failed (probably: path exist already)
      std::perror("#*** UService:: Failed to create log path:");
    }
    if (nologging)
    { // command line option for no logging at start
      stop_logging = true;
      printf("# UService:: no logging at start\n");
    }
    if (ini["service"]["log_service"] == "true")
    { // open logfile
      std::string fn = service.logPath + "log_service.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Service logfile\n");
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tMessage\n");
    }
    // mqtt
    mqtt.setup();
    mqttin.setup();
    lastMqttMessage.now();
    // teensy interface
    if (teensyConnect)
    { // open the main data source
      for (int tn = 0; tn < NUM_TEENSY_MAX; tn++)
      { // these primary interfaces are related to a Teensy
        teensy[tn].setup(tn);
      }
      setupTeensyConnection();
      gpio.setup();
    }
    else
    {
      printf("# UService::setup: Ignoring robot hardware (Regbot and GPIO)\n");
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Ignoring Teensy hardware (disabled in robot.ini)\n", t.getSec(), t.getMicrosec()/100);
    }
    // setup of all that do not directly interact with the robot
    // drive control loop
    mixer.setup();
    // manuel control from joypad
    joyLogi.setup();
    joy.setup();
    setupComplete = true;
    // allow threads to start
    usleep(2000);
    //
  }
  // Regbot (Teensy) need to accept settings before continue
  if (not theEnd and teensyConnect)
  { // wait for all settings to be accepted
    for (int tn = 0; tn < NUM_TEENSY_MAX; tn++)
    {
      if (teensy[tn].teensyConnectionOpen)
      {
        while (teensy[tn].getTeensyCommQueueSize() > 0 and t.getTimePassed() < 5.0)
          usleep(10000);
//         printf("# UService::setup - waited %.2f sec for full setup\n", t.getTimePassed());
        // decide if all setup is OK
        int retry = 0;
        int dumped = teensy[tn].getTeensyCommError(retry);
        if (dumped > 0 or retry > 15)
        { // may be OK
          if (dumped > 0)
            printf("# UService:: ************************************************************\n");
          printf("# UService:: Teensy %d setup: resend %d (OK), dumped %d messages (dumped should be 0)\n", tn, retry, dumped);
          if (dumped > 0)
            printf("# UService:: ************************************************************\n");
        }
        else
        {
          printf("# UService:: setup of Teensy %d modules finished OK.\n", tn);
        }
        theEnd = dumped > 0 or teensy[tn].getTeensyCommQueueSize() > 0;
        if (logfile != nullptr)
          fprintf(logfile, "%lu.%04ld Setup finished OK=%d\n", t.getSec(), t.getMicrosec()/100, dumped == 0);
      }
      else
      {
        if (not teensy[tn].disabled)
        {
          printf("# UService:: setup failed, no connection to Teensy %d - terminating.\n", tn);
          theEnd = true;
        }
      }
    }
  }
  // running from main loop
  if (not theEnd)
  { // start listen to the keyboard
    th1 = new std::thread(runObj, this);
  }
  // wait for optional tasks that require system to run.
  if ((regbotNumber >= 0 or
       regbotHardware > 3 or
       imu[regbotInterface].inCalibration[0] or
       testSec > 0.05) and
       not theEnd)
  { // wait until finished, then terminate
    UTime t("now");
    while ( (teensy[regbotInterface].saveRegbotNumber >= 0 and
      teensy[regbotInterface].saveRegbotNumber != robot[regbotInterface].idx) or
      (imu[regbotInterface].inCalibration[0] and t.getTimePassed() < 10) or
      t.getTimePassed() < testSec)
    {
      printf("# Service is waiting for a specified action to finish (waited %.0f sec)\n", t.getTimePassed());
      sleep(1);
    }
    theEnd = true;
  }
  return theEnd;
}

void UService::setupTeensyConnection()
{
  for (int tn = 0; tn < NUM_TEENSY_MAX; tn++)
  { // these primary interfaces are related to a Teensy
    // teensy[tn].setup(tn);
    robot[tn].setup(tn);
    //
    // wait for base setup to finish
    if (teensy[tn].teensyConnectionOpen)
    { // wait for initial setup
      usleep(10000);
      UTime t("now");
      while (teensy[tn].getTeensyCommQueueSize() > 0 and t.getTimePassed() < 7.0)
        usleep(10000);
      if (t.getTimePassed() >= 7.0)
        printf("# UService::setup - waited %g sec for initial Teensy setup\n", t.getTimePassed());
    }
    // setup and initialize all modules
    encoder[tn].setup(tn);
    usleep(3000);
    imu[tn].setup(tn);
    usleep(3000);
    servo[tn].setup(tn);
    usleep(3000);
    mvel[tn].setup(tn);
    usleep(3000);
    motor[tn].setup(tn);  // after mvel, as mvel makes sample time
    usleep(3000);
    current[tn].setup(tn);
    usleep(3000);
    distforce[tn].setup(tn);
    usleep(3000);
    edge[tn].setup(tn);
    usleep(30000);
  }
}

bool UService::mqttDecode(const char* topic, const char * payload, UTime& msgTime)
{ // message received from MQTT channel
  bool used = true;
  const int MSL = 100;
  char s[MSL];
  // if (strstr(topic, "/cmd/") != nullptr)
  // { // debug
  //   printf("# got from MQTT/ROS: topic:'%s', payload bytes %lu, '%s'\n",
  //          topic, strlen(payload), payload);
  // }
  if (strstr(topic, "/cmd/T0") != nullptr)
  { // message to the Teensy , pass on
    // keyword is last part of topic or embedded in payload
    // const char * p1 = strstr(topic, "/cmd/T0") + 7;
    bool ok;
    std::snprintf(s, MSL, "%s\n", payload);
    ok = teensy[0].send(s);
    if (not ok)
    {
      printf("# UService::mqttDecode: got '%s' '%s', send (queued) to T0 ok=%d as %s", topic, payload, ok, s);
    }
  }
  else if (strstr(topic, "/cmd/shutdown") != nullptr)
  {
    if (logfile != nullptr)
      fprintf(logfile, "%lu.%04ld Shutdown: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    printf(" Uservice::mqttDecode %lu.%04ld Shutdown: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    power_off_request(false, payload);
    used = true;
  }
  else if (strstr(topic, "/cmd/ti") != nullptr)
  { // message to this teensy_interface
    // const char * p1 = strstr(topic, "/cmd/ti") + 7;
    // bool ok = false;
    // if (logfile != nullptr)
    //   fprintf(logfile, "%lu.%04ld Interface command: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    // printf("# pre  %lu.%04ld Interface command: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    // decode message to this interface
    const char * p1 = payload;
    if (mixer.decode(p1, msgTime))
    {
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Mixer order: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    }
    else if (strncmp(p1, "log", 3) == 0)
    { // start or stop logging
      p1 += 3;
      int v = strtol(p1, nullptr, 10);
      stop_logging = v == 0;
      if (stop_logging)
      { // tell keyboard function to flush
        UTime t("now");
        flushLog = true;
      }
      else
      {
        printf("# Started logging\n");
        startedLogging.now();
      }
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Log command: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    }
    else if (strncmp(p1, "alive", 5) == 0)
    {
      p1 += 5;
      while (isspace(*p1))
        p1++;
      int n = strlen(p1);
      if (n > MID)
        n = MID - 1;
      if (masterAliveCnt == 0)
      {
        masterAliveCnt += 1;
        strncpy(masterAliveID, p1, n);
        masterAliveTime.now();
        printf("# MQTT decode:: (err=%d) new master '%s'\n",
               masterAliveErr, masterAliveID);
        if (logfile != nullptr)
          fprintf(logfile, "%lu.%04ld new Master: (%d) %s\n",
                  msgTime.getSec(), msgTime.getMicrosec()/100,
                  masterAliveErr, masterAliveID);

      }
      else
      {
        if (strncmp(p1, masterAliveID, n) == 0)
        {
          masterAliveTime.now();
          if (masterAliveErr > 0)
            masterAliveErr--;
          // printf("# MQTT decode:: (err=%d) same master '%s'=='%s'\n",
          //        masterAliveErr, payload, masterAliveID);
        }
        else
        {
          if (masterAliveErr == 0)
            masterAliveErr = 10;
          masterAliveErr++;
          printf("# MQTT decode:: (err=%d) master overload '%s' != '%s'\n",
                 masterAliveErr, payload, masterAliveID);
          if (logfile != nullptr)
            fprintf(logfile, "%lu.%04ld Master overload: (%d) %s != %s\n",
                    msgTime.getSec(), msgTime.getMicrosec()/100,
                    masterAliveErr, payload, masterAliveID);
        }
        // inform about who is master
        if (ini["mqtt"]["use"] == "true")
        { // send master ID back to master and to false newcomers
          mqtt.publish(topicMaster.c_str(), masterAliveID, msgTime);
        }
      }
    }
    else
      used = false;
    // printf("# post %lu.%04ld Interface command: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    if (not used)
    {
      printf("# UService::mqttDecode: got '%s' '%s', but left unused (as %s)\n", topic, payload, p1);
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Unused command: %s %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, topic, payload);
    }
  }
  else
  {
    printf("# UService MQTT messages not handled yet (%s %s)\n", topic, payload);
    used = false;
  }
  lastMqttMessage.now();
  return used;
}


bool UService::decode(const char* msg, UTime& msgTime, int tn)
{ // decode messages from Teensy
  bool used = true;
  if      (robot[tn].decode(msg, msgTime)) {}
  else if (encoder[tn].decode(msg, msgTime)) {}
  else if (imu[tn].decode(msg, msgTime)) {}
  else if (servo[tn].decode(msg, msgTime)) {}
  // else if (as5147[tn].decode(msg, msgTime)) {}
  else if (motor[tn].decode(msg, msgTime)) {}
  else if (current[tn].decode(msg, msgTime)) {}
  else if (distforce[tn].decode(msg, msgTime)) {}
  else if (edge[tn].decode(msg, msgTime)) {}
  //
  // add other Teensy data users here
  //
  else
    used = false;
  return used;
}

void UService::stopNow(const char * who)
{ // request a terminate and exit
  printf("# UService:: %s say stop now\n", who);
  stopNowRequest = true;
}


void UService::terminate()
{ // Terminate modules (especially threads and log files)
  UTime t("now");
  if (terminating or not setupComplete)
  {
    // printf("# terminate called twice or premature ??? -----------\n");
    return;
  }
  terminating = true;
  printf("# --------- terminating -----------\n");
  for (int tn = 0; tn < NUM_TEENSY_MAX; tn++)
  { // stop any motor activity
    teensy[tn].send("stop\n");
  }
  stop = true; // stop all threads, when finished current activity
  // wait 100ms to allow most threads to stop
  usleep(100000);
  joy.terminate();
  joyLogi.terminate();
  gpio.terminate();
  mixer.terminate();
  for (int tn = 0; tn < NUM_TEENSY_MAX; tn++)
  {
    edge[tn].terminate();
    motor[tn].terminate();
    encoder[tn].terminate();
    imu[tn].terminate();
    robot[tn].terminate();
    servo[tn].terminate();
    mvel[tn].terminate();
    current[tn].terminate();
    distforce[tn].terminate();
    // terminate sensors before Teensy
    teensy[tn].terminate();
  }
  mqtt.terminate(); // outgoing to MQTT server
  mqttin.terminate(); // from MQTT server
  // service must be the last to close
  if (not ini.has("ini"))
  {
    ini["ini"]["; set 'saveConfig' to 'false' to avoid autosave"] = "";
    ini["ini"]["saveConfig"] = "true";
  }
  std::string shouldSave = ini["ini"]["saveConfig"];
  // fflush(NULL);
  if (shouldSave != "false")
  { // write any changes ini-file values (and structures)
    ini["ini"]["version"] = getVersionString();
    printf("# UService:: Trying to save %s (crashes sometimes)\r\n", iniFileName.c_str());
    iniFile->write(ini, true);
    printf("# UService:: configuration saved to %s\n", iniFileName.c_str());
  }
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld All terminated; closing logfile\n", t.getSec(), t.getMicrosec()/100);
    fclose(logfile);
  }
}

std::string UService::getVersionString()
{
  // #define REV "$Id: uservice.cpp 1234 2026-02-01 08:46:32Z jcan $"
  std::string ver = REV;
  int n1 = ver.find(' ', 10);
  int n2 = ver.rfind("Z ");
  std::string part = ver.substr(n1, n2-n1);
  return part;
}

bool UService::GetLineFromCin()
{
  bool result = false;
  int c = read(STDIN_FILENO, &keyLine[keyLineIdx], 1);
  if (c == 1)
  {
    if (keyLine[keyLineIdx] == '\n')
    {
      keyLine[keyLineIdx] = '\0';
      result = true;
    }
    else if (keyLineIdx < MKL-1)
      keyLineIdx++;
  }
  return result;  
}

void UService::run()
{ // 
  UTime t("now");
  UTime t2("now");
  if (not asDaemon)
  { // set keyboard to non-blocking
    printf("# Type quit to stop, or 'h' for help\r\n>>");
    fcntl(STDIN_FILENO, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  }
  while (not (stop or cliAction or stopNowRequest))
  { // listen for simple commands (e.g. quit)
    t2.now();
    if (GetLineFromCin())
    { // listen to keyboard
      const char * p1 = keyLine;
      // printf("# UService::run: Got %d from keyboard: %s\n", keyLineIdx, p1);
      if (keyLineIdx == 4)
      {
        stopNowRequest  =  strstr("stop quit exit", p1) != nullptr;
      }
      if (strncmp(p1, "mqtt sub ", 9) == 0)
      {
        p1 += 9;
        mqttin.subscribe(p1, 0);
        printf("# subscribe to %s called\n", p1);
      }
      else if (strncmp(p1, "sub ", 4) == 0)
      {
        teensy[0].send(p1);
        printf("# subscribe '%s' send to Teensy\n", p1);
      }
      else if (strncmp(p1, "log", 3) == 0)
      {
        p1 += 3;
        bool logging = not stop_logging;
        if (strlen(p1) > 0)
          stop_logging = strtol(p1, nullptr, 10) == 0;
        else
          stop_logging = false;
        if (stop_logging)
        { // tell keyboard function to flush
          if (logging)
          {
            printf("# Stopped logging, logged after %.1f seconds\n", startedLogging.getTimePassed());
            flushLog = true;
          }
          else
            printf("# Logging already stopped \n");
        }
        else
        {
          if (not logging)
          {
            printf("# Started logging\n");
            startedLogging.now();
          }
          else
            printf("# Logging already started\n");
        }
      }
      else if (*p1 == 'h')
      {
        printf("# Available commands:\n");
        printf("#     quit \tTerminate teenst_interface - closing logfiles etc.\n");
        printf("#     log E \tLogging enable: E=1 enable logging, E=0 disable logging (enabled=%d)\n", not stop_logging);
        printf("#     sub xx i \tSubscribe to additional data from Teensy.\n");
        printf("#              \txx is subject (see rsewiki).\n");
        printf("#              \ti is interval in ms (i=0 stops subscription).\n");
        printf("#     help \tThis help message.\n");
      }
      if (not stopNowRequest)
        printf("# Type quit to stop, or 'h' for help\r\n>>");
      keyLineIdx = 0;
    }
    else
    {
      if (t2.getTimePassed() > 0.05)
      {
        printf("# no line, idx=%d, took %.3f sec (flush=%d) appTime=%.3f\r\n", keyLineIdx, t2.getTimePassed(), flushLog, app_time);
      }
    }
    if (flushLog)
    {
      flushLog = false;
      t2.now();
      fflush(nullptr);
      printf("# Flush of logfiles to disk took %f sec\n", t2.getTimePassed());
    }
    if (false and lastMqttMessage.getTimePassed() > 4.0)
    { // dropped for now -- make manual MQTT messages problemetic
      // we have lost the python app, stop the wheels
      // probably never used, as alive will timeout first
      if (mixer.shouldWheelsBeRunning())
      {
        mixer.setVelocity(0, 0);
        if (logfile != nullptr)
          fprintf(logfile, "%lu.%04ld Lost MQTT messages for %.1f sec, so set velocity to (0,0)\n", t.getSec(), t.getMicrosec()/100, lastMqttMessage.getTimePassed());
        lastMqttMessage.now();
      }
    }
    if (masterAliveTime.getTimePassed() > 4.0 and masterAliveCnt > 0)
    { // master lost
      printf("# UService::run: master lost (%s), no alive in %.2f sec\n",
             masterAliveID, masterAliveTime.getTimePassed());
      masterAliveCnt = 0;
      masterAliveErr = 0;
      // stop the robot
      printf("# UService:: should probably stop the robot, but ignored for now.\n");
      // mixer.setVelocity(0, 0);
      //
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Lost Master, alive since %s\n",
                t.getSec(), t.getMicrosec()/100,
                masterAliveID);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    app_time += 0.1; // rough estimate of app time without using system time
    //
    if (startedLogging.getTimePassed()/60 > maxLogMinutes and not service.stop_logging)
    {
      printf("# Main:: Teensy interface stopped logging after %.2f minutes\n", startedLogging.getTimePassed()/60);
      service.stop_logging = true;
      if (logfile != nullptr)
        fprintf(logfile, "%lu.%04ld Stopped logging after %.1f min\n", t.getSec(), t.getMicrosec()/100, startedLogging.getTimePassed()/60);
    }
  }
  if (not asDaemon)
  { 
    printf("# UService:: stopped listening to keyboard\r\n");
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) & ~O_NONBLOCK);
    // tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
  }
}


// not used
float UService::usedBatteryCapacity()
{ // sum used capacity from each of the Teensy boards connected
  double usedWh = 0;
  for (int i=0; i < NUM_TEENSY_MAX; i++)
    usedWh += robot[i].batteryUsedWh;
  return usedWh;
}

void UService::power_off_request(bool fromTeensy, const char * who)
{ // from Teensy
  if (powerOffRequested)
    return;
  powerOffRequested = true;
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld Got power off from (to teensy %d) %s\n",
            t.getSec(), t.getMicrosec()/100, fromTeensy, who);
  }
  if (fromTeensy)
    printf("# Power off requested (from %s, ~40 sec before power cut)\n", who);
  else
    printf("# Power off requested (from %s)\n", who);
  mixer.setVelocity(0, 0);
  if (fromTeensy)
  {
    mqtt.publish("robobot/cmd/shutdown", who, t, 1);
  }
  else
  {  // tell Teensy to cut power in 40 seconds
    teensy[0].send("off 40\n");
    if (logfile != nullptr)
      fprintf(logfile, "%lu.%04ld send power off to teensy\n", t.getSec(), t.getMicrosec()/100);
  }
  flushLog = true;
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

void UService::logMessage(const char * msg)
{
  if (logfile != nullptr)
  {
    UTime t("now");
    fprintf(logfile, "%lu.%04ld msg: %s\n", t.getSec(), t.getMicrosec()/100, msg);
  }
}
