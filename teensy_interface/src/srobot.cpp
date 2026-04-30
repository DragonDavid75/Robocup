/*
 * 
 * Copyright © 2022 DTU, 
 * Author:
 * Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include <string>
#include <string.h>
#include <math.h>

#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>
#include <unistd.h>
//#include <netpacket/packet.h>
//#include <linux/wireless.h>
//#include <sys/ioctl.h>


#include "steensy.h"
#include "srobot.h"
#include "uservice.h"
#include "umqtt.h"

// create the class with received info
SRobot robot[NUM_TEENSY_MAX];


void SRobot::setup(int teensyNumber)
{ /// subscribe to pose information
  tn = teensyNumber;
  ini_section = "robot" + std::to_string(tn);
  tnGroup = "teensy" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no teensy group, so make one.
    ini[ini_section]["log"] = "true";
    ini[ini_section]["print"] = "false";
    ini[ini_section]["regbot_version"] = "000";
    // ini[ini_section]["shutdown_file"] = "/home/local/shutdown.now"; // changed to off_by_mqtt
    ini[ini_section]["batteryUsedWh"] = "0";
    ini[ini_section]["batteryCalibrate"] = "1.0";
    ini[ini_section]["robotnamepath"] = "/home/local/svn/log/robotname";
  }
  else if (not ini[ini_section].has("robotnamepath"))
  {
    ini[ini_section]["robotnamepath"] = "/home/local/svn/log/robotname";
  }
  toConsole = ini[ini_section]["print"] == "true";
  batteryUsedWh = strtod(ini[ini_section]["batteryUsedWh"].c_str(), nullptr);
  batteryScale = strtod(ini[ini_section]["batteryCalibrate"].c_str(), nullptr);
  string ss = "batcal " + to_string(batteryScale) + "\n";
  teensy[tn].send(ss.c_str());
  // topic names
  topicHbt = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/hbt";
  if (ini[ini_section]["log"] == "true" and logfile == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_hbt.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Heartbeat logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tRobot name index\n");
    fprintf(logfile, "%% 3 \tVersion\n");
    fprintf(logfile, "%% 4 \tState (0 = control is external to Teensy)\n");
    fprintf(logfile, "%% 5 \tBattery voltage (V)\n");
    fprintf(logfile, "%% 6 \tTeensy load (%%)\n");
    fprintf(logfile, "%% 7 \tBoard supply current (A)\n");
    fprintf(logfile, "%% 8 \tUsed battery capacity (Wh) (never reset properly)\n");
    fprintf(logfile, "%% 9 \tShutdown request.\n");
    fprintf(logfile, "%% 10 \tCPU temperature (Raspberry).\n");
    // fprintf(logfile, "%% 11 \tIP4 adresses.\n");
  }
  if (th1 == nullptr)
    th1 = new std::thread(runObj, this);
  else
    ip_sentToTeensy = false;
  setupOK = true;
}

void SRobot::subscribeDataFromTeensy()
{
  teensy[tn].send("sub hbt 500\n");
  service.logMessage("# SRobot:: subscribed to HBT");
  hbtTime.now();
}


void SRobot::terminate()
{
  if (logfile != nullptr)
  {
    dataLock.lock();
    fclose(logfile);
    logfile = nullptr;
    dataLock.unlock();
  }
  if (th1 != nullptr)
    th1->join();
}


bool SRobot::decode(const char* msg, UTime & msgTime)
{ // like: regbot:hbt 37708.7329 74 1430 5.01 0 6 1 1
  /* hbt 1 : time in seconds, updated every sample time
  *     2 : device ID (probably 1)
  *     3 : software revision number - from SVN * 10 + REV_MINOR
  *     4 : Battery voltage
  *     5 : state
  *     6 : hw type
  *     7 : load
  *     8 : Battery capacity
  *     9 : Shutdown request
  */
  bool used = true;
  if (not setupOK)
    // not ready
    return used;
  const char * p1 = msg;
  if (strncmp(p1, "hbt ", 4) == 0)
  { // decode HBT message
    // advance to first parameter
    if (strlen(p1) > 5)
      p1 += 4;
    else
      return false;
    // get data
    dataLock.lock();
    // time in seconds from Teensy
    double tt = strtof64(p1, (char**)&p1);
    teensyTime = tt;
    int x = strtol(p1, (char**)&p1, 10); // index (robot number)
    if (x != idx)
    { // set robot number into ini-file
      idx = x;
      ini[tnGroup]["idx"] = to_string(idx);
      // also ask for the new name
      teensy[tn].send("idi\n", true);
//       printf("# SRobot::decode: asked for new name (idi -> dname)\n");
    }
    int rv = strtol(p1, (char**)&p1, 10); // index (from SVN)
    if (rv != version)
    {
      version = rv;
      ini[ini_section]["regbot_version"] = to_string(rv);
    }
    batteryVoltage = strtof(p1, (char**)&p1); // y
    controlState = strtol(p1, (char**)&p1, 10); // control state 0=no control, 1=RC, 2=auto (if exist)
    //
    type = strtol(p1, (char**)&p1, 10); // hardware type
    ini[tnGroup]["hardware"] = std::to_string(type);
    //
    load = strtof(p1, (char**)&p1); // Teensy load in %
    supplyCurrent = strtof(p1, (char**)&p1); // supply current
    float dt = msgTime - hbtTime;
    if (dt < 2.0)
    { // when battery is fully charged, the used capacity is reset.
      // beeing charged, but not full, is unknown.
      if (batteryVoltage > fullyChargedVoltage)
        batteryUsedWh = 0;
      if (batteryVoltage > 6.0)
      { // we are not on USB power, so trust supply current
        float usedWs = supplyCurrent * batteryVoltage * dt;
        batteryUsedWh += usedWs / 3600.0;
        ini[ini_section]["batteryUsedWh"] = std::to_string(batteryUsedWh);
      }
    }
    shutdown_count = strtol(p1, (char**)&p1, 10); // Request from Teensy to shut down (off button or low battery_low_cnt)
    //
    hbtTime = msgTime;
    // save to log if file is open
    toLog();
    dataLock.unlock();
    if (shutdown_count > 100)
    { // from keyboard or from Teensy when low on power - counts to 20000 (20 seconds), then cut power
      if (not ini[ini_section].has("shutdown_file"))
        ini[ini_section]["shutdown_file"] = "shutdown.now";
      FILE * shutdown = fopen(ini[ini_section]["shutdown_file"].c_str(), "a"); // "a" for append
      if (shutdown != nullptr)
      {
        const int MSL = 100;
        char s[MSL];
        fprintf(shutdown, "# URobot:: received this 'hbt' with power-off bit set: %s", msg);
        fprintf(shutdown, "Shutdown at %lu.%03ld (%s)\n", hbtTime.getSec(), hbtTime.getMillisec(), hbtTime.getDateTimeAsString(s));
        fclose(shutdown);
      }
      printf("# URobot:: received a 'hbt' with power-off count %d/10000 from: %s", shutdown_count, msg);
      if (shutdown_count > 1000)
      {
        service.stopNow("request from Teensy hbt");
      }
    }
  }
  else if (strncmp(p1, "power off", 9) == 0)
  { // power off button pressed
    service.power_off_request(true, "Teensy (battery)");
  }
  else
    used = false;
  return used;
}


void SRobot::toLog()
{ // data is already locked
  if (service.stop)
    return;
  const int MSL = 200;
  char s[MSL];
  char st[MSL];
  snprintf(st, MSL, "%lu.%04ld", hbtTime.getSec(), hbtTime.getMicrosec()/100);
  snprintf(s, MSL, "%d %d %d %.2f %.1f %.2f %.3f %d %.1f\n",
           idx, version, controlState, batteryVoltage,
           load, supplyCurrent, batteryUsedWh, shutdown_count, cpuTemp);
  // if (ini["mqtt"]["use"] == "true")
  // {
  //   mqtt.publish(topicHbt.c_str(), s, hbtTime);
  // }
  if (logfile != nullptr and not service.stop_logging)
  {
    fprintf(logfile, "%s %s", st, s);
  }
  if (toConsole)
    printf("# state %s %s", st, s);
}

void SRobot::run()
{ // potential keyboard input
  bool ip_list_changed;
  int loop = 0;
  bool ipRemoved = false;
  std::string tm = exec("date");
  while (not service.stop)
  {
    findIPs();
    // update message
    ip_list_changed = updateIPlist();
    // tell Regbot display about the host IP
    float t = measureCPUtemp();
    if (fabsf(t - cpuTemp) > 1.5)
    { // log on temperature shift
      cpuTemp = t;
      toLog();
    }
    //
    if (ip_list_changed)
      ip_sentToTeensy = false;
    if (not ip_sentToTeensy or ipRemoved)
    { // tell Teensy and show IP on display
      // but don't if a mission is running
      if (true)
      { // device present and no user
//         printf("# Teensy available\n");
        if (teensy[tn].teensyConnectionOpen)
        {
          const int MSL = 200;
          char s[MSL];
          snprintf(s, MSL, "disp # %s\n", ip4list);
          teensy[tn].send(s);
          toLog();
          // request also robot name
          ip_sentToTeensy = true;
          ip_list_changed = false;
          ipRemoved = false;
        }
      }
    }
    sleep(1);
    loop++;
  }
}

bool SRobot::findIPs()
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  bool changed = false;
  //
  getifaddrs(&ifAddrStruct);
  ipsCnt = 0;
  //   macCnt = 0;
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
  { // all interface names and types
    if (ifa->ifa_addr->sa_family == AF_INET)
    { // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      if (strcmp(ifa->ifa_name, "lo") != 0)
      { // not loop back - loopback is skipped
        int sl = strlen(addressBuffer);
        if (sl > 0 and strncmp(ips[ipsCnt], addressBuffer, sl) != 0)
        {
          changed = true;
          snprintf(ips[ipsCnt], MHL, "%s", addressBuffer);
          printf("# SRobot:: (t%d) found IP %d: %s %s\n", tn, ipsCnt, ifa->ifa_name, ips[ipsCnt]);
        }
        if (ipsCnt < MIPS - 1)
          ipsCnt++;
      }
    }
  }
  return changed;
}

bool SRobot::updateIPlist()
{ // update IP list
  int n = 0;
  char * p1 = ip4list;
  bool changed = false;
  for (int i = 0; i < ipsCnt; i++)
  {
    snprintf(p1, MHL2 - n, " %s", ips[i]);
    n += strlen(p1);
    p1 = &ip4list[n];
  }
  // see if it has changed
  if (strcmp(ip4list, ip4listLast) != 0)
  { // make IP list message
//     toLog(ip4list);
    changed = true;
    strncpy(ip4listLast, ip4list, MHL2);
  }
  return changed;
}

float SRobot::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
    //     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s);
    fclose(temp);
  }
  return t/1000.0;
}


std::string SRobot::exec(std::string cmd)
{
//   toLog(cmd.c_str());
  std::array<char, 100> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
//   toLog(result.c_str());
  return result;
}

void SRobot::setRobotName(const char* toName)
{
  int same = strcmp(toName, robotName.c_str());
  if (same != 0)
  { // toName includes a '\n'
    printf("# SRobot::setRobotName (section %s) new robot name %s", ini_section.c_str(), toName);
    robotName = toName;
    FILE * f = fopen(ini[ini_section]["robotnamepath"].c_str(), "w");
    if (f != nullptr)
    {
      fprintf(f, "%s", toName);
      fclose(f);
    }
    else
      printf("SRobot::setRobotName: failed to save robot %d name (%s) to '%s'\n", tn, toName, ini[ini_section]["robotnamepath"].c_str());
  }
}

void SRobot::tick()
{
  // printf("# Robot tick, time=%f\n", hbtTime.getTimePassed());
  if (hbtTime.getTimePassed() > 2.2)
  {
    subscribeDataFromTeensy();
  }
}

