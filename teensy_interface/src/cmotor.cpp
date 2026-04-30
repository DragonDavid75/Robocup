/*  
 * 
 * Copyright © 2024 DTU,
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
#include "sencoder.h"
#include "cmotor.h"
#include "steensy.h"
#include "uservice.h"
#include "mvelocity.h"
#include "cmixer.h"
#include "umqtt.h"
#include "srobot.h"

// create value
CMotor motor[NUM_TEENSY_MAX];


void CMotor::setup(int teensy_number)
{ // ensure there is default values in ini-file
  tn = teensy_number;
  ini_section = "motor_teensy_" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // motor block is OK, but control parameters are needed too
    ini[ini_section]["m1kp"] = "7.0"; // unit is (V per (m/sec))
    ini[ini_section]["m1lead"] = "0 1.0"; // tau_d (sec) and alpha, tau_d = 0.0 means no function
    ini[ini_section]["m1taui"] = "0.05"; // tau_i (sec) 0.0 is no integrator function
    ini[ini_section]["m1feedForward"] = "0"; // tau_i (sec) 0.0 is no integrator function
    ini[ini_section]["m1maxMotV"] = "10.0"; // (volt)
    ini[ini_section]["m1Voffset"] = "0.0"; // added to motor voltage (volt)
    //
    ini[ini_section]["m2kp"] = "7.0"; // unit is (V per (m/sec))
    ini[ini_section]["m2lead"] = "0 1.0"; // tau_d (sec) and alpha, tau_d = 0.0 means no function
    ini[ini_section]["m2taui"] = "0.05"; // tau_i (sec) 0.0 is no integrator function
    ini[ini_section]["m2feedForward"] = "0"; // tau_i (sec) 0.0 is no integrator function
    ini[ini_section]["m2maxMotV"] = "10.0"; // (volt)
    ini[ini_section]["m2Voffset"] = "0.0"; // added to motor voltage (volt)
    //
    // log motor voltage (from Teensy)
    ini[ini_section]["log_voltage"] = "false";
    //
    ini[ini_section]["m1log_PID"] = "false";
    ini[ini_section]["m2log_PID"] = "false";
    //
    ini[ini_section]["m1print"] = "false";
    ini[ini_section]["m2print"] = "false";
    // update times for motor control
    ini[ini_section]["interval_motv_ms"] = "33"; // ms
    ini[ini_section]["interval_motpwm_ms"] = "0"; // ms
    ini[ini_section]["relax_sec"] = "3.5";  // keep zero velocity for 3.5 secs before relaxing motors
  }
  //
  // get ini-values
  sampleTime = mvel->sampleTime;
  for (int m = 1; m <= SRobot::MAX_MOTORS; m++)
  {
    string mkp = "m" + to_string(m) + "kp";
    float kp = strtof(ini[ini_section][mkp].c_str(), nullptr);
    string mlead = "m" + to_string(m) + "lead";
    const char * p1 = ini[ini_section][mlead].c_str();
    // lead
    float taud = strtof(p1, (char**)&p1);
    float alpha = strtof(p1, (char**)&p1);
    // integrator
    string mtaui = "m" + to_string(m) + "taui";
    float taui = strtof(ini[ini_section][mtaui].c_str(), nullptr);
    // feed forward
    string mff = "m" + to_string(m) + "feedForward";
    float ff = strtof(ini[ini_section][mff].c_str(), nullptr);
    // output limit
    string mmaxv = "m" + to_string(m) + "maxMotV";
    float maxMotV = strtof(ini[ini_section][mmaxv].c_str(), nullptr);
    // This should be changes
    pid[m-1].setup(sampleTime, kp, taud, alpha, taui, ff, maxMotV);
    //
    string moffset = "m" + to_string(m) + "Voffset";
    motorVoltageOffset[m-1] = strtof(ini[ini_section][moffset].c_str(), nullptr);
    //
    timeToRelax = strtof(ini[ini_section]["relax_sec"].c_str(), nullptr);
  }
  // mqtt
  topicMotv = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/mot";

  // sample time from encoder module
  // sampleTime = strtof(ini["encoder0"]["interval_ms"].c_str(), nullptr) / 1000.0;
  pid[0].toConsole = ini[ini_section]["m1print"] == "true";
  pid[1].toConsole = ini[ini_section]["m2print"] == "true";
  // subscribe interval
  sub_mvt = strtol(ini[ini_section]["interval_motv_ms"].c_str(), nullptr, 10);
  sub_mpt = strtol(ini[ini_section]["interval_motpwm_ms"].c_str(), nullptr, 10);
  // subscribeDataFromTeensy();
  //printf("# cmotor:: debug 5\n");
  // initialize logfile
  for (int i = 0; i < SRobot::MAX_MOTORS; i++)
  {
    string ts = "m" + to_string(i+1) + "log_PID";
    if (ini[ini_section][ts] == "true" and logfile[i] == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_t" + to_string(teensy_number) +  "_motor_" + to_string(i) + "_pid.txt";
      logfile[i] = fopen(fn.c_str(), "w");
      logfileLeadText(logfile[i], fn.c_str());
      pid[i].logPIDparams(logfile[i], false);
    }
  }
  if (ini[ini_section]["log_voltage"] == "true" and logfileMv == nullptr)
  {
    std::string fn = service.logPath + "log_t" + to_string(tn) + "_motor_voltage.txt";
    logfileMv = fopen(fn.c_str(), "w");
    fprintf(logfileMv, "%% Motor voltage for all motors\n");
    fprintf(logfileMv, "%% 1 \tTime (sec)\n");
    fprintf(logfileMv, "%% 2-3 \tVoltage to motor (to Teensy) 1,2 (Volt)\n");
    fprintf(logfileMv, "%% 4-5 \tVoltage to motor (from Teensy) 1,2 (Volt)\n");
    fprintf(logfileMv, "%% 6-7 \tPWM to motor (+/- 2096) (from Teensy) 1,2\n");
    fprintf(logfileMv, "%% 8 \tRelax motor controller (standing still for some time)\n");
  }
  //printf("# cmotor:: debug 6\n");
  if (th1 == nullptr)
    th1 = new std::thread(runObj, this);
  //printf("# cmotor:: debug 7\n");
}

void CMotor::subscribeDataFromTeensy()
{
  if (sub_mvt > 0)
  {
    string ts = "sub mot " + to_string(sub_mvt) + "\n";
    teensy[tn].send(ts.c_str());
  }
  if (sub_mpt > 0)
  {
    string ts = "sub motpwm " + to_string(sub_mpt) + "\n";
    teensy[tn].send(ts.c_str());
  }
  if (sub_mpt > 0 or sub_mpt > 0)
  {
    service.logMessage("# CMotor: subscribed to Teensy data");
    sub_mpt_t.now();
    sub_mvt_t.now();
  }
}

void CMotor::logfileLeadText(FILE * f, const char * ms)
{
    fprintf(f, "%% Teensy %d Motor control (%s) logfile\n", tn, ms);
    fprintf(f, "%% 1 \tTime (sec)\n");
    fprintf(f, "%% 2 \tReference for teensy %d motor (rad/sec or mm/s)\n", tn);
    fprintf(f, "%% 3 \tMeasured velocity for motor (rad/sec or mm/s)\n");
    fprintf(f, "%% 4 \tValue after Kp (V)\n");
    fprintf(f, "%% 5 \tValue after Lead (V)\n");
    fprintf(f, "%% 6 \tIntegrator value (V)\n");
    fprintf(f, "%% 7 \tMotor voltage output (V)\n");
    fprintf(f, "%% 8 \tIs output limited (1=limited)\n");
}

void CMotor::terminate()
{
  if (th1 != nullptr)
    th1->join();
  UTime t("now");
  char d[100];
  t.getDateTimeAsString(d);
  for (int i = 0; i < SRobot::MAX_MOTORS; i++)
  {
    if (logfile[1] != nullptr and not service.stop_logging)
    {
      fprintf(logfile[i], "%% ended at %lu.%4ld %s\n", t.getSec(), t.getMicrosec()/100, d);
      fclose(logfile[i]);
      logfile[i] = nullptr;
    }
  }
  if (logfileMv != nullptr)
    fclose(logfileMv);
}

bool CMotor::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "mot ", 4) == 0)
  {
    p1 += 4;
//     motvTime = msgTime;
    for (int i = 0; i < SRobot::MAX_MOTORS; i++)
      motorVoltage[i] = strtof(p1, (char**)&p1);
    // save to log_encoder_pose
    toLogMv(msgTime);
    sub_mvt_t.now();
  }
  else if (strncmp(p1, "motpwm ", 7) == 0)
  { /* From umotor.cpp (Teensy code)
    snprintf(s, MSL, "motpwm %d %d %d %d %d %d %d %d  %d  %d %d %d %d\r\n",
    motorAnkerDir[0], motorAnkerPWM[0], motorAnkerDir[1], motorAnkerPWM[1],
    motorAnkerDir[2], motorAnkerPWM[2], motorAnkerDir[3], motorAnkerPWM[3],
    PWMfrq, m1ok, m2ok, m3ok, m4ok);
    */
    p1 += 7;
//     motvTime = msgTime;
    for (int i = 0; i < SRobot::MAX_MOTORS; i++)
    {
      strtol(p1, (char**)&p1, 10); // Ignore direction
      motorPWM[i] = strtol(p1, (char**)&p1, 10); // PWM
    }
    // save to log_encoder_pose
    toLogMv(msgTime);
    sub_mpt_t.now();
  }
  else
    used = false;
  return used;
}

void CMotor::toLogMv(UTime & updt)
{
  if (logfileMv != nullptr and not service.stop_logging)
  {
    fprintf(logfileMv, "%lu.%04ld %.2f %.2f %.2f %.2f %d %d %d\n",
            updt.getSec(), updt.getMicrosec()/100,
            u[0], u[1],
            motorVoltage[0], motorVoltage[1],
            motorPWM[0], motorPWM[1], relax);
  }
}

void CMotor::setRelax(bool value)
{
  relax = value;
  relaxing = 0;
}

void CMotor::run()
{
  int loop = 0;
  int euc;
  UTime t;
  relaxTime.now();
  while (not service.stop)
  { // run an update at same rate as velocity estimate update
    euc = mvel[tn].updateCnt;
    if (euc != velUpdateCnt)
    { // do constant rate control
      // that is every time new encoder data is available
      // new motor control values should be calculated.
      velUpdateCnt = euc;
      if (not relax)
      { // do velocity control.
        // got new encoder data
        float dt = updTime - mvel[tn].velTime;
        // desired velocity from mixer
        if (dt < 1.0)
        { // valid control timing
          for (int m= 0; m < SRobot::MAX_MOTORS; m++)
          {
            u[m] = pid[m].pid(desiredVelocity[m], mvel[tn].motorVel[m], limited[m], motorVoltageOffset[m]);
          }
        }
        updTime = mvel[tn].velTime;
        // log_pose - for both motors
        for (int i = 0; i < SRobot::MAX_MOTORS; i++)
        {
          pid[i].saveToLog(logfile[i], updTime);
        }
        // finished calculating motor voltage (into u)
        const int MSL = 100;
        char s[MSL];
        /// Note that left and right requires different sign to move forward.
        /// This may be hidden by a sign change in the motor driver firmware.
        snprintf(s, MSL, "motv %.2f %.2f\n", u[0], u[1]);
        t.now();
        teensy[tn].send(s, true);
        // if (mixer.shouldWheelsBeRunning())
        // { // we are driving (or should)
        //   relaxTime.now();
        // }
        // else if (relaxTime.getTimePassed() > timeToRelax)
        // { // velocities are commanded zero for some seconds
        //   // stop controlling the wheels (allow motor controller relax)
        //   relax = true;
        //   relaxing = 0;
        // }
        relaxing = 0;
      }
      else
      { // relax motor loop
        if (relaxing < 3)
        { // send relax values
          for (int i = 0; i < SRobot::MAX_MOTORS; i++)
          { // clear PID controller
            u[i] = 0;
            pid[i].resetHistory();
          }
          t.now();
          teensy[tn].send("motv 0 0\n", true);
          // printf("# SMotor:: relax %d\n", relaxing);
        }
        relaxing++;
      }
      toLogMv(t);
    }
    loop++;
    //
    //
    // sleep a little while, the sample time is
    // determined by the encoder (longer than 2ms)
    // actually determined by the Teensy, so on average
    // a constant sample rate (defined in the robot.ini file)
    usleep(500);
  }
  teensy[tn].send("motv 0 0\n", true);
}

void CMotor::tick()
{
  // subscribe if no data
  if ((sub_mpt > 0 and sub_mpt_t.getTimePassed() > 2.5) or (sub_mvt > 0 and sub_mvt_t.getTimePassed() > 2.5))
    subscribeDataFromTeensy();
}
