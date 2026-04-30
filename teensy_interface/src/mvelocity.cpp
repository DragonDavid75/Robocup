/*  
 * 
 * Copyright © 2023 DTU,
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
#include <thread>
#include <math.h>
#include "sencoder.h"
#include "mvelocity.h"
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
#include "cmixer.h"
#include "umqtt.h"

// create value
MVelocity mvel[NUM_TEENSY_MAX];


void MVelocity::setup(int teensyNumber)
{ // ensure there is default values in ini-file
  tn = teensyNumber;
  ini_section = "velocity" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no data yet, so generate some default values
    ini[ini_section]["encTickPerRev"] = "68";
    ini[ini_section]["motorScale"] = "1 1";
    ini[ini_section]["useTeensyVel"] = "true";
    ini[ini_section]["log"] = "true";
    ini[ini_section]["print"] = "false";
  }
  // Mqtt topic names
  topicVel = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/mvel";
  // get values from ini-file
  encTickPerRev = strtol(ini[ini_section]["encTickPerRev"].c_str(), nullptr, 10);
  radPerTick = 2.0 * M_PI  / encTickPerRev;
  // scaling set to 1:1 if no additional gear
  const char * p1 = ini[ini_section]["motorScale"].c_str();
  for (int i = 0; i < SRobot::MAX_MOTORS; i++)
    motorScale[i] = strtof(p1, (char **)&p1);
  //
  useTeensyVelEstimate = ini[ini_section]["useTeensyVel"] != "false";
  string encIni = "encoder" + std::to_string(tn);
  const char *  encSampleTime;
  if (useTeensyVelEstimate)
    encSampleTime = ini[encIni]["interval_vel_ms"].c_str();
  else
    encSampleTime = ini[encIni]["interval_pos_ms"].c_str();
  // find sample time in seconds
  sampleTime = strtof(encSampleTime, nullptr) * 0.001;
  toConsole = ini[ini_section]["print"] == "true";
  if (ini[ini_section]["log"] == "true" and logfile == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_encoder_velocity.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Pose and velocity (%s) for Teensy %d\n", fn.c_str(), tn);
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2-3 \tVelocity motor 1..2 (m/s or rad/s) m/s if use Teensy, else rad/sec motor vel, see robot.ini\n");
    fprintf(logfile, "%% 4-5 \tUpdate number (encoder, velocity) - mostly debug\n");
  }
  if (th1 == nullptr)
    th1 = new std::thread(runObj, this);
}


void MVelocity::terminate()
{ // wait for thread to finish
  if (th1 != nullptr)
  {
    th1->join();
    th1 = nullptr;
  }
}


void MVelocity::run()
{
//   printf("# MVelocity::run started\n");
  int loop = 0;
  int64_t encLast[SRobot::MAX_MOTORS] = {0};
  UTime t("now"); // time of update
  UTime encTimeLast[SRobot::MAX_MOTORS];
  for (int e = 0; e < SRobot::MAX_MOTORS; e++)
    encTimeLast[e].now();
  float dd[SRobot::MAX_MOTORS]; // motor moved since last update
  int encup; // pos update
  int encuv; // velocity update
  bool updated = false;
  while (not service.stop)
  { // there is an update - encoder or velocity
    encup = encoder[tn].updatePosCnt;
    encuv = encoder[tn].updateVelCnt;
    if (encup != oldEncUpdate and not useTeensyVelEstimate)
    { // new encoder update - this actually calculates
      // the motor velocity, and not the wheel velocity
      int64_t enc[SRobot::MAX_MOTORS]; // shorthand value
      for (int i = 0; i < SRobot::MAX_MOTORS; i++)
      { // get value
        enc[i] = encoder[tn].enc[i];
        if ( encup < 2)
        { // first two updates take last value as current
          for (int e = 0; e < SRobot::MAX_MOTORS; e++)
            encLast[e] = enc[e]; // left
        }
      }
      t = encoder[tn].encTime;
      float dtt = 1.0; // in seconds - for turnrate
      float dt[SRobot::MAX_MOTORS];
      int64_t de[SRobot::MAX_MOTORS];
      for (int i = 0; i < SRobot::MAX_MOTORS; i++)
      { // find movement in time and distance for each wheel
        dt[i] = t - encTimeLast[i]; // time
        if (dt[i] < dtt)
        { // the minimum update time (the other wheel may be stationary)
          dtt = dt[i];
        }
        de[i] = enc[i] - encLast[i];
        if (llabs(de[i]) > 1000)
        { // given up in calculating folding around MAXINT,
          // so one sample of zero change should be OK.
          de[i] = 0;
        }
        // distance traveled since last
        dd[i] = float(de[i]) * radPerTick; // encoder ticks
        if (enc[i] != encLast[i])
        { // wheel has moved since last update
          encLast[i] = enc[i];
          encTimeLast[i] = t;
          motorVel[i] = dd[i]/dt[i] * motorScale[i];
          updated = true;
        }
        else if (fabsf(motorVel[i]) > 0.001)
        { // no tick change since last update
          // update (reduce) velocity waiting for next tick
          motorVel[i] = copysignf(1.0, motorVel[i]) * radPerTick/dt[i] * motorScale[i];
          updated = true;
        }
      }
      if (updated)
      { // publish and log if there is a change only
        // maybe fixed rate would be better?
        updateCnt++;
        velTime.now();
      }
      oldEncUpdate = encup;
    }
    else if (encuv != oldEncVelUpdate and useTeensyVelEstimate)
    { // use wheel velocity already calculated by the Teensy
      for (int i = 0; i < SRobot::MAX_MOTORS; i++)
      { // get value
        motorVel[i] = encoder[tn].vel[i] * motorScale[i]; // m/s
      }
      velTime = encoder[tn].encVelTime;
      updateCnt++;
      oldEncVelUpdate = encuv;
      updated = true;
    }
//     else if (tn == 0)
//       printf("# MVelocity:: tn=%d, loop %d, updCnt %d, encup %d, encuv %d, old=%d, usev %d, usevel = '%s'\n", tn, loop,
//              updateCnt, encup, encuv, oldEncUpdate, useTeensyVelEstimate,
//              ini[ini_section]["useTeensyVel"].c_str());
    if (updated)
    { // finished making a new pose
      if (ini["mqtt"]["use"] == "true")
      {
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "%.3f %.3f\n",
                motorVel[0], motorVel[1]);
        UTime t("now");
        // topic robobot/drive/T0/mvel
        mqtt.publish(topicVel.c_str(), s, t);
      }
      toLog();
      updated = false;
    }
    // just wait a bit (1ms)
    usleep(1000);
    loop++;
  }
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}


void MVelocity::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr and not service.stop_logging)
    { // log_pose
      fprintf(logfile, "%lu.%04ld %.4f %.4f %d %d\n",
              velTime.getSec(), velTime.getMicrosec()/100,
              motorVel[0], motorVel[1], oldEncUpdate, oldEncVelUpdate
              );
    }
    if (toConsole)
    { // print_pose
      printf("# motor velocity (%d) %lu.%04ld %.4f %.4f\n",
             tn,
             velTime.getSec(), velTime.getMicrosec()/100,
              motorVel[0], motorVel[1]
              );
    }
  }
}
