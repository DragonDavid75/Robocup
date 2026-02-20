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
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
#include "umqtt.h"
// create value
SEncoder encoder[NUM_TEENSY_MAX];


void SEncoder::setup(int teensy_number)
{ // ensure default values
  tn = teensy_number;
  ini_section = "encoder" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no data yet, so generate some default values
    ini[ini_section]["interval_vel_ms"] = "8";
    ini[ini_section]["interval_pose_ms"] = "5";
    ini[ini_section]["log_enc"] = "true";
    ini[ini_section]["log_pose"] = "true";
    ini[ini_section]["print"] = "false";
    ini[ini_section]["encoder_reversed"] = "false";
  }
  // reset encoder and pose
  teensy[tn].send("enc0\n");
  topicEnc = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/enc";
  topicEncVel = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/vel";
  topicPose = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/pose";
  // use values and subscribe to source data
  /// subscripe interval
  sub_pose = strtol(ini[ini_section]["interval_pose_ms"].c_str(), nullptr, 10);
  sub_vel = strtol(ini[ini_section]["interval_vel_ms"].c_str(), nullptr, 10);
  // std::string s = "sub pose " + ini[ini_section]["interval_pose_ms"] + "\n";
  // teensy[tn].send(s.c_str());
  // /// subscripe to estimated velocity based on encoder interrupts (time between interrupts)
  // s = "sub vel " + ini[ini_section]["interval_vel_ms"] + "\n";
  // teensy[tn].send(s.c_str());
  /// other debug feature
  toConsole = ini[ini_section]["print"] == "true";
  // ensure default is true if no 'encoder_reversed' entry is available
  // Robobot motors has reversed encoders (encoder A and B is swapped)
  // this will be fixed in the Regbot firmware by this command
  if (ini[ini_section]["log_enc"] == "true" and logfileEnc == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_encoder.txt";
    logfileEnc = fopen(fn.c_str(), "w");
    fprintf(logfileEnc, "%% Encoder logfile\n");
    fprintf(logfileEnc, "%% 1 \tTime (sec)\n");
    fprintf(logfileEnc, "%% 2-3 \tencoder position m1, m2 (ticks)\n");
    fprintf(logfileEnc, "%% 4-5 \tencoder velocity v1, v2 (rad/sec for motor before gear)\n");
    fprintf(logfileEnc, "%% 6 \tencoder posion update count\n");
    fprintf(logfileEnc, "%% 7 \tencoder velocity update count\n");
  }
  if (ini[ini_section]["log_pose"] == "true" and logfilePose == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_pose.txt";
    logfilePose = fopen(fn.c_str(), "w");
    fprintf(logfilePose, "%% Pose logfile\n");
    fprintf(logfilePose, "%% 1 \tTime (sec)\n");
    fprintf(logfilePose, "%% 2,3 \tX, Y position (m)\n");
    fprintf(logfilePose, "%% 4 \tHeading in radians (m)\n");
    fprintf(logfilePose, "%% 5 \tTilt angle, if calculated (rad)\n");
  }
  // allow Teensy to process subscriptions
  usleep(23000);
}


void SEncoder::subscribeDataFromTeensy()
{
  if (sub_pose > 0)
  {
    std::string s = "sub pose " + to_string(sub_pose) + "\n";
    teensy[tn].send(s.c_str());
  }
  if (sub_vel > 0)
  {
    std::string s = "sub vel " + to_string(sub_vel) + "\n";
    teensy[tn].send(s.c_str());
  }
  if (sub_pose > 0 or sub_vel > 0)
  {
    service.logMessage("# SEncoder: subscribed to Teensy data (pose and/or vel)");
    sub_pose_t.now();
    sub_vel_t.now();
  }
  encoder_reversed = false;
  if (ini[ini_section].has("encoder_reversed"))
  { // some motors has swapped HAL sensor A and B in the wiring
    encoder_reversed = ini[ini_section]["encoder_reversed"] == "true";
    const char * s;
    if (encoder_reversed)
      s = "encrev 1\n";
    else
      s = "encrev 0\n";
    teensy[tn].send(s);
  }
}


void SEncoder::terminate()
{
  if (logfileEnc != nullptr)
  {
    fclose(logfileEnc);
  }
  if (logfilePose != nullptr)
  {
    fclose(logfilePose);
  }
}

bool SEncoder::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "enc ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    encTime = msgTime;
    /*double teensyTime =*/ strtod(p1, (char**)&p1);
    enc[0] = strtoll(p1, (char**)&p1, 10);
    enc[1] = strtoll(p1, (char**)&p1, 10);
    // notify users of a new update
    updatePosCnt++;
    // save to log_encoder_pose
    logTime = msgTime;
    toLogEnc();
    // save new value as old value
    encLast[0] = enc[0];
    encLast[1] = enc[1];
  }
  else if (strncmp(p1, "vel ", 4) == 0)
  { // Teensy calculated velocity of wheels (m/s)
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    encVelTime = msgTime;
    /*double teensyTime =*/ strtod(p1, (char**)&p1);
    vel[0] = strtof(p1, (char**)&p1);
    vel[1] = strtof(p1, (char**)&p1);
    // notify users of a new update
    updateVelCnt++;
    // logged in the velocity module
    // after potential additional gear
    sub_vel_t.now();
  }
  else if (strncmp(p1, "pose ", 5) == 0)
  {
    if (strlen(p1) > 5)
      p1 += 5;
    else
      return false;
    poseTime = msgTime;
    /*double teensyTime =*/ strtod(p1, (char**)&p1);
    for (int i = 0; i < 4; i++)
      pose[i] = strtof(p1, (char**)&p1);
    // notify users of a new update
    updatePoseCnt++;
    // save to log_encoder_pose
    logTime = msgTime;
    toLogPose();
    sub_pose_t.now();
  }
  else
    used = false;
  return used;
}

void SEncoder::toLogEnc()
{
  if (not service.stop)
  {
    if (logfileEnc != nullptr and not service.stop_logging)
    {
      fprintf(logfileEnc,"%lu.%04ld %lu %lu %g %g %d %d\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              (unsigned long int)enc[0], (unsigned long int)enc[1],
              vel[0], vel[1], updatePosCnt, updateVelCnt);
    }
    if (toConsole)
    {
      printf("Encoder: %lu.%04ld %lu %lu %g %g %d %d\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              (unsigned long int)enc[0], (unsigned long int)enc[1],
             vel[0], vel[1], updatePosCnt, updateVelCnt);
    }
  }
}

void SEncoder::toLogPose()
{
  if (not service.stop)
  {
    if (logfilePose != nullptr and not service.stop_logging)
    {
      fprintf(logfilePose,"%lu.%04ld %.3f %.3f %.4f %.4f\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              pose[0], pose[1], pose[2], pose[3]);
    }
    if (toConsole)
    {
      printf("Pose: %lu.%04ld %.3f %.3f %.4f %.4f\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              pose[0], pose[1], pose[2], pose[3]);
    }
  }
}


void SEncoder::tick()
{
  if ((sub_pose > 0 and sub_pose_t.getTimePassed() > 1.5) or (sub_vel > 0 and sub_vel_t.getTimePassed() > 1.5))
  {
    subscribeDataFromTeensy();
  }
}

