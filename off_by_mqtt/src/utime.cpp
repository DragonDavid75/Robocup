 /* #***************************************************************************
 #*   Copyright (C) 2006-2023 by DTU
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>
#include <math.h>
#include "utime.h"

/////////////////////////////////////////

UTime::UTime()
{
  clear();
}

UTime::UTime(const char *)
{
  now();
}

/////////////////////////////////////////////

UTime::~UTime()
{
}

/////////////////////////////////////////

void UTime::clear()
{ // clear to zero
  time.tv_sec = 0;
  time.tv_usec = 0;
  valid = false;
}

unsigned long UTime::getSec()
{
  if (valid)
    return time.tv_sec;
  else
    return 0;
}

/////////////////////////////////////////

float UTime::getDecSec()
{
  if (valid)
    return float(time.tv_sec) + float(time.tv_usec) * 1e-6;
  else
    return 0;
}

/////////////////////////////////////////

float UTime::getDecSec(UTime t1)
{ // get time compared to t1
  return float(time.tv_sec - t1.time.tv_sec) + float(time.tv_usec - t1.time.tv_usec) * 1e-6;
}

/////////////////////////////////////////

float UTime::getTimePassed()
{
  UTime t;
  t.now();
  return (t - *this);
}

/////////////////////////////////////////

long UTime::getMilisec()
{
  if (valid)
    return time.tv_usec / 1000;
  else
    return 0;
}

///////////////////////////////////////////////

unsigned long UTime::getMicrosec()
{
  if (valid)
    return time.tv_usec;
  else
    return 0;
}

/////////////////////////////////////////////

int UTime::getTimeAsString(char * info, bool local)
{ // writes time to string in format "hh:mm:ss.msec"
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%2d:%02d:%02d.%03d", ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return strlen(info);
}

//////////////////////////////////////////

char * UTime::getForFilename(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d%02d%02d_%02d%02d%02d.%03d",
            ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
            ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return info;
}

std::string UTime::getForFilename()
{
  const int MSL = 100;
  char s[MSL];
  std::string d = getForFilename(s, true);
  return d;
}

//////////////////////////////////////////

char * UTime::getDateTimeAsString(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
          ymd.tm_hour,
          ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return info;
}


//////////////////////////////////////////

void UTime::setTime(timeval iTime)
{
  time = iTime;
  valid = true;
}

/////////////////////////////////////////

void UTime::setTime(long sec, long uSec)
{
  time.tv_sec = sec;
  time.tv_usec = uSec;
  valid = true;
}

/////////////////////////////////////////

struct tm UTime::getTimeTm(bool local)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  return ymd;
}

/////////////////////////////////////////////

UTime UTime::operator+ (float seconds)
{
  UTime t = *this;
  t.add(seconds);
  return t;
}

/////////////////////////////////////////////

UTime UTime::operator- (float seconds)
{
  UTime t = *this;
  t.sub(seconds);
  return t;
}

/////////////////////////////////////////////

void UTime::add(float seconds)
{
  float rem = seconds - long(seconds);
  long remL = (unsigned long)(rem * 1000000.0);
  //
  time.tv_sec += (unsigned long)seconds;
  if (time.tv_usec + remL > 1000000)
  { // adjust if overflow
    time.tv_sec++;
    time.tv_usec += remL - 1000000;
  }
  else
    time.tv_usec += remL;
}

void UTime::sub(float seconds)
{
  float rem = seconds - long(seconds);
  long remL = (unsigned long)(rem * 1000000.0);
  //
  time.tv_sec -= (unsigned long)seconds;
  if (time.tv_usec < remL)
  { // adjust if overflow
    time.tv_sec--;
    time.tv_usec += 1000000 - remL;
  }
  else
    time.tv_usec -= remL;
}
/////////////////////////////////////////////



