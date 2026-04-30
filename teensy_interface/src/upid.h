/*  
 * 
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
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


#ifndef UPID_H
#define UPID_H

#include "utime.h"

using namespace std;
// forward declaration

class UPID{
  
public:
  /** controller setup */
  void setup(float sTime, /// sample time (sec)
             float proportional, /// Kp
             float lead_tau,     /// lead time constant (sec) = 1/(w_m*sqrt(alpha))
             float lead_alpha,
             float tau_integrator,
             float feedForward,
             float maxu);
  /**
   * PID controller
   * \param reference is the set-point reference
   * \param measurement is the current measured value
   * \param limitingIsActive if true, then a potential integrator stops integrating
   * \returns the calculated control value
   * */
  float pid(float reference, float measurement, bool & limitedNow, float uOffset);
  /**
   * when restarting control, it is important to
   * reset the control history */
  void resetHistory();
  /**
   * save PID parameters to this logfile */
  void logPIDparams(FILE * logfile, bool andColumns);
  /**
   * Sage the current control values to this logfile
   * \param logfile is a valid file handle
   * \param t is the time where the values are valid
   * */
  void saveToLog(FILE * logfile, UTime t);
  /**
   * reference and measurement may be in radians
   * ensure correct folding of angles. */
  inline void doAngleFolding(bool doFolding)
  {
    angleFolding = doFolding;
  }

protected:
  /** velocity controller - left and right
   * PID values */
  float kp;    // proportional gain (u/error)
  float taui;  /// time constant for integrator (sec)
  float taud;  /// time constant for lead zero (sec)
  float alpha; /// ratio between pole and zero time constant for lead
  float ffp;   /// feed forward proportional (u/ref)
  /// controller output limit
  float umax;  /// maximum allowed u value
  bool angleFolding = false; // Set to true is ref is an angle in radians (+/- Pi)
  //
public:
  // is output limited, this may be valuable for other controllers.
  bool limited = false;
  // should be printed on console during run (debug feature)
  bool toConsole = false;

protected:
  /// more private internal values
  float r = 0, m = 0;
  float sampleTime;
  /// old values for PID
  float ep1 = 0, up1 = 0, ui1 = 0;
  /// pre-calculated lead values
  float le0, le1, lu1;
  /// pre-calculated integrator values
  float ie;
  // controller output
  float u = 0;
  //
  bool useIntegrator = false;
  bool useLead = false;
};


#endif
