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
#include "upid.h"
#include "uservice.h"


// PID controller class:
void UPID::setup(float sTime,
                 float proportional,
                 float lead_tau,
                 float lead_alpha,
                 float tau_integrator,
                 float feedForward,
                 float maxu)
{ // ensure there is default values in ini-file
  // get values from ini-file
  kp = proportional;
  taud = lead_tau;
  alpha = lead_alpha;
  // integrator
  taui = tau_integrator;
  useIntegrator = taui > 1e-3;
  ffp = feedForward;
  umax = maxu;
  // sample time from encoder module
  sampleTime = sTime;
  //
  // Calculate PID parameters - see PID function for explanation
  // lead
  useLead = taud > 1e-3;
  if (useLead)
  {
    float lu0 = sampleTime + 2.0 * taud * alpha;
    le0 = (sampleTime + 2.0 * taud)/lu0;
    le1 = (sampleTime - 2.0 * taud)/lu0;
    lu1 = (sampleTime - 2.0 * alpha * taud)/lu0;
  }
  else
  { // no lead/lag
    le0 = 1.0;
    le1 = 0;
    lu1 = 0;
  }
  // integrator
  if (useIntegrator)
    ie = sampleTime/(taui * 2.0);
  else
    ie = 0.0;
  // printf("# UPid::setup usei=%d, ie=%g==%g/(%f*2)\n", useIntegrator, ie, sampleTime, tau_integrator);
  //
}

void UPID::logPIDparams(FILE* logfile, bool andColumns)
{
  fprintf(logfile, "%% PID parameters\n");
  fprintf(logfile, "%% \tKp = %g\n", kp);
  fprintf(logfile, "%% \ttau_d = %g, alpha = %g (use lead=%d)\n", taud, alpha, useLead);
  fprintf(logfile, "%% \ttau_i = %g (used=%d)\n", taui, useIntegrator);
  fprintf(logfile, "%% \tfeed forward = %g\n", ffp);
  fprintf(logfile, "%% \tsample time = %.1f ms\n", sampleTime*1000.0);
  fprintf(logfile, "%% \t(derived values: le0=%g, le1=%g, lu1=%g, ie=%g)\n", le0, le1, lu1, ie);
  if (andColumns)
  { // column description
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tReference for desired value\n");
    fprintf(logfile, "%% 3 \tMeasured value\n");
    fprintf(logfile, "%% 4 \tValue after Kp\n");
    fprintf(logfile, "%% 5 \tValue after Lead\n");
    fprintf(logfile, "%% 6 \tIntegrator value\n");
    fprintf(logfile, "%% 7 \tAfter controller (u)\n");
    fprintf(logfile, "%% 8 \tIs output limited (1=limited)\n");
  }
}


float UPID::pid(float reference, float measurement, bool & limitedNow, float uOffset)
{ // PID controller with minor timing variation allowed
  //
  // error and Kp
  float e = reference - measurement;
  if (angleFolding)
  {
    if (e > M_PI)
      e -= 2.0 * M_PI;
    else if (e < -M_PI)
      e += 2.0 * M_PI;
  }
  float ep0 = e * kp;
  /** lead filter
   * Pole - Zero 1st order filter (Lead)
   * u(s)/e(s) = (td * s + 1)/(al*td*s + 1);
   * Translate to z (Tustin) using:
   *     2(z - 1)
   * s = -------- (and divide with z in numerator and denominator)
   *     T(z + 1)
   *
   * u(z)   (T + 2 td) + (T - 2 td)z^-1
   * ---- = ----------------------------------
   * e(z)   (T + 2 td al) + (T - 2 td al) z^-1
   *
   * u0(T + 2 td al) = (T + 2 td) e0 + (T - 2 td) e1 - (T - 2 td al) u1
   * where u0 is new value, u1 is T seconds old (same with e).
   * All brackets can be pre-calculated, as
   * u0 * lu0 = za * e0 + zb * e1 - zc * u1
   * <=>
   * u0 = za/lu0 * e0 + zb/lu0 * e1 - zc/lu0 * u1
   * or with new constants
   * u0 = le0 * e0 + le1 * e1 - lu1 * u1
   * */
  float up0 = le0 * ep0 + le1 * ep1 - lu1 * up1;
  /**
   * Integrator
   * u(s)/e(s) = 1/(ti*s);
   *
   * u(z)   T * (1 + z^-1)
   * ---- = ----------------
   * e(z)   ti * 2 * (1 - z^-1)
   *
   * u0(ti*2) = T*e0 + T*e1 + ti*2*u1
   * or
   * u0 = T/(ti*2)*(e0 + e1) + u1
   * or
   * u0 = ie*(e0 + e1) + u1
   * */
  float ui0;
  if (limited or not useIntegrator)
    // do not integrate further (integrator limiter)
    ui0 = ui1;
  else
    ui0 = ie * (up0 + up1) + ui1;
  // sum the integrated value with the PD value
  u = ui0 + up0 + ffp * reference + uOffset;
  if ( u > umax)
  {
    u = umax;
    limited = true;
  }
  else if ( u < -umax)
  {
    u = -umax;
    limited = true;
  }
  else
    limited = false;
  limitedNow = limited;
  // save as old values for next iteration
  ep1 = ep0;
  ui1 = ui0;
  up1 = up0;
  r = reference;
  m = measurement;
  //
  return u;
}

void UPID::resetHistory()
{
  ep1 = 0;
  ui1 = 0;
  up1 = 0;
}


void UPID::saveToLog(FILE* logfile, UTime t)
{// log_pose
  if (logfile != nullptr and not service.stop_logging)
  {
    fprintf(logfile, "%lu.%04ld %.3f %.3f %.3f %.3f %.3f %.3f %d\n",
            t.getSec(), t.getMicrosec()/100,
            r, m,
            ep1,
            up1,
            ui1,
            u,
            limited
    );
  }
  if (toConsole)
  {
    printf("%lu.%04ld %.3f %.3f %.3f %.3f %.3f %.3f %d\n",
            t.getSec(), t.getMicrosec()/100,
            r, m,
            ep1,
            up1,
            ui1,
            u,
            limited
    );
  }
}


