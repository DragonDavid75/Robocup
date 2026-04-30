/* #***************************************************************************
 #*   Copyright (C) 2017-2023 by DTU
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


#ifndef SREGBOT_H
#define SREGBOT_H

#include <stdio.h>
#include <mutex>
#include <queue>
#include <thread>
#include <string.h>
#include <string>

#include "utime.h"

#define NUM_TEENSY_MAX 1

/**
 * Queue class for messages that require confirmation
 *  */
class UOutQueue
{
public:
  static const int MML = 400;
  char msg[MML];
  int len;
  bool isSend = false;
  UTime queuedAt;
  UTime sendAt;
  int resendCnt;
  int tn = 0;
  /**
   * Constructor */
  UOutQueue(const char * msg)
  {
    setMessage(msg);
    queuedAt.now();
    isSend = false;
    resendCnt = 0;
  }
  /**
   * set new message */
  bool setMessage(const char* message);
  /**
   * Confirm a match */
  bool compare(const char * got)
  { // ignore potential \r\n
    int n = strlen(got) - 2;
    bool equal = strncmp(&msg[3], got, n) == 0;
    return equal;
  }
};


/**
 * The robot class handles the 
 * port to the REGBOT part of the robot,
 * REGBOT handles the most real time issues
 * and this class is the interface to that. */
class STeensy //: public URun, public USource
{ // REGBOT interface
public:
  // teensy interface number
  int tn = 0; // Teensy number
  /// Is port opened successfully
  bool teensyConnectionOpen = false;
  // mission state from hbt 
  int missionState = 0;
  // reference time
  UTime justConnectedTime;
  // flag to allocate a number (and robobot type) to the Teensy (Regbot)
  // must be in range [0..149]
  int saveRegbotNumber = -1;
  // save regbot hardware type to regbot if number is [5..15]
  // should typically be 9 (blue pcb version 6.3)
  int regbotHardware = -1;
  // all used motors has encoder (A,B) reversed.
  bool encoderReversed = true;
  // is this Teensy connection disabled (else should be active)
  bool disabled = false;

  
private:
  // serial port handle
  std::string ini_section;
  int usbport = -1;
  // serial port (USB)
//   int usbdeviceNum = 0;
  // simulator hostname
//   const char * simHost;
  // simulator port
//   int simPort = 0;
  // mutex to ensure commands to regbot is not mixed
//   mutex txLock;
//   mutex logMtx;
  std::mutex eventUpdate;
  std::mutex sendLock;
  // receive buffer
  static const int MAX_RX_CNT = 1000;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  //
  UTime lastTxTime;
  // socket to simulator
//   tcpCase socket;
  /**
   * communication count */
  int gotCnt = 0;
  int sendCnt = 0;
  /** interface just opened */
  bool justConnected = false;
  bool confirmSend = false;
//   bool sendDirectFromNowOn = false;

  std::thread * th1;

  
public:
  /**
   * Set device */
  void setup(int teensyNumber);
  /**
   * terminate */
  void terminate();
  /**
   * Send a string to the serial port (Teensy) through the queue.
   * Anything send through the queue is confirmed and resend 3 times
   * if no confirm is received. (would take 3 times 30ms)
   * for streaming use then send directly, setting direct=true)
   * \param message is c_string to send,
   * \param direct for bypassing the default message queue
   * \returns true if send direct and delivered OK */
  bool send(const char * message, bool direct = false);
  /**
   * runs the receive thread 
   * This run() function is called in a thread after a start() call.
   * This function will not return until the thread is stopped. */
  void run();
  /**
  * decode commands potentially for this device */
  bool decode(const char* msg, UTime & msgTime);
  /** Generate 3 character CRC as ";XX", where
   * NN is sum of character value modulus 99 + 1.
   * Only characters with a value c>' ' counts
   * @param cmd is message string to generate CRC from
   * @param rcr is a string of (at least) 4 characters, where the result is returned.
   * @returns true is message ends with a '\n' */
  static bool generateCRC(const char * cmd, char * crc);
  /**
   * Get Teensy communication errors */
  int getTeensyCommError(int & retryCnt);
  /**
   * get messages queued, but not send */
  int getTeensyCommQueueSize();
  /**
   * Data flow check */
  void tick();

private:
  /**
   * queue a message
   * @param message  */
  void sendToQueue(const char* message);
  /**
   * send this message directly to the Teensy port */
  bool sendDirect(const char* message);
  /**
   * Check for crc error
   * \param rawMsg is the message preceded by crc
   * \return true if OK */
  bool crcCheck(const char * rawMsg);
  /**
   * is data source active (is device open) */
  virtual bool isActive()
  {
    return (usbport >= 0) and gotActivityRecently and not justConnected;
  }

  static void runObj(STeensy * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }

private:
  /**
   * Open the connection.
   * \returns true if successful */
  bool openToTeensy();
  std::string robotName;
  int confirm_timeout_ms = 100;
  /**
   * A confirm message is received,
   * Check, and
   * release the next in the queue */
  void messageConfirmed(const char * confirm);
  void closeUSB();
  int connectErrCnt = 0;
  ///
  bool gotActivityRecently = true;
  UTime lastRxTime;
  std::string usbDevName;
  int alternativeDevice = 0;
  int deviceIsBuzyCnt = 0;
  bool initialized = false;
  bool stopUSB = false;
  UTime lastSent;
  /**
   * uotgoing message queue */
  std::queue<UOutQueue> outQueue;
  float confirmTimeout = 0.03; // timeout in seconds for writing to Teensy
  // transmission statistics
  int confirmMismatchCnt = 0;
  int confirmRetryCnt = 0;
  /// number of retry attempts before drop
  int confirmRetryCntMax = 50;
  /// count of dropped messages requiring confirm
  int confirmRetryDump = 0;
  /// save in log with different time + marking
  void toLog(const char * msg);
  void toLogRx(const char*, UTime& mt);
  void toLogTx();
  void toLogQu();
  /// should logged messages be printed on console too.
  bool toConsole = false;
  /// data io logfile
  FILE * logfile = nullptr;
  std::mutex dataLock; // ensure consistency
  //
  // MQTT
  std::string topicBase;
  std::string topicDName;
  std::string topicHelp;
  //
  void subscribeDataFromTeensy();
  UTime sub_dname_t;
};

extern STeensy teensy[NUM_TEENSY_MAX];

#endif
