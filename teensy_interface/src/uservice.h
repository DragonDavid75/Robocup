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

#pragma once

#include <string>
#include <thread>
#include "utime.h"
#include "uini.h"


class UService
{
public:
    /**
     * Initialize all message data items
     * \returns true if app is to end now (error, help or calibration)
    */
    bool setup(int argc,char **argv);
    /**
     * decode messages from Teensy
     * \param msg already CRC checked text line from teensy
     * \param msgTime is time of arrival of the message
     * \param tn is the Teensy interface number.
     * \returns true, if the message was used. */
    bool decode(const char * msg, UTime & msgTime, int tn);
    /**
     * decode MQTT message to be split to either Teensy or a more
     * abstract message handler
     * \param topic is something like /scorpi/drive/t2/motv
     * \param payload could be "3.3 3.3 0.2"
     * \param msgTime is time of arrival */
    bool mqttDecode(const char* topic, const char * payload, UTime& msgTime);

    /**
     * decode command-line parameters */
    bool readCommandLineParameters(int argc, char ** argv);
    /**
     * Stop application - like an interrupt
     * \param who is a string to identify from where the stop order came
     * */
    void stopNow(const char * who);
    /**
     * Full system should power off */
    void power_off_request(bool fromTeensy, const char * who);
    /**
     * shut down and save ini-file */
    void terminate();
    /**
    * thread to listen to keyboard */
    void run();
    // void run2(); // and terminate request
    /**
     * Got keyboard input - e.g. enter */
    // bool gotKey();
    /**
     * Return the SVN version string (version part) */
    std::string getVersionString();
    /**
     * used battery capacity in Wh */
    float usedBatteryCapacity();
    /**
     * run a bash command and get console output */
    std::string exec(std::string cmd);
    bool teensyFileFree = false;
    /**
     * Check number of times this name exist in the process list
     * \param name is the process name or part of a name to check.
     * \returns the number of times this mane occur in the process list */
    int isThisProcessRunning(std::string name);
    /**
     * Do all the Teensy releated setup */
    void setupTeensyConnection();
    /**
     * put message into logfile */
    void logMessage(const char * msg);

public:
    // file with calibration values etc.
    std::string iniFileName = "robot.ini";
    mINI::INIFile * iniFile;
    std::string logPath = ""; // this directory
    FILE * logfile = nullptr;
    // stop all processing
    bool stop = false;
    bool theEnd;
    bool stopNowRequest = false;
    bool powerOffRequested = false;
//     bool start = false;
    // keyboard input
    bool gotKeyInput;
    std::string keyString;
    bool asDaemon = false;
    //
    bool stop_logging = false;
    float maxLogMinutes = 60.0; // set in robot.ini
    UTime startedLogging; // system time
    float app_time = 0; // seconds since start of app
    bool setupComplete = false;

private:
    static void runObj(UService * obj)
    { // called, when thread is started
        // transfer to the class run() function.
        obj->run();
    }
    std::thread * th1;
    //
    bool terminating = false;
    bool cliAction = false;
    bool flushLog = false;
    bool GetLineFromCin();
    static const int MKL = 100;
    char keyLine[MKL];
    int keyLineIdx = 0;
    UTime lastMqttMessage;
    //
    static const int MID = 100;
    char masterAliveID[MID];
    int masterAliveCnt = 0;
    UTime masterAliveTime;
    int masterAliveErr = 0;
    //
    std::string topicMaster;
};

extern UService service;
extern mINI::INIStructure ini;
