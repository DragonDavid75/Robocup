/*  
 * 
 * Copyright © 2024 DTU, Christian Andersen jcan@dtu.dk
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


#ifndef UMQTT_H
#define UMQTT_H

#include "MQTTClient.h"

#include "utime.h"


/**
 * Class based on MQTTClient form https://github.com/eclipse/paho.mqtt.c.git
 * installed as ubuntu package
 * sudo apt install libpaho-mqttpp3
 *
 * Using mosquitto, also installed as ubuntu packages:
 * mosquitto         - MQTT version 5.0/3.1.1/3.1 compatible message broker
 * mosquitto-clients - Mosquitto command line MQTT clients
 * mosquitto-dev     - Development files for Mosquitto
 */
class UMqtt
{
public:
  /** setup and request data */
  void setup();
  /**
   * terminate */
  void terminate();
  /**
   * Publish a message
   * \param something like robobot/drive/yaw
   * \param payload a string with parameters in clear text
   * \param qos quality of service: 0: at most once (fast), 1: at least once (resend if fail), 2: exactly once
   */
  bool publish(const char * topic, const char * payload, UTime & msgTime, int qos = 0);
  /**
   * \param topic is something like robobot/drive/t1/mot
   * \param qos, quality of service: 0: at most once (fast), 1: at least once (resend if fail), 2: exactly once
   * \return true (error handling not implemented */
  bool subscribe(const char * topic, int qos);

  bool connected = false;
  /**
   * Reconnect to MQTT */
  void reconnect();


private:
  MQTTClient client = nullptr;
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  MQTTClient_message pubmsg = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token;
  MQTTClient_deliveryToken deliveredtoken;

  static void delivered(void */*context*/, MQTTClient_deliveryToken dt);
  static int msgarrvd(void */*context*/, char *topicName, int /*topicLen*/, MQTTClient_message *message);
  static void connlost(void */*context*/, char *cause);
  int publish_error = 0;
};

/**
 * Make this visible to the rest of the software */
extern UMqtt mqtt;

#endif
