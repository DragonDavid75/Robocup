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
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "uservice.h"
#include "umqttin.h"

using namespace std::chrono;

// create object
UMqttIn mqttin;

// #define ADDRESS     "tcp://localhost:1883"
// #define CLIENTID    "Joe"
// #define TOPIC       "acc"
// #define PAYLOAD     "1.0111 0.222 0.333"
// #define QOS         1
// #define TIMEOUT     10000L

void UMqttIn::setup()
{ // ensure default values
  if (not ini.has("mqttin"))
  { // no data yet, so generate some default values
    ini["mqttin"]["broker"] = "tcp://localhost:1883"; // IP and port to broker
    ini["mqttin"]["context"] = "drive"; // not used
    ini["mqttin"]["clientid"] = "tif_in"; // MQTT connection name
    ini["mqttin"]["function"] = "cmd/"; // drive/ or crane/. Function of this process
    ini["mqttin"]["system"] = "robobot/"; // top name in MQTT topic
    ini["mqttin"]["log"] = "true";
    ini["mqttin"]["print"] = "false";
    ini["mqttin"]["use"] = "true";
  }
  if (ini["mqttin"]["print"] == "true")
  // logfiles
  toConsole = ini["mqttin"]["print"] == "true";
  if (ini["mqttin"]["log"] == "true" and logfile == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_mqtt_in.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% mqtt logfile, enabled=%s\n", ini["mqttin"]["use"].c_str());
    fprintf(logfile, "%% broker=%s\n", ini["mqttin"]["broker"].c_str());
    fprintf(logfile, "%% context=%s\n", ini["mqttin"]["context"].c_str());
    fprintf(logfile, "%% client id=%s\n", ini["mqttin"]["clientid"].c_str());
    fprintf(logfile, "%% system=%s (top level ID)\n", ini["mqttin"]["system"].c_str());
    fprintf(logfile, "%% function=%s (next level name)\n", ini["mqttin"]["function"].c_str());
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tQuality of service 0=max once, 1=at least once, 2=once, S=subscribe\n");
    // fprintf(logfile, "%% 3 \tTx = published, Rx = received, Su = subscribe\n");
    fprintf(logfile, "%% 3 \t'Topic'\n");
    fprintf(logfile, "%% 4 \tMessage / payload\n");
  }
  // MQTT
  if (ini["mqttin"]["use"] == "true" and not connected)
  { // MQTT enabled
    int rc;
    int connectCnt = 0;
    bool isOK = false;
    rc = MQTTClient_create(&client,
                            ini["mqttin"]["broker"].c_str(),
                            ini["mqttin"]["clientid"].c_str(),
                            MQTTCLIENT_PERSISTENCE_NONE,
                            NULL);
    isOK = rc == MQTTCLIENT_SUCCESS;
    if (not isOK)
    {
      printf("# UMqttIn:: Failed to create client, return code %d\n", rc);
  //     rc = EXIT_FAILURE;
    }
    if (isOK)
    {
      rc = MQTTClient_setCallbacks(client, (void*)ini["mqttin"]["context"].c_str(), connlost, msgarrvd, delivered);
      isOK = rc == MQTTCLIENT_SUCCESS;
      if (not isOK)
      {
        printf("# UMqttIn:: Failed to set callbacks, return code %d\n", rc);
  //       rc = EXIT_FAILURE;
  //       goto destroy_exit;
      }
    }
    if (isOK)
    {
      conn_opts.keepAliveInterval = 20;
      conn_opts.cleansession = 1;
      while (connectCnt < 15)
      {
        rc = MQTTClient_connect(client, &conn_opts);
        isOK = rc == MQTTCLIENT_SUCCESS;
        if (isOK)
          break;
        connectCnt++;
        sleep(1);
        printf("# UMqttIn:: Waiting for (cmd) Mosquitto (%d)\n", connectCnt);
      }
      if (not isOK)
      {
        printf("# UMqttIn:: Failed to connect for incoming commands, return code %d\n", rc);
  //       rc = EXIT_FAILURE;
  //       goto destroy_exit;
        if (logfile != nullptr and not service.stop_logging)
        {
          logLock.lock();
          UTime t("now");
          fprintf(logfile,"%lu.%04ld Failed (%d) to connect to to MQTT server\n",
                  t.getSec(), t.getMicrosec()/100, rc);
          logLock.unlock();
        }
      }
    }
    if (isOK)
    {
      printf("# UMqttIn:: connection to MQTT broker on %s established\n", ini["mqttin"]["broker"].c_str());
      service.logMessage("# Mqtt cmd connection established");
      // subscribe
      std::string topic = ini["mqttin"]["system"] + ini["mqttin"]["function"] + "#";
      subscribe(topic.c_str(), 1);
    }
    //end MQTT

    // if (isOK and not service.stop)
    //   // start listen to the keyboard
    //   th1 = new std::thread(runObj, this);
    connected = isOK;
  }
  else
    printf("# UMqttIn:: disabled in robot.ini\n");
}


void UMqttIn::reconnect()
{
  int connectCnt = 0;
  bool isOK = false;
  int rc = 0;
  UTime t("now");
  const int MSL = 50;
  char s[MSL];
  printf("# %lu.%04ld MQTT (cmd) reconnect (%s)\n",
         t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
  if (logfile != nullptr and not service.stop_logging)
  {
    logLock.lock();
    fprintf(logfile,"%lu.%04ld Reconnecting %s\n",
            t.getSec(), t.getMicrosec()/100, s);
    logLock.unlock();
  }
  //
  if (connected)
  { // try reconnect
    rc = MQTTClient_disconnect(client, 10000);
    if (rc != MQTTCLIENT_SUCCESS)
    {
      printf("# UMqtt:: (cmd) Failed to disconnect, return code %d\n", rc);
      rc = EXIT_FAILURE;
    }
  }
  //
  while (connectCnt < 15)
  {
    rc = MQTTClient_connect(client, &conn_opts);
    isOK = rc == MQTTCLIENT_SUCCESS;
    if (isOK)
      break;
    connectCnt++;
    sleep(1);
    printf("# UMqtt:: (cmd) Waiting for Mosquitto (%d)\n", connectCnt);
  }
  if (not isOK)
  {
    printf("# UMqtt:: (cmd) Failed to connect, return code %d\n", rc);
    //       rc = EXIT_FAILURE;
    //       goto destroy_exit;
    if (logfile != nullptr)
    {
      UTime t("now");
      fprintf(logfile,"%lu.%04ld Failed (%d) to connect to to MQTT server at %s\n",
              t.getSec(), t.getMicrosec()/100, rc, t.getDateTimeAsString(s, true));
    }
  }
  else
  {
    printf("# %lu.%04ld MQTT (cmd) reconnected (%s)\n",
           t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
    // subscribe("robobot/cmd/shutdown", 1);
    // subscribe
    std::string topic = ini["mqttin"]["system"] + ini["mqttin"]["function"] + "#";
    subscribe(topic.c_str(), 1);
  }
  connected = isOK;
}

void UMqttIn::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
  if (client != nullptr)
  {
    int rc = MQTTClient_disconnect(client, 10000);
    if (rc != MQTTCLIENT_SUCCESS)
    {
      printf("# UMqttIn:: Failed to disconnect, return code %d\n", rc);
      rc = EXIT_FAILURE;
    }
    MQTTClient_destroy(&client);
    printf("# UMqttIn:: client shutdown\r\n");
  }
}

bool UMqttIn::subscribe ( const char* topic, int qos )
{
  MQTTClient_subscribe(client, topic, qos);
  if (logfile != nullptr and not service.stop_logging)
  {
    logLock.lock();
    UTime t("now");
    fprintf(logfile, "%lu.%04ld S %d %s\n",
    t.getSec(), t.getMicrosec()/100,
            qos, topic);
    logLock.unlock();
  }
  return true;
}

void UMqttIn::toLogRx(const char* context, const char* topic, char * message, int qos, UTime t, bool /*used*/)
{ // pv is pin-value
  if (service.stop)
    return;
  if (logfile != nullptr and not service.stop_logging)
  {
    logLock.lock();
    fprintf(logfile,"%lu.%04ld %d %s %s %s\n",
            t.getSec(), t.getMicrosec()/100,
            qos, context, topic, message);
    logLock.unlock();
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %s %s %s\n",
            t.getSec(), t.getMicrosec()/100,
           qos, context, topic, message);
  }
}


void UMqttIn::delivered(void * /*context*/, MQTTClient_deliveryToken dt)
{
  //printf("# MQTT %s Message with token value %d delivery confirmed\n", (char*) context,  dt);
  mqttin.deliveredtoken = dt;
}

int UMqttIn::msgarrvd(void *context, char *topicName, int /*topicLen*/, MQTTClient_message *message)
{
  if (false)
  { // debug
    printf("# MQTT Message arrived\n");
    if (context != nullptr)
      printf("#   context: %s\n", (char *)context);
    printf(  "#     topic: %s\n", topicName);
    printf(  "#     length: %d\n", message->payloadlen);
    printf(  "#   message: %.*s\n", message->payloadlen, (char*)message->payload);
  }
  //
  UTime t("now");
  const int MSL = 200;
  char s[MSL];
  int n = 200;
  if (message->payloadlen > MSL - 1)
    n = MSL-1;
  else
    n = message->payloadlen;
  strncpy(s, (char*)message->payload, n);
  s[n] = '\0';
  bool used = service.mqttDecode(topicName, s, t);
  mqttin.toLogRx((char*)context, topicName, s, message->qos, t, used);
  //
  MQTTClient_freeMessage(&message);
  MQTTClient_free(topicName);
  return 1;
}

void UMqttIn::connlost(void */*context*/, char *cause)
{
  UTime t("now");
  char s[50];
  printf("# %lu.%04lu UMqttIn:: Connection lost (rc=%s) at %s\n", t.getSec(), t.getMicrosec()/100, cause, t.getDateTimeAsString(s));
}



bool UMqttIn::publish(const char * topic, const char * payload, UTime & msgTime, int qos)
{
  if (ini["mqttin"]["use"] != "true")
    // MQTT disabled in robot.ini
    return false;
  if (not connected)
    // mosquitto is probably not found
    return false;
  if (strlen(topic) < 5)
    // no topic
    return false;
  mqttPublishLock.lock();  //
  //const auto mtime{system_clock::now()};
  //auto ms_since_epoch = duration_cast<milliseconds>(mtime.time_since_epoch());
  // time since 1 January 1970 in seconds from system clock
  //double mtime = duration<double>(system_clock::now().time_since_epoch()).count();
  //std::cout << "# msg time " << ms_since_epoch << "\n";
  // add timestamp
  const int MSL = 2000;
  char s[MSL];
  snprintf(s, MSL-1, "%lu.%04ld %s", msgTime.getSec(), msgTime.getMicrosec()/100, payload);
  //printf("# MQTT publish topic %s payload %s", topic, s);
  pubmsg.payload = (void*)s;
  pubmsg.payloadlen = (int)strlen(s);
  pubmsg.qos = qos;
  pubmsg.retained = 0;
  deliveredtoken = 0;
  int rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
  char * nl = strchrnul(s, '\n');
  if (*nl != '\n')
  { // ensure string ends at \n (newline) for logfile
    *nl++ = '\n';
    *nl = '\0';
  }
  if (rc != MQTTCLIENT_SUCCESS)
  {
    if (logfile != nullptr and not service.stop_logging)
    {
      logLock.lock();
      UTime t("now");
      fprintf(logfile,"%lu.%04ld Fail to publish (%d, %d) %d, topic '%s', payload: %s",
              t.getSec(), t.getMicrosec()/100, rc, publish_error,
              qos, topic, s);
      logLock.unlock();
    }
    publish_error++;
    if (publish_error > 2)
      reconnect();
    //printf("Failed to publish message, return code %d\n", rc);
  }
  else
  { // wait for message to be delivered (for quality services only).
    //
    if (logfile != nullptr and not service.stop_logging)
    {
      logLock.lock();
      UTime t("now");
      fprintf(logfile,"%lu.%04ld %d '%s' %s",
              t.getSec(), t.getMicrosec()/100,
              qos, topic, s);
      logLock.unlock();
    }
    // printf("Waiting for publication of '%s' "
    // "on topic '%s' for client with ClientID: '%s'\n",
    // payload, topic, ini["mqttin"]["clientid"].c_str());
    while (qos != 0 and deliveredtoken != token)
    {
      #if defined(_WIN32)
      Sleep(100);
      #else
      usleep(500L);
      #endif
    }
  }
  mqttPublishLock.unlock();
  return true;
}

void UMqttIn::tick()
{
  if (tAlive.getTimePassed() > 2)
  { // keep connection alive
    tAlive.now();
    char s[50];
    publish("robobot/teensy_interface/alive", tAlive.getDateTimeAsString(s), tAlive);
  }
}
