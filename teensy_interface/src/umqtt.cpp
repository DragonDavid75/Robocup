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
#include "umqtt.h"

using namespace std::chrono;

// create value
UMqtt mqtt;

// #define ADDRESS     "tcp://localhost:1883"
// #define CLIENTID    "Joe"
// #define TOPIC       "acc"
// #define PAYLOAD     "1.0111 0.222 0.333"
// #define QOS         1
// #define TIMEOUT     10000L


void UMqtt::setup()
{ // ensure default values
  if (not ini.has("mqtt"))
  { // no data yet, so generate some default values
    ini["mqtt"]["broker"] = "tcp://localhost:1883"; // IP and port to broker
    ini["mqtt"]["context"] = "drive"; // not used
    ini["mqtt"]["clientid"] = "tif_out"; // data source for publish
    ini["mqtt"]["function"] = "drive/"; // drive/ or crane/. Function of this process
    ini["mqtt"]["system"] = "robobot/"; // top name in MQTT topic
    ini["mqtt"]["log"] = "true";
    ini["mqtt"]["print"] = "false";
    ini["mqtt"]["use"] = "true";
  }
  if (ini["mqtt"]["print"] == "true")
  // logfiles
  toConsole = ini["mqtt"]["print"] == "true";
  if (ini["mqtt"]["log"] == "true" and logfile == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_mqtt.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% mqtt logfile, enabled=%s\n", ini["mqtt"]["use"].c_str());
    fprintf(logfile, "%% broker=%s\n", ini["mqtt"]["broker"].c_str());
    fprintf(logfile, "%% context=%s\n", ini["mqtt"]["context"].c_str());
    fprintf(logfile, "%% client id=%s\n", ini["mqtt"]["clientid"].c_str());
    fprintf(logfile, "%% system=%s (top level ID)\n", ini["mqtt"]["system"].c_str());
    fprintf(logfile, "%% function=%s (next level name)\n", ini["mqtt"]["function"].c_str());
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tQuality of service 0=max once, 1=at least once, 2=once, S=subscribe\n");
    // fprintf(logfile, "%% 3 \tTx = published, Rx = received, Su = subscribe\n");
    fprintf(logfile, "%% 3 \t'Topic'\n");
    fprintf(logfile, "%% 4 \tMessage / payload\n");
  }
  // MQTT
  if (ini["mqtt"]["use"] == "true" and not connected)
  { // MQTT enabled
    int rc;
    int connectCnt = 0;
    bool isOK = false;
    rc = MQTTClient_create(&client,
                            ini["mqtt"]["broker"].c_str(),
                            ini["mqtt"]["clientid"].c_str(),
                            MQTTCLIENT_PERSISTENCE_NONE,
                            NULL);
    isOK = rc == MQTTCLIENT_SUCCESS;
    if (not isOK)
    {
      printf("# UMqtt:: Failed to create client, return code %d\n", rc);
  //     rc = EXIT_FAILURE;
    }
    if (isOK)
    {
      rc = MQTTClient_setCallbacks(client, (void*)ini["mqtt"]["context"].c_str(), connlost, msgarrvd, delivered);
      isOK = rc == MQTTCLIENT_SUCCESS;
      if (not isOK)
      {
        printf("# UMqtt:: Failed to set callbacks, return code %d\n", rc);
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
        printf("# UMqtt:: Waiting for (drive) Mosquitto (%d)\n", connectCnt);
      }
      if (not isOK)
      {
        printf("# UMqtt:: Failed to connect for drive data, return code %d\n", rc);
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
      printf("# UMqtt:: connection to MQTT broker on %s established\n", ini["mqtt"]["broker"].c_str());
      service.logMessage("# Mqtt data connection established");
      // subscribe
      // moved to IN-channel
      // subscribe("robobot/cmd/#", 1);
    }
    //end MQTT

    // if (isOK and not service.stop)
    //   // start listen to the keyboard
    //   th1 = new std::thread(runObj, this);
    connected = isOK;
  }
  else
    printf("# UMqtt:: disabled in robot.ini\n");
}


void UMqtt::reconnect()
{
  int connectCnt = 0;
  bool isOK = false;
  int rc = 0;
  UTime t("now");
  const int MSL = 50;
  char s[MSL];
  printf("# %lu.%04ld MQTT (data) reconnect (%s)\n",
         t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
  if (logfile != nullptr and not service.stop_logging)
  {
    logLock.lock();
    fprintf(logfile,"%lu.%04ld Reconnecting (data) %s\n",
            t.getSec(), t.getMicrosec()/100, s);
    logLock.unlock();
  }
  //
  if (connected)
  { // try reconnect
    rc = MQTTClient_disconnect(client, 10000);
    if (rc != MQTTCLIENT_SUCCESS)
    {
      printf("# UMqtt:: Failed to disconnect (data), return code %d\n", rc);
      rc = EXIT_FAILURE;
    }
    connected = false;
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
    printf("# UMqtt:: (data) Waiting for Mosquitto (%d)\n", connectCnt);
  }
  if (not isOK)
  {
    printf("# UMqtt:: (data) Failed to connect, return code %d\n", rc);
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
    printf("# %lu.%04ld MQTT (data) reconnected (%s)\n",
           t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
    // subscribe("robobot/cmd/shutdown", 1);
    // subscribe
    // std::string topic = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "#";
    // subscribe(topic.c_str(), 1);
    //
    // no subscription for this interface
  }
  connected = isOK;
}
void UMqtt::terminate()
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
      printf("# UMqtt:: Failed to disconnect, return code %d\n", rc);
      rc = EXIT_FAILURE;
    }
    MQTTClient_destroy(&client);
    printf("# UMqtt:: client shutdown\r\n");
  }
}

bool UMqtt::subscribe ( const char* topic, int qos )
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


void UMqtt::toLogRx(const char* context, const char* topic, char * message, int qos, UTime t, bool /*used*/)
{ // pv is pin-value
  if (service.stop)
    return;
  if (logfile != nullptr and not service.stop_logging)
  {
    logLock.lock();
    fprintf(logfile,"%lu.%04ld Rx %d %s %s %s\n",
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


void UMqtt::delivered(void * /*context*/, MQTTClient_deliveryToken dt)
{
  //printf("# MQTT %s Message with token value %d delivery confirmed\n", (char*) context,  dt);
  mqtt.deliveredtoken = dt;
}

int UMqtt::msgarrvd(void *context, char *topicName, int /*topicLen*/, MQTTClient_message *message)
{
  if (true)
  { // debug
    printf("# MQTT Message arrived (NB! no message should arrive here - moved to IN-channel)\n");
    if (context != nullptr)
      printf("#   context: %s\n", (char *)context);
    printf(  "#     topic: %s\n", topicName);
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
  mqtt.toLogRx((char*)context, topicName, s, message->qos, t, used);
  //
  MQTTClient_freeMessage(&message);
  MQTTClient_free(topicName);
  return 1;
}

void UMqtt::connlost(void */*context*/, char *cause)
{
  printf("# UMqtt:: (data) Connection lost (cause: %s)\n", cause);
  // printf("#     cause: %s\n", cause);
}



bool UMqtt::publish(const char * topic, const char * payload, UTime & msgTime, int qos)
{
  if (ini["mqtt"]["use"] != "true")
    // MQTT disabled in robot.ini
    return false;
  if (not connected)
  {  printf("# publish, but not connected; a %s\n", topic);
    // mosquitto is probably not found
    return false;
  }
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
  // printf("# MQTT publish topic %s payload %s", topic, s);
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
    if (publish_error > 4)
      reconnect();
    printf("Failed to publish message, return code %d\n", rc);
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
    // payload, topic, ini["mqtt"]["clientid"].c_str());
    while (qos != 0 and deliveredtoken != token)
    {
      #if defined(_WIN32)
      Sleep(100);
      #else
      usleep(1000L);
      #endif
    }
  }
  mqttPublishLock.unlock();
  return true;
}

