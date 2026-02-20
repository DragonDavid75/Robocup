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
  // MQTT
  if (true)
  { // MQTT enabled
    int rc;
    int connectCnt = 0;
    bool isOK = false;
    rc = MQTTClient_create(&client,
                            service.mqtthost.c_str(),
                            service.mqttclient.c_str(),
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
      rc = MQTTClient_setCallbacks(client, (void*)"drive", connlost, msgarrvd, delivered);
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
        printf("# UMqtt:: Waiting for Mosquitto (%d) %s\n", connectCnt, service.mqtthost.c_str());
      }
      if (not isOK)
      {
        printf("# UMqtt:: Failed to connect, return code %d\n", rc);
      }
    }
    if (isOK)
    {
      printf("# UMqtt:: %s connected to MQTT broker %s\n", service.mqttclient.c_str(), service.mqtthost.c_str());
      // subscribe
      subscribe("robobot/cmd/shutdown", 1);
      // subscribe("robobot/drive/T0/#", 0);
    }
    //end MQTT

    // if (isOK and not service.stop)
    //   // start listen to the keyboard
    //   th1 = new std::thread(runObj, this);
    connected = isOK;
  }
}

void UMqtt::reconnect()
{
  int connectCnt = 0;
  bool isOK = false;
  int rc = 0;
  UTime t("now");
  const int MSL = 50;
  char s[MSL];
  printf("# %lu.%04ld MQTT reconnect (%s)\n",
         t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
  //
  if (connected)
  { // try reconnect
    rc = MQTTClient_disconnect(client, 10000);
    if (rc != MQTTCLIENT_SUCCESS)
    {
      printf("# UMqtt:: Failed to disconnect, return code %d\n", rc);
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
    printf("# UMqtt:: Waiting for Mosquitto (%d)\n", connectCnt);
  }
  if (not isOK)
  {
    printf("# UMqtt:: Failed to connect, return code %d\n", rc);
  }
  else
  {
    printf("# %lu.%04ld MQTT reconnected (%s)\n",
           t.getSec(), t.getMicrosec()/100, t.getDateTimeAsString(s, true));
    subscribe("robobot/cmd/shutdown", 1);
  }
  connected = isOK;
}

void UMqtt::terminate()
{
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
  return true;
}


// void UMqtt::run()
// {
//   int loop = 0;
//   auto sampleTime =  1ms;
//   auto loopTime = std::chrono::steady_clock::now() + sampleTime;
//   printf("# MQtt thread started\n");
//   while (not service.stop)
//   {
//     loop++;
//     //
//     std::this_thread::sleep_until(loopTime);
//     loopTime += sampleTime;
//   }
//   printf("# MQtt terminated\n");
// }


// void UMqtt::toLogRx(const char* context, const char* topic, MQTTClient_message * message, UTime t, bool /*used*/)
// { // pv is pin-value
//   if (service.stop)
//     return;
//   if (logfile != nullptr)
//   {
//     fprintf(logfile,"%lu.%04ld Rx %d %s %s %s\n",
//             t.getSec(), t.getMicrosec()/100,
//             message->qos, context, topic, (char*)message->payload);
//   }
//   if (toConsole)
//   {
//     printf("%lu.%04ld %d %s %s %s\n",
//             t.getSec(), t.getMicrosec()/100,
//            message->qos, context, topic, (char*)message->payload);
//   }
// }


void UMqtt::delivered(void * /*context*/, MQTTClient_deliveryToken dt)
{
  //printf("# MQTT %s Message with token value %d delivery confirmed\n", (char*) context,  dt);
  mqtt.deliveredtoken = dt;
}

int UMqtt::msgarrvd(void *context, char *topicName, int /*topicLen*/, MQTTClient_message *message)
{
  if (false)
  { // debug
    printf("# MQTT Message arrived\n");
    if (context != nullptr)
      printf("#   context: %s\n", (char *)context);
    printf(  "#     topic: %s\n", topicName);
    printf(  "#   message: %.*s\n", message->payloadlen, (char*)message->payload);
  }
  //
  char m[200];
  if (message->payloadlen < 200)
  {
    strncpy(m, (char*)message->payload, message->payloadlen);
    m[message->payloadlen] = '\0';
  }
  else
    strcpy(m, "too long");
  UTime t("now");
  /*bool used = */service.mqttDecode(topicName, m, t);
  //
  MQTTClient_freeMessage(&message);
  MQTTClient_free(topicName);
  return 1;
}

void UMqtt::connlost(void */*context*/, char *cause)
{
  UTime t("now");
  const int MSL = 100;
  char s[MSL];
  printf("# at %s UMqtt:: Connection lost\n", t.getDateTimeAsString(s, true));
  printf("#     cause: %s\n", cause);
  // don't reconnect here, wait for 10 failed publish
  // mqtt.connected = false;
}



bool UMqtt::publish(const char * topic, const char * payload, UTime & msgTime, int qos)
{
  if (not connected)
    // mosquitto is probably not found
    return false;
  if (strlen(topic) < 5)
    // no topic
    return false;
  //
  //const auto mtime{system_clock::now()};
  //auto ms_since_epoch = duration_cast<milliseconds>(mtime.time_since_epoch());
  // time since 1 January 1970 in seconds from system clock
  //double mtime = duration<double>(system_clock::now().time_since_epoch()).count();
  //std::cout << "# msg time " << ms_since_epoch << "\n";
  // add timestamp
  const int MSL = 2000;
  char s[MSL];
  char s2[50];
  UTime t("now");
  snprintf(s, MSL, "%lu.%04ld %s", msgTime.getSec(), msgTime.getMicrosec()/100, payload);
  //printf("# MQTT publish topic %s payload %s", topic, s);
  pubmsg.payload = (void*)s;
  pubmsg.payloadlen = (int)strlen(s);
  pubmsg.qos = qos;
  pubmsg.retained = 0;
  deliveredtoken = 0;
  int rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
  // int rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
  if (rc != MQTTCLIENT_SUCCESS)
  {
    publish_error++;
    if (publish_error > 3)
      reconnect();
    printf("# At %s; failed (rc=%d) to publish %s %s",
           t.getDateTimeAsString(s2), rc, topic, s);
  }
  else
  { // wait for message to be delivered (for quality services only).
    //
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
  return true;
}

