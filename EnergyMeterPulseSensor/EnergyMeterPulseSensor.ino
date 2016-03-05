/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik EKblad

   DESCRIPTION
   This sketch provides an example how to implement a distance sensor using HC-SR04
   Use this sensor to measure KWH and Watt of your house meeter
   You need to set the correct pulsefactor of your meeter (blinks per KWH).
   The sensor starts by fetching current KWH value from gateway.
   Reports both KWH and Watt back to gateway.

   Unfortunately millis() won't increment when the Arduino is in
   sleepmode. So we cannot make this sensor sleep if we also want
   to calculate/report watt-number.
   http://www.mysensors.org/build/pulse_power
*/

#include <SPI.h>
#include <MySensor.h>

#define NODE_ID 2
#define CHILD_ID_POWERM 3     // Id of the sensor child

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT 10000          // Max watt value to report. This filetrs outliers.
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)          
unsigned long SEND_FREQUENCY = 20000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
MySensor gw;
double ppwh = ((double)PULSE_FACTOR) / 1000; // Pulses per watt hour
boolean pcReceived = false;
volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile unsigned long watt = 0;
unsigned long oldPulseCount = 0;
unsigned long oldWatt = 0;
double oldKwh;
unsigned long lastSend;
MyMessage wattMsg(CHILD_ID_POWERM, V_WATT);
MyMessage kwhMsg(CHILD_ID_POWERM, V_KWH);
MyMessage pcMsg(CHILD_ID_POWERM, V_VAR1);
// Using message type VAR2 for the requests of the power meter to have them in a separate mqtt-topic:
// V_VAR2 =  '1' Sensors requests the last known number of pulses from the GW
MyMessage pcReq(CHILD_ID_POWERM, V_VAR2);

void setup()
{
  gw.begin(incomingMessage);
  // changed to manual id
  gw.begin(incomingMessage, NODE_ID, true);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Energy Meter (OH-MQTT modification)", "1.0");

  // Register this device as power sensor
  gw.present(CHILD_ID_POWERM, S_POWER);

  // Fetch last known pulse count value from gw
  //gw.request(CHILD_ID, V_VAR1);
  // changed to V_VAR2=1 because I don't know how to handle NULL messages in OpenHAB-MQTT
  gw.send(pcReq.set(1));

  attachInterrupt(INTERRUPT, onPulse, RISING);
  lastSend = millis();
}


void loop()
{
  gw.process();
  unsigned long now = millis();
  // Only send values at a maximum frequency or woken up from sleep
  bool sendTime = now - lastSend > SEND_FREQUENCY;
  if (pcReceived && (SLEEP_MODE || sendTime)) {
    // New watt value has been calculated
    if (!SLEEP_MODE && watt != oldWatt) {
      // Check that we dont get unresonable large watt value.
      // could hapen when long wraps or false interrupt triggered
      if (watt < ((unsigned long)MAX_WATT)) {
        gw.send(wattMsg.set(watt));  // Send watt value to gw
      }
      Serial.print("Watt:");
      Serial.println(watt);
      oldWatt = watt;
    }

    // Pulse cout has changed
    if (pulseCount != oldPulseCount) {
      gw.send(pcMsg.set(pulseCount));  // Send pulse count value to gw
      double kwh = ((double)pulseCount / ((double)PULSE_FACTOR));
      oldPulseCount = pulseCount;
      if (kwh != oldKwh) {
        gw.send(kwhMsg.set(kwh, 4));  // Send kwh value to gw
        oldKwh = kwh;
      }
    }
    lastSend = now;
  } else if (sendTime && !pcReceived) {
    // No count received. Try requesting it again
    //gw.request(CHILD_ID, V_VAR1);
    // changed to V_VAR2=1 because I don't know how to handle NULL messages in OpenHAB-MQTT
    gw.send(pcReq.set(1));
    lastSend = now;
  }

  if (SLEEP_MODE) {
    gw.sleep(SEND_FREQUENCY);
  }
}

void incomingMessage(const MyMessage &message) {
  if (message.type == V_VAR1) {
    long receivedCount = message.getLong();
    // Puls counts == 0 are being ignorned because the GW sends this when OH subscribes to the topic (which happens e.g. every time the items are updated)
    // otherwise each change in OH would result in a reset of the power meter
    // if power meter should be resetted manually set the count to 1
    if (receivedCount > 0 ) {
      pulseCount = oldPulseCount = receivedCount;
      Serial.print("Received last pulse count from gw:");
      Serial.println(pulseCount);
      pcReceived = true;
    }
    else {
      Serial.println("Received pulse count from gw is zero, ignoring.");
    }
  }
}

void onPulse()
{
  if (!SLEEP_MODE) {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;
    if (interval < 10000L) { // Sometimes we get interrupt on RISING
      return;
    }
    watt = (3600000000.0 / interval) / ppwh;
    lastBlink = newBlink;
  }
  pulseCount++;
}
