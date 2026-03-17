/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3 
 */
 
/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/*LiDAR*/
// creating an RPLIDAR driver instance 
RPLidar lidar;
// HardwareSerial lidarSerial(2);

const int MAX_SCAN = 400;
uint16_t angles[MAX_SCAN];
uint16_t distances[MAX_SCAN];
int  scanIndex   = 0;
bool scanStarted = false;

#define RPLIDAR_MOTOR 26 // The PWM pin for controling the speed of RPLIDAR's motor.
                        // This pin should be connected to the RPLIDAR's MOTOCTRL signal 
/*LiDAR*/                       

/*WiFi*/
WiFiUDP udp;

const char* ssid     = "DESKTOP-31JKIMJ 1479";
const char* password = "R%2t6069";
const char* laptopIP = "192.168.137.1";
const int   port     = 50001;
/*WiFi*/

void setup() {
  Serial.begin(115200);
  // delay(300);

  /*Lidar*/
  Serial2.begin(115200);
  // bind the RPLIDAR driver to the hardware serial UART2
  // delay(300);
  lidar.begin(Serial2);
  // Serial.println("lidar binded");
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  /*Lidar*/

  /*WiFi*/
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print("*");
    delay(500);
  }

  udp.begin(port);
  /*WiFi*/
}

void loop() {
  //Serial.println("loop started");
  if (IS_OK(lidar.waitPoint())) {
    RPLidarMeasurement point = lidar.getCurrentPoint();
    
    if (point.startBit) {
      // New scan starting - send the completed previous scan
      if (scanStarted && scanIndex > 0) {
        uint16_t count = (uint16_t)scanIndex;
        udp.beginPacket(laptopIP, port);
        udp.write((uint8_t*)&count,     sizeof(uint16_t));
        udp.write((uint8_t*)angles,    count * sizeof(uint16_t));
        udp.write((uint8_t*)distances, count * sizeof(uint16_t));
        udp.endPacket();
      }
      scanIndex   = 0;
      scanStarted = true;
    }

    if (scanStarted && point.quality > 0 && scanIndex < MAX_SCAN) {
      angles[scanIndex]    = (uint16_t)(point.angle * 100);
      distances[scanIndex] = (uint16_t)(point.distance);
      scanIndex++;
      //Serial.print(angles[scanIndex]);
      //Serial.print(point.angle);
      //Serial.print("\t");
      //Serial.println(distances[scanIndex]);
      //Serial.print(point.distance);
      //y = (point.distance/1000) * sin(((point.angle)/180)*pi)
      //x = (point.distance/1000) * cos(((point.angle)/180)*pi)
    }
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}
