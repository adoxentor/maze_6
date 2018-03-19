/*
 Name:		Sketch1.ino
 Created:	3/1/2018 4:22:40 PM
 Author:	IDDO
*/

// the setup function runs once when you press reset or power the board
#include <SPI.h>
#include <Wire.h>
#include <MPU9250.h>

MPU9250 IMU(Wire, 0x68);
//begin aa
void setup() {
	IMU.begin();
	IMU.begin();



}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
