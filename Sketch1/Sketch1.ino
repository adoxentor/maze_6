/*
  Name:		Sketch1.ino
  Created:	3/1/2018 4:22:40 PM
  Author:	IDDO
  hi gilad
  how are you?
*/

//#include <MPU9250_RegisterMap.h>
#include <mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <inv_mpu.h>
#include <I2Cdev.h>
#include <freeram.h>
#include <dmpmap.h>
#include <dmpKey.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
//#include <SparkFunMPU9250-DMP.h>
//#include <quaternionFilters.h>
//#include <MPU9250.h>

int mission = 3;
const int MSN_SYNCING = 1;
const int MSN_CHECK_CROSSROAD = 2;
const int MSN_MOVE_STRIGHT = 3;
const int MSN_TURN_RIGHT = 4;
const int MSN_TURN_LEFT = 5;
const int MSN_FIND_ROAD = 6;

int move = 0;
const int MOVE_FORWARD_RIGHT = 0;
const int MOVE_FORWARD_LEFT = 1;
const int MOVE_BACKWARD_RIGHT = 2;
const int MOVE_BACKWARD_LEFT = 3;

Servo myservo;

float maxMagX = -488.97;
float minMagX= -550.34;
float maxMagY=223.71;
float minMagY= 155.29;
float maxMagZ= 119.28;
float minMagZ= 44.41;
float magX;
float magY;
float magZ;
float magX100;
float magY100;
float magZ100;
float angleNow;

double avrDistanceR = 0;
double alphaDistance = 0.5;

double timer;

int LMotorB = A0; //Left motor backwards
int LMotorF = A1; //Left motor forwards
int LMotorS = 5; //Left motor speed

int RMotorF = A2; //Right motor backwards
int RMotorB = A3; //Right motor forwards
int RMotorS = 3; //Right motor speed

int inputUSLeft = 9;
int outputUSLeft = 8;

int inputUSRight = 13;
int outputUSRight = 12;

boolean b = true;
boolean synced = false;

SoftwareSerial BTserial(10, 11);
int ret;

void setup() {
	//myservo.attach(7);
	Serial.begin(9600);
	BTserial.begin(9600);
	pinMode(LMotorB, OUTPUT);
	pinMode(LMotorF, OUTPUT);
	pinMode(LMotorS, OUTPUT);
	pinMode(RMotorB, OUTPUT);
	pinMode(RMotorF, OUTPUT);
	pinMode(RMotorS, OUTPUT);

	pinMode(inputUSLeft, INPUT);
	pinMode(outputUSLeft, OUTPUT);
	pinMode(inputUSRight, INPUT);
	pinMode(outputUSRight, OUTPUT);

	print2ln("begin");


	Fastwire::setup(400, 0);
	ret = mympu_open(200);
	print2("MPU init: "); print2ln(ret);
	print2("Free mem: "); print2ln(freeRam());



	// Use setSensors to turn on or off MPU-9250 sensors.
	// Any of the following defines can be combined:
	// INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
	// INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
	// Enable all sensors:
	//print2ln(imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS));

	// Use setGyroFSR() and setAccelFSR() to configure the
	// gyroscope and accelerometer full scale ranges.
	// Gyro options are +/- 250, 500, 1000, or 2000 dps
	//imu.setGyroFSR(2000); // Set gyro to 2000 dps
	// Accel options are +/- 2, 4, 8, or 16 g
	//imu.setAccelFSR(2); // Set accel to +/-2g
	// Note: the MPU-9250's magnetometer FSR is set at
	// +/- 4912 uT (micro-tesla's)

	// setLPF() can be used to set the digital low-pass filter
	// of the accelerometer and gyroscope.
	// Can be any of the following: 188, 98, 42, 20, 10, 5
	//// (values are in Hz).
	//imu.setLPF(5); // Set LPF corner frequency to 5Hz

	//// The sample rate of the accel/gyro can be set using
	//// setSampleRate. Acceptable values range from 4Hz to 1kHz
	//imu.setSampleRate(10); // Set sample rate to 10Hz

	//// Likewise, the compass (magnetometer) sample rate can be
	//// set using the setCompassSampleRate() function.
	//// This value can range between: 1-100Hz
	//imu.setCompassSampleRate(10); // Set mag rate to 10Hz
	//mission = MSN_SYNCING;
}

void loop() {
	while (true)
	{
		Serial.println(angle());
	}
	switch (mission) {
	case(MSN_SYNCING):
		//syncingMaze();
		break;
	case(MSN_CHECK_CROSSROAD):

		break;
	case(MSN_MOVE_STRIGHT):
		print2ln("Choose the side of the wall that the robot need to follow (send 'R' for right and 'L' for left)");
		while (1) {
			if (BTserial.available() && BTserial.read() == 'R') {
				print2ln("Choose the direction that the robot will drive (send 'F' for forward and 'B' for backward)");
				while (1) {
					if (BTserial.read() == 'F') {
						move = MOVE_FORWARD_RIGHT;
						break;
					}
					if (BTserial.read() == 'B') {
						move = MOVE_BACKWARD_RIGHT;
						break;
					}
				}
				break;
			}
			if (BTserial.available() && BTserial.read() == 'L') {
				print2ln("Choose the direction that the robot will drive (send 'F' for forward and 'B' for backward)");
				while (1) {
					if (BTserial.read() == 'F') {
						move = MOVE_FORWARD_LEFT;
						break;
					}
					if (BTserial.read() == 'B') {
						move = MOVE_BACKWARD_LEFT;
						break;
					}
				}
				break;
			}
		}
		moveStraight();
		break;
	case(MSN_TURN_RIGHT):
		turnRight();
		break;
	case(MSN_TURN_LEFT):
		turnLeft();
		break;
	case(MSN_FIND_ROAD):
		findRoad();
		break;
	}
}

double angle() {
//	imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
//	magX = imu.calcMag(imu.mx);
//	magY = imu.calcMag(imu.my);
//
//	magX100 = ((magX - minMagX) / (maxMagX - minMagX)) - 0.5;
//	magY100 = ((magY - minMagY) / (maxMagY - minMagY)) - 0.5;
//	//   print2ln(String(magX100));
//	//   print2ln(String(magY100));
//	angleNow = atan2(magX100, magY100) * 180 / PI;
//	//print2ln(String(angleNow));
//	return angleNow;

	unsigned int c = 0; //cumulative number of successful MPU/DMP reads
	unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
	unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
	unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

	while (true) {
		ret = mympu_update();
		// errorReporting(); // turn on for debug information

		if (!(c % 25)) { // output only every 25 MPU/DMP reads
			print2("Y: "); print2(mympu.ypr[0]);
			print2(" P: "); print2(mympu.ypr[1]);
			print2(" R: "); print2(mympu.ypr[2]);
			print2("\tgy: "); print2(mympu.gyro[0]);
			print2(" gp: "); print2(mympu.gyro[1]);
			print2(" gr: "); print2ln(mympu.gyro[2]);
			delay(100);

		}
	}

}
double distance(int outputUS, int inputUS, double avrDistance)
{
	digitalWrite(outputUS, LOW);
	delayMicroseconds(2);
	digitalWrite(outputUS, HIGH);
	delayMicroseconds(10);
	digitalWrite(outputUS, LOW);
	double distance = pulseIn(inputUS, HIGH) / 5.8 / 10;
	avrDistance = alphaDistance * distance + avrDistance * (1 - alphaDistance);
	return avrDistance;
}

void syncingMaze() {
//	if (imu.dataReady())
//	{
//		Serial.println("Press 'S' to syncing the program");
//		if (true|| BTserial.available() && BTserial.read() == 'S') {
//			Serial.println("Syncing the program");
//			if (maxMagX == 100000) {
//				imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
//				maxMagX = imu.calcMag(imu.mx);
//				minMagX = imu.calcMag(imu.mx);
//				maxMagY = imu.calcMag(imu.my);
//				minMagY = imu.calcMag(imu.my);
//				maxMagZ = imu.calcMag(imu.mz);
//				minMagZ = imu.calcMag(imu.mz);
//			}
//			while (b) {
//				timer = millis() + 20000;
//				while (millis() < timer) {
//					imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
//					magX = imu.calcMag(imu.mx);
//					magY = imu.calcMag(imu.my);
//					magZ = imu.calcMag(imu.mz);
//					if (magX > maxMagX)
//						maxMagX = magX;
//					if (magX < minMagX)
//						minMagX = magX;
//					if (magY > maxMagY)
//						maxMagY = magY;
//					if (magY < minMagY)
//						minMagY = magY;
//					if (magZ > maxMagZ)
//						maxMagZ = magZ;
//					if (magZ < minMagZ)
//						minMagZ = magZ;
//					Serial.println(String(magX) + " ," + String(magY) + " ," + String(magZ));
//					//analogWrite(RMotorS, 120);
//					//digitalWrite(RMotorF, HIGH);
//				}
//				digitalWrite(RMotorF, LOW);
//				//printIMUData();
//				delay(50);
//				Serial.println();
//				Serial.println();
//				Serial.println();
//				Serial.println();
//				Serial.println("Max x: " + String(maxMagX) + ", Min x: " + String(minMagX) + ", Max y: " + String(maxMagY) + ", Min y: " + String(minMagY) + ", Max z: " + String(maxMagZ) + ", Min z: " + String(minMagZ));
//				b = false;
//			}
//			synced = true;
//		}
//		if (synced) {
//		}
//	}
//	else {
//		Serial.println("Loading...");
//	}
}


void moveStraight() {
	int outputUS_Side;
	int inputUS_Side;
	int outputUS_Front;
	int inputUS_Front;
	char direction;
	switch (move) {
	case(MOVE_FORWARD_RIGHT):
		outputUS_Side = outputUSRight;
		inputUS_Side = inputUSRight;
		outputUS_Front = outputUSLeft;
		inputUS_Front = inputUSLeft;
		digitalWrite(RMotorF, HIGH);
		digitalWrite(LMotorF, HIGH);
		myservo.write(45);
		break;
	case(MOVE_FORWARD_LEFT):
		outputUS_Side = outputUSLeft;
		inputUS_Side = inputUSLeft;
		outputUS_Front = outputUSRight;
		inputUS_Front = inputUSRight;
		digitalWrite(RMotorF, HIGH);
		digitalWrite(LMotorF, HIGH);
		myservo.write(135);
		break;
	case(MOVE_BACKWARD_RIGHT):
		outputUS_Side = outputUS_Side;
		inputUS_Side = inputUSRight;
		outputUS_Front = outputUSLeft;
		inputUS_Front = inputUSLeft;
		digitalWrite(RMotorB, HIGH);
		digitalWrite(LMotorB, HIGH);
		myservo.write(45);
		break;
	case(MOVE_BACKWARD_LEFT):
		outputUS_Side = outputUSLeft;
		inputUS_Side = inputUSLeft;
		outputUS_Front = outputUSRight;
		inputUS_Front = inputUSRight;
		digitalWrite(RMotorB, HIGH);
		digitalWrite(LMotorB, HIGH);
		myservo.write(135);
		break;
	}
	
	print2ln("Put the robot in the start of the maze, looking forword into the maze (parallel to the side wall). After you did it send 'G'");
	while (1) {
		if (BTserial.available() && BTserial.read() == 'G') {
			break;
		}
	}
	while (1) {
		//double upAngle = angle();
		if (distance(outputUS_Side, inputUS_Side, avrDistanceR) > 5) {
			if (distance(outputUS_Side, inputUS_Side, avrDistanceR) > 15) {
				int farDis;
				for (int i = 0; i < 10; i++) {
					farDis = distance(outputUS_Side, inputUS_Side, 0);
					farDis = farDis / 2;
				}
				if (farDis > 15) {
					mission = MSN_CHECK_CROSSROAD;
					digitalWrite(RMotorF, LOW);
					digitalWrite(LMotorF, LOW);
					digitalWrite(RMotorB, LOW);
					digitalWrite(LMotorB, LOW);
					break;
				}
			}
			analogWrite(LMotorS, 150);
			analogWrite(RMotorS, 80);
		}
		if (distance(outputUS_Side, inputUS_Side, avrDistanceR) < 5) {
			analogWrite(RMotorS, 130);
			analogWrite(LMotorS, 80);
		}
		if (5 == distance(outputUS_Side, inputUS_Side, avrDistanceR)) {
			analogWrite(LMotorS, 90);
			analogWrite(RMotorS, 90);
		}
		if (distance(outputUS_Front, inputUS_Front, 0) < 5) {
			double blockDis;
			for (int i = 0; i < 10; i++) {
				blockDis = distance(outputUS_Front, inputUS_Front, blockDis);
				blockDis = blockDis / 2;
			}
			if (blockDis < 5) {
				digitalWrite(RMotorF, LOW);
				digitalWrite(LMotorF, LOW);
				digitalWrite(RMotorB, LOW);
				digitalWrite(LMotorB, LOW);
				mission = MSN_CHECK_CROSSROAD;
				break;
			}

		}

	}


}

void turnRight() {
	double firstAngle = angle();
	while (abs(abs(firstAngle) - abs(angle())) < 90) {
		digitalWrite(RMotorF, HIGH);
		digitalWrite(LMotorB, HIGH);
		digitalWrite(RMotorS, 100);
		digitalWrite(LMotorS, 100);
	}
	digitalWrite(RMotorF, LOW);
	digitalWrite(LMotorB, LOW);
	mission = MSN_FIND_ROAD;
}

void turnLeft() {
	double firstAngle = angle();
	while (abs(abs(firstAngle) - abs(angle())) < 90) {
		digitalWrite(RMotorB, HIGH);
		digitalWrite(LMotorF, HIGH);
		digitalWrite(RMotorS, 100);
		digitalWrite(LMotorS, 100);
	}
	digitalWrite(RMotorB, LOW);
	digitalWrite(LMotorF, LOW);
	mission = MSN_FIND_ROAD;
}

void findRoad() {
	myservo.write(45);
	if (distance(outputUSRight, inputUSRight, 0) < 10) {
		move = MOVE_FORWARD_RIGHT;
		mission = MSN_MOVE_STRIGHT;
	}
	else {
		myservo.write(135);
		if (distance(outputUSLeft, inputUSLeft, 0) < 10) {
			move = MOVE_FORWARD_LEFT;
			mission = MSN_MOVE_STRIGHT;
		}
	}
}



void printIMUData(void)
{
	// After calling update() the ax, ay, az, gx, gy, gz, mx,
	// my, mz, time, and/or temerature class variables are all
	// updated. Access them by placing the object. in front:

	// Use the calcAccel, calcGyro, and calcMag functions to
	// convert the raw sensor readings (signed 16-bit values)
	// to their respective units.
	////float accelX = imu.calcAccel(imu.ax);
	////float accelY = imu.calcAccel(imu.ay);
	////float accelZ = imu.calcAccel(imu.az);
	////float gyroX = imu.calcGyro(imu.gx);
	////float gyroY = imu.calcGyro(imu.gy);
	////float gyroZ = imu.calcGyro(imu.gz);
	////float magX = imu.calcMag(imu.mx);
	////float magY = imu.calcMag(imu.my);
	////float magZ = imu.calcMag(imu.mz);

	////print2ln("Accel: " + String(accelX) + ", " +
	////	String(accelY) + ", " + String(accelZ) + " g");
	////print2ln("Gyro: " + String(gyroX) + ", " +
	////	String(gyroY) + ", " + String(gyroZ) + " dps");
	////print2ln("Mag: " + String(magX) + ", " +
	////	String(magY) + ", " + String(magZ) + " uT");
	////print2ln("Time: " + String(imu.time) + " ms");
	////print2ln();
}

void print2(String s) {
	Serial.print(s);
	BTserial.print(s);
}
void print2ln(String s) {
	Serial.println(s);
	BTserial.println(s);
}
