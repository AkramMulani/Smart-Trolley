/**
* Servo motor : D10
* IR Sensor   : D11
* Bluetooth
          TX  : D0
          RX  : D1
* Motors
          M1  : 1
          M2  : 2
          M3  : 3
          M4  : 4
*/


#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define TRIG 3
#define ECHO 2
#define IR 11
#define BT_TX 0
#define BT_RX 1
#define GPS_TX 4
#define GPS_RX 5

// The TinyGPS++ object
TinyGPSPlus gps;

// Servo motor
Servo myServo;

// Motors
AF_DCMotor M1(1), M2(2), M3(3), M4(4);

/**
Bluetooth Module HC-05
TX = A4, RX = A5
*/
SoftwareSerial bluetooth(BT_RX,BT_TX);
/**
GPS Module NEO-6M
TX = A2, RX = A3
*/
SoftwareSerial gpsSerial(GPS_RX,GPS_TX);

// DIRECTIONS
bool RIGHT_DIRECTION=false, LEFT_DIRECTION=false, MOVE=false;

// Setup all the requirenments
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bluetooth.begin(9600); // Set Bluetooth communication baud rate
  gpsSerial.begin(9600); // Set GPS baud rate

  pinMode(IR, INPUT);

  Serial.println("Welcome to smart trolley project..........");
  Serial.println("Smart trolley started........");
  myServo.attach(10);
  myServo.write(90);
}
/**
* Function to run continuously
*/
void loop() {

  // looking for the direction
  if (MOVE) lookForDirection();
  else myServo.write(90);

  // printing direction
  Serial.println(RIGHT_DIRECTION?"Turn Right":LEFT_DIRECTION?"Turn Left":MOVE?"Moving Straight":"Looking for direction");
  delay(100);

  // move trolley accordingly
  if (RIGHT_DIRECTION) {
    highSpeed();
    // turn right
    moveLeft(0);
    moveRight(1);
  }
  else if (LEFT_DIRECTION) {
    highSpeed();
    // turn left
    moveRight(0);
    moveLeft(1);
  }
  else if (MOVE) {
    normalSpeed();
    // move forward
    moveLeft(0);
    moveRight(0);
  }
  else {
    // stop motors
    stopMotors();
  }

}
/**
* To recognise obstacle in path
* - returns int
*/
int getIR() {
  int val = digitalRead(IR);
  Serial.print("IR: ");
  Serial.print(val);
  Serial.println(val==HIGH?" - HIGH":" - LOW");
  return val;
}
/**
* Looks right and left for the direction and sets flags of direction
* - RIGHT_DIRECTION: Flag to turn right
* - LEFT_DIRECTION: Flag to turn left
* - MOVE: Flag to move forward
*/
void lookForDirection() {
  int i;
  // LOOK RIGHT
  for (i=90;i>=0;i--) { myServo.write(i); delay(10); }
  RIGHT_DIRECTION = getIR()==LOW;
  delay(100);
  
  // LOOK LEFT
  for (i=0;i<=180;i++) { myServo.write(i); delay(10); }
  LEFT_DIRECTION = getIR()==LOW;
  delay(100);

  // LOOK STRAIGHT
  for (i=180;i>=90;i--) { myServo.write(i); delay(10); }
  MOVE = getIR()==LOW && !RIGHT_DIRECTION && !LEFT_DIRECTION;
  delay(100);
}
/**
* Set the speed of motors to 150 (NORMAL)
*/
void normalSpeed() {
  M1.setSpeed(150);
  M2.setSpeed(150);
  M3.setSpeed(150);
  M4.setSpeed(150);
}
/**
* Set the speed of motors to 250 (HIGH)
*/
void highSpeed() {
  M1.setSpeed(250);
  M2.setSpeed(250);
  M3.setSpeed(250);
  M4.setSpeed(250);
}
/**
* Moves the wheels of left side of trolley in either FORWARD or BACKWARD direction according to the parameter dir
* -dir - specify the direction of the wheels [0: forward direction, 1: backward direction]
*/
void moveLeft(int dir) {
  M2.run(dir==0?FORWARD:BACKWARD);
  M4.run(dir==0?FORWARD:BACKWARD);
}
/**
* Moves the wheels of right side of trolley in either FORWARD or BACKWARD direction according to the parameter dir
* -dir - specify the direction of the wheels [0: forward direction, 1: backward direction]
*/
void moveRight(int dir) {
  M1.run(dir==0?FORWARD:BACKWARD);
  M3.run(dir==0?FORWARD:BACKWARD);  
}
/**
* Stops all wheels of trolley
*/
void stopMotors() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

/**Send the message via bluetooth*/
void sendBluetoothMessage(const char* message) {
  bluetooth.println(message);
}

/**Receive the message via bluetooth*/
String receiveBluetoothMessage() {
  // format = latidute,longitude
  String msg="";
  if (bluetooth.available()) {
    Serial.println("Message received from Bluetooth: ");
    while (bluetooth.available()) {
      msg = bluetooth.readString();
      Serial.print(msg);
    }
    Serial.println();
  }
  return msg;
}

/**Get location coordinates in Longitude and Latitude format*/
TinyGPSPlus getLongitudeLatitude() {
  String latitude = "";
  String longitude = "";
  if (gpsSerial.available()>0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }
  return gps;
}
