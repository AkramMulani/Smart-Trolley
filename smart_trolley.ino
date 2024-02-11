/**
Ultrasonic sensor
Trig = A0, Echo = A1
*/
#include <Arduino.h>
#include <AFMotor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define TRIG A0
#define ECHO A1
#define GPS_TX A2
#define GPS_RX A3
#define BT_TX A4
#define BT_RX A5

/**
Servo Motor
signal pin = 10
*/
Servo myServo;
int pos = 0;

/**
DC Motors
M1 = 3, M2 = 2, M3 = 1, M4 = 4
*/
AF_DCMotor M1(3);
AF_DCMotor M2(2);
AF_DCMotor M3(1);
AF_DCMotor M4(4);

/**
Bluetooth Module HC-05
TX = A4, RX = A5
*/
SoftwareSerial bluetooth(BT_RX,BT_TX);

// The TinyGPS++ object
TinyGPSPlus gps;

/**
GPS Module NEO-6M
TX = A2, RX = A3
*/
SoftwareSerial gpsSerial(GPS_RX,GPS_TX);

void setSpeedNormal() {
  M1.setSpeed(150);
  M2.setSpeed(150);
  M3.setSpeed(150);
  M4.setSpeed(150);
}

void setSpeedHigh() {
  M1.setSpeed(255);
  M2.setSpeed(255);
  M3.setSpeed(255);
  M4.setSpeed(255);
}

/**Turn the trolley to left side by 30 degrees*/
void turnLeft() {
  // right wheels move forward
  setSpeedHigh();
  moveMotorForward(M1);
  moveMotorForward(M2);
  moveMotorBackward(M3);
  moveMotorBackward(M4);
  delay(150);
}

/**Turn the trolley to right side by 30 degrees*/
void turnRight() {
  // left wheels move forward
  setSpeedHigh();
  moveMotorBackward(M1);
  moveMotorBackward(M2);
  moveMotorForward(M3);
  moveMotorForward(M4);
  delay(150);
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

/**Move motors to forward direction*/
void moveMotorForward(AF_DCMotor &motor) {
  motor.run(FORWARD);
}

/**Move motors to backward direction*/
void moveMotorBackward(AF_DCMotor &motor) {
  motor.run(BACKWARD);
}

/**Stop running motors*/
void stopMotor(AF_DCMotor &motor) {
  motor.run(RELEASE);
}

/**Find distance measurement in cm*/
long getDistance() {
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration * 0.034 / 2); // Speed of sound is 340 m/s or 0.034 cm/µs
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

/**Send the message via bluetooth*/
void sendBluetoothMessage(const char* message) {
  bluetooth.println(message);
}

/**Receive the message via bluetooth*/
void receiveBluetoothMessage() {
  // format = latidute,longitude
  String msg="";
  if (bluetooth.available()) {
    Serial.println("Message received from Bluetooth: ");
    while (bluetooth.available()) {
      msg = bluetooth.readString();
      Serial.write(msg);
    }
    Serial.println();
  }
  return msg;
}

void printAll() {
  String distance = String(getDistance(),2);
  TinyGPSPlus g = getLongitudeLatitude();
  String lat = String(g.location.lat(),6);
  String lng = String(g.location.lng(),6);
  String location = receiveBluetoothMessage();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("\t||\t");
  Serial.print("My Location:[Latitude"+lat+" Longitude:"+lng+"]");
  Serial.print("\t||\t");
  Serial.println("Device Location:["+location+"]");
  String msg = "Distance:"+distance+",Location:[Latitude:"+lat+" Longitude:"+lng+"]";
  sendBluetoothMessage(msg.c_str());
}

void lookLeft() {
  for(pos=90;pos>0;pos--) {
    myServo.write(pos);
    delay(15);
  }
}

void lookRight() {
  for(pos=0;pos<180;pos++) {
    myServo.write(pos);
    delay(15);
  }
}

void setServo() {
  if (pos==0) {
    for(pos=0;pos<90;pos++) {
      myServo.write(pos);
      delay(15);
    }
  }
  else if (pos==180) {
    for(pos=180;pos>90;pos--) {
      myServo.write(pos);
      delay(15);
    }
  }
  else {
    myServo.write(90);
    delay(15);
  }
}

void setup() {

  Serial.begin(9600); // For debugging purposes
  bluetooth.begin(9600); // Set Bluetooth communication baud rate
  gpsSerial.begin(9600); // Set GPS baud rate

  // Ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Motors initial speed
  M1.setSpeed(200);
  M2.setSpeed(200);
  M3.setSpeed(200);
  M4.setSpeed(200);

  // Setup servo motor
  myServo.attach(10);
  setServo();
  delay(1000);
}

void loop() {
  printAll();

  setServo();
  long distance = getDistance();
  if (distance > 20 && distance<=50) {
    lookLeft();
    long leftDistance = getDistance();
    lookRight();
    long rightDistance = getDistance();

    if (leftDistance > rightDistance) {
      turnLeft();
    }
    else if (rightDistance > leftDistance) {
      turnRight();
    }
    delay(2000);
  }
  else if (distance > 50) {
    moveMotorForward(M1);
    moveMotorForward(M2);
    moveMotorForward(M3);
    moveMotorForward(M4);
    delay(1000);
  }
  else {
    stopMotor(M1);
    stopMotor(M2);
    stopMotor(M3);
    stopMotor(M4);
    
    // turn around
    turnLeft();
    turnLeft();
  }

}
