#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <AFMotor.h>
#include <Servo.h>

// GPS Pins
static const int RXPin = A4, TXPin = A5;

// Bluetooth pins
static const int RXbt = A2, TXbt = A3;

// default baud rate
static const uint32_t BAUD = 9600;

// Ultrasonic sensor pins
static const int TRIG=A0, ECHO=A1;

// IR sensor pin
static const int IR=9;

// Servo pin
static const int S=10;

// Servo object
Servo servo;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The serial connection to the Bluetooth device
SoftwareSerial bt(RXbt, TXbt);

// motor 1
AF_DCMotor M1(1);

// motor 2
AF_DCMotor M2(2);

// motor 3
AF_DCMotor M3(3);

// motor 4
AF_DCMotor M4(4);

// movement flags
bool FRONT=false, BACK=false, RIGHT=false, LEFT=false;

// user location
double lat=0,lng=0;

// trolley location
double Tlat=0,Tlng=0;

// Previous trolley location variables
double prevTlat = 0.0, prevTlng = 0.0;

// setup all the things first
void setup(){
  pinMode(TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input
  pinMode(IR, INPUT);
  servo.attach(S);
  servo.write(90);
  Serial.begin(BAUD);
  ss.begin(BAUD);
  bt.begin(BAUD);
  Serial.println("Welcome to smart trolley....");
}

// continuously runs like loop
void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  String data = "";
  setUserLocation();
  setTrolleyLocation();
  
  while (ss.available()>0 && bt.available()>0) {
    followUser();
  }
  
}

// Function to calculate the distance between two GPS coordinates
double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
  // Earth's radius in kilometers
  const double R = 6371;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lng2 - lng1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2))
            * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Function to follow the user
void followUser() {
  double userDistance = calculateDistance(Tlat, Tlng, lat, lng);
  
  if (userDistance < 1.0) { // User is within 1 meter, stop the trolley
    stopAllMotors();
    return;
  }

  // Calculate the direction difference between user and trolley
  double userDirection = atan2(lat - Tlat, lng - Tlng);

  // Calculate the estimated trolley direction based on recent movement
  double trolleyDirection = getTrolleyDirection();

  // Adjust for directional wraparound (0 to 360 degrees)
  if (userDirection > M_PI) {
    userDirection -= 2.0 * M_PI;
  } else if (userDirection < -M_PI) {
    userDirection += 2.0 * M_PI;
  }
  if (trolleyDirection > M_PI) {
    trolleyDirection -= 2.0 * M_PI;
  } else if (trolleyDirection < -M_PI) {
    trolleyDirection += 2.0 * M_PI;
  }

  // Calculate the direction difference to adjust
  double directionDiff = userDirection - trolleyDirection;

  // Decide on movement based on direction difference
  if (abs(directionDiff) < 0.1) { // User is directly in front, move forward
    runAllForward();
    while(true) {
      long d = getDistance();
      if (d<=20) {
        stopAllMotors();
        checkMovement();
        if (FRONT) { runAllForward(); break; }
        else if (RIGHT) { turnRight(); break; }
        else if (LEFT) { turnLeft(); break; }
      }
    }
  } else if (directionDiff > 0.1) { // User is to the right, turn right slightly
    stopAllMotors();
    delay(100);
    turnRight();
    delay(500); // Adjust delay for smoother turning
  } else if (directionDiff < -0.1) { // User is to the left, turn left slightly
    stopAllMotors();
    delay(100);
    turnLeft();
    delay(500);
  }
}

// Function to estimate trolley direction based on recent movement
double getTrolleyDirection() {
  // Check if there's a valid previous location
  if (prevTlat == 0.0 || prevTlng == 0.0) {
    return 0.0; // No previous data, return neutral direction
  }

  // Calculate the direction difference based on displacement between current and previous locations
  double deltaX = Tlng - prevTlng;
  double deltaY = Tlat;
}

// ultrasonic sensor's returned distance value
int getDistance() {
  // defines variables
  long duration;
  int distance;
  // Clears the trigPin
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

// IR sensor's returned boolean value
bool getIR() {
  int i = digitalRead(IR);
  Serial.println(i==LOW?"IR LOW":"IR HIGH");
  return i==LOW;
}

// find user location
void setUserLocation() {
  if (bt.available()>0) {
    String location = bt.readStringUntil('\n');
    for (int i;i<location.length()-1;i++) {
      if (location.substring(i)==',') {
        lat = atof(location.substring(0, i).c_str());
        lng = atof(location.substring(i+1, location.length()-1).c_str());
      }
    }
  }
}

// find trolley location
void setTrolleyLocation() {
  if (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Tlat = gps.location.lat();
      Tlng = gps.location.lng();
      Serial.print("Latitude= "); 
      Serial.print(lat, 6);
      Serial.print(" Longitude= "); 
      Serial.println(lng, 6);
    }
  }
}

// run all motors forward
void runAllForward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

// run all motors backward
void runAllBackward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

// stop all the motors
void stopAllMotors() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

// set speed 255
void setMaxSpeed() {
  M1.setSpeed(255);
  M2.setSpeed(255);
  M3.setSpeed(255);
  M4.setSpeed(255);
}

// set speed 155
void setNormalSpeed() {
  M1.setSpeed(155);
  M2.setSpeed(155);
  M3.setSpeed(155);
  M4.setSpeed(155);
}

// command to turn left
void turnLeft() {
  setMaxSpeed();
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  delay(2000);
  stopAllMotors();
  LEFT = false;
}

// command to turn right
void turnRight() {
  setMaxSpeed();
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  delay(2000);
  stopAllMotors();
  RIGHT = false;
}

// command turn around
void turnBack() {
  setMaxSpeed();
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
  delay(4000);
  stopAllMotors();
  BACK = false;
}

// checking for movement
void checkMovement() {
  int R_DIST, L_DIST;
  int i;
  delay(100);
  if (getDistance()>=30) {
    FRONT = true;
    BACK = false;
    RIGHT = false;
    LEFT = false;
    return;
  }
  for (i=90;i>0;i--) {
    servo.write(i);
    delay(10);
  }
  R_DIST = getDistance();
  delay(100);
  for (i=0;i<180;i++) {
    servo.write(i);
    delay(10);
  }
  L_DIST = getDistance();
  delay(100);
  for (i=180;i>=90;i--) {
    servo.write(i);
    delay(10);
  }
  delay(100);
  if (R_DIST>L_DIST && R_DIST >= 30) {
    RIGHT = true;
    LEFT = false;
    FRONT = false;
    BACK = false;
  }
  else if (L_DIST>R_DIST && L_DIST >= 30) {
    LEFT = true;
    RIGHT = false;
    FRONT = false;
    BACK = false;
  }
  else if (L_DIST<30 && R_DIST<30) {
    BACK = true;
    FRONT = false;
    RIGHT = false;
    LEFT = false;
  }
}
