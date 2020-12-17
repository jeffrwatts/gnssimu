#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

float x_min = 100;
float x_max = -100;

Adafruit_BNO055::adafruit_vector_type_t types[] = {
  Adafruit_BNO055::VECTOR_GRAVITY,
  Adafruit_BNO055::VECTOR_EULER,
  Adafruit_BNO055::VECTOR_LINEARACCEL,
  Adafruit_BNO055::VECTOR_ACCELEROMETER,
  Adafruit_BNO055::VECTOR_MAGNETOMETER,
  Adafruit_BNO055::VECTOR_GYROSCOPE};

void setup() {
  Serial.begin(9600);
 
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
 
  // Display splash
  display.display();
  delay(1000);

  // Reset for text.
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.display(); // actually display all of the above

  boolean bno_initialized = bno.begin();
  bno.setExtCrystalUse(true);
}

void displayEvent(sensors_event_t* event) {
  display.setCursor(0,0);
  display.clearDisplay();
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem

  Serial.print("Version: ");
  Serial.print(event->version);
  Serial.print("; Sensor ID: ");
  Serial.print(event->sensor_id);
  Serial.print("; Type: ");
  Serial.println(event->type);
  
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    // From Adafruit_BNO055::VECTOR_LINEARACCEL: Acceleration not including gravity.
    display.println("LINEAR_ACCELERATION");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    // From Adafruit_BNO055::VECTOR_ACCELEROMETER or Adafruit_BNO055::VECTOR_GRAVITY
    display.println("ACCELEROMETER");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    // From Adafruit_BNO055::VECTOR_EULER
    display.println("ORIENTATION");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    // From Adafruit_BNO055::VECTOR_GYROSCOPE
    display.println("ROTATION_VECTOR");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // From Adafruit_BNO055::VECTOR_MAGNETOMETER
    display.println("MAGNETIC_FIELD");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else {
    display.println("UNKNOWN");
  }

  display.print("x: ");
  display.println(x);
  display.print("y: ");
  display.println(y);
  display.print("z: ");
  display.println(z);
  display.display();
}

void displayQuat(imu::Quaternion quat) {
  display.setCursor(0,0);
  display.clearDisplay();
  display.print("W: ");
  display.println(quat.w());
  display.print("X: ");
  display.println(quat.x());
  display.print("Y: ");
  display.println(quat.y());
  display.print("Z: ");
  display.println(quat.z());
  display.display();
}

void displayCalibration(uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) {
  display.setCursor(0,0);
  display.clearDisplay();
  display.print("Sys: ");
  display.println(sys);
  display.print("Gyro: ");
  display.println(gyro);
  display.print("Accel: ");
  display.println(accel);
  display.print("Mag: ");
  display.println(mag);
  display.display();
}

boolean calibration = true;
boolean quat = false;
uint8_t type_index = 0;


void loop() {
  if(!digitalRead(BUTTON_A)) {
    calibration = false; 
    quat = false;
    type_index += 1;
    if (type_index >= sizeof(types)) {
      type_index = 0;
    }
    delay(500);
  }
  
  if(!digitalRead(BUTTON_B)) {
    calibration = false; 
    quat = true; 
  }
  
  if(!digitalRead(BUTTON_C)) {
    calibration = true;
    quat = false;
  }

  if (calibration == true) {
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    displayCalibration(sys, gyro, accel, mag);
    Serial.print("Calibrated = ");
    Serial.println(bno.isFullyCalibrated());
  } else if (quat == true) {
    imu::Quaternion quat = bno.getQuat();           
    displayQuat(quat);
  }
  else {
    sensors_event_t event; 
    bno.getEvent(&event, types[type_index]);
    displayEvent(&event); 
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
