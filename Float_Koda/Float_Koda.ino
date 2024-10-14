// File path: src/main.cpp

#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_VL53L0X.h>  // VL53L0X sensor library

// Create an instance of the VL53L0X sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define pins
const int potPin = A0;  // Pin for potentiometer
const int fanPin = 3;   // Pin to control transistor/MOSFET (PWM control)

// Variables for PID
double setpoint, input, output, output1, errOld;

// Adjust these PID values to reduce oscillation
double Kp = 0.9;  // Lower Kp to reduce the aggressive response
double Ki = 0.01;  // Set Ki to 0 to avoid integral windup for now
double Kd = 0;  // Increase Kd to dampen oscillations
double errI=0;
double errD=0;
double err=0;
double dt=0.001;
// PID controller instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE); // Set to REVERSE mode

// Maximum detectable range of the sensor in mm
const int maxSensorRange = 396;  // VL53L0X can measure up to around 1200 mm

void setup() {
  // Start Serial communication
  Serial.begin(9600);

  // Initialize fan pin
  pinMode(fanPin, OUTPUT);

  // Initialize the VL53L0X sensor
  if (!lox.begin()) {
    Serial.println("Failed to find VL53L0X sensor! Please check wiring.");
    while (1); // Halt execution if sensor initialization fails
  }

  Serial.println("VL53L0X sensor initialized!");

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range for fan speed

  // Initial setpoint (from potentiometer)
  setpoint = analogRead(potPin) * (360.0 / 1023.0);
}

void loop() {
  // Read the desired height (setpoint) from the potentiometer
  setpoint = analogRead(potPin) * (360.0 / 1023.0);

  // Variable to store the measurement data
  VL53L0X_RangingMeasurementData_t measure;

  // Perform a ranging test
  lox.rangingTest(&measure, false);  // Perform the ranging measurement
 input =measure.RangeMilliMeter; // Get the distance in millimeters

    // Map the sensor input to the 0-255 range for the PID controller
  input = 360- (map(input, 0, maxSensorRange, 0, 360));

  errOld=err;
  err=setpoint-input;
  errI=errI+err;
  errD=err-errOld;
  //output1 = Kp+(setpoint-input)*Ki;
  output1 = Kp*err+Ki*errI+Kd*errD;
  
  
   
    // Compute PID output to adjust the fan speed
    myPID.Compute();

    // Adjust the fan speed according to PID output
    analogWrite(fanPin, output1);

    // Output the values for monitoring
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Height: ");
    Serial.print(input);
    Serial.print(" | Fan Speed: ");
    Serial.println(output1);
 
  

  // Short delay for stability
    delay(dt);
}
