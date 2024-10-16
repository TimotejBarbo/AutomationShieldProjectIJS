#include <Wire.h>  // Include the Wire library for I2C communication
#include <PID_v1.h>  // Include the PID library

// AS5600L I2C Address
#define AS5600L_ADDR 0x40  // I2C address for AS5600L

// AS5600L Register Addresses
#define AS5600L_REG_ANGLE 0x0C  // Register to read angle

// Motor control pin
const int motorPin = 5;  // Motor control pin

// Potentiometer pin
const int potPin = A3;

// Target angle and control parameters
double targetAngle = 0.0;  // Desired angle to hold
const double targetAngleTolerance = 1.0;  // Tolerance in degrees for angle holding
const double setpointOffset = 2; // Offset to adjust the setpoint

// PID control parameters
double kp = 1.5;   // Proportional gain (reduced to prevent overshoot)
double ki = 0.3;   // Integral gain (unchanged)
double kd = 0.5;  // Derivative gain (increased to dampen oscillations)

// PID variables
double currentAngle = 0.0;  // Current angle from the sensor
double motorSpeed = 0.0;    // Motor speed calculated by PID
double error = 0.0;         // Error between target and current angle

// PID controller
PID myPID(&currentAngle, &motorSpeed, &targetAngle, kp, ki, kd, DIRECT);

// Timeout for sensor reading
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 200;  // 200 ms timeout

// Low-pass filter constant for smoothing sensor readings
const double alpha = 0.1;  // Smoothing factor, 0 < alpha < 1

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
  Wire.begin();        // Join I2C bus as master

  // Initialize the AS5600L sensor
  if (!initializeAS5600L()) {
    Serial.println("AS5600L initialization failed!");
    while (1);  // Halt program if initialization fails
  }
  Serial.println("AS5600L initialized successfully.");

  // Set up motor pin
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);  // Initialize motor to off

  // Set up PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Limit motor speed to 0 to 255 for forward direction only
}

void loop() {
  unsigned long currentTime = millis();

  // Check if the time interval has passed
  if (currentTime - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = currentTime;

    // Read current angle from AS5600L
    double rawAngle = readAngle();

    // Apply low-pass filter to smooth the angle reading
    currentAngle = alpha * rawAngle + (1.0 - alpha) * currentAngle;

    // Map potentiometer value to 60-200 degree range
    int potValue = analogRead(potPin);
    targetAngle = map(potValue, 0, 1023, 60, 200);  
    targetAngle += setpointOffset;  // Apply offset to target angle

    // Calculate error
    error = targetAngle - currentAngle;

    // Compute PID output
    myPID.Compute();

    // Control motor to hold the target angle
    controlMotor(motorSpeed);

    // Print debug information
    Serial.print("Target Angle: ");
    Serial.print(targetAngle);
    Serial.print(" degrees, Current Angle: ");
    Serial.print(currentAngle);
    Serial.print(" degrees, Motor Speed: ");
    Serial.println(motorSpeed);
  }

  delay(50);  // Adjust delay for control loop speed
}

// Function to initialize the AS5600L sensor
bool initializeAS5600L() {
  Wire.beginTransmission(AS5600L_ADDR);
  Wire.write(AS5600L_REG_ANGLE);  // Set register pointer to angle register
  return (Wire.endTransmission() == 0);
}

// Function to read angle from AS5600L
double readAngle() {
  Wire.beginTransmission(AS5600L_ADDR);
  Wire.write(AS5600L_REG_ANGLE);  // Set register pointer to angle register
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600L_ADDR, 2);  // Request 2 bytes of data
  if (Wire.available() == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int angle = (highByte << 8) | lowByte;  // Combine high and low byte
    return angle * 360.0 / 4096;  // Convert raw angle to degrees
  }
  return -1;  // Return -1 if reading failed
}

// Function to control the motor based on the PID output
void controlMotor(double motorSpeed) {
  // Ensure motor speed is within the valid range
  motorSpeed = constrain(motorSpeed, 0, 255);

  // Set motor speed; if speed is negative, stop the motor
  if (motorSpeed > 0) {
    analogWrite(motorPin, motorSpeed);  // Forward direction
  } else {
    analogWrite(motorPin, 0);  // Stop motor
  }
}
