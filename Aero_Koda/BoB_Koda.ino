#include <AutomationShield.h>
#include <BOBShield.h> // Correct header file

void setup() {
  Serial.begin(9600); // Start serial communication for debugging
  
  BOBShield.begin(); // Initialize the BOBShield
  
  // Optional: If calibration is needed
  // BOBShield.calibration(); // Use the correct calibration method if available
}

void loop() {
  // Reading the sensor value
  float sensorValue = BOBShield.sensorRead(); // Use the correct method to read sensor data
  Serial.print("Sensor value: ");
  Serial.println(sensorValue);
  
  // Define the setpoint
  float setpoint = 50.0; // Desired value
  
  // Compute error
  float error = setpoint - sensorValue;
  
  // Simple proportional control
  float Kp = 0.1; // Proportional gain
  float controlSignal = Kp * error;
  
  // Apply control signal
  BOBShield.actuatorWrite(controlSignal); // Use the correct method to write to the actuator
  
  // Wait before next iteration
  delay(100);
}
