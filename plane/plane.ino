// Include the necessary libraries
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TimeLib.h>

// Create MPU and Magnetometer instances
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Create servo objects
Servo elevator;
Servo rudder;
Servo aileron_left;
Servo aileron_right;

// Logging flag
bool log_enabled = false;

// Angle limits for control surfaces around a central position of 90 degrees
const int SERVO_CENTER = 90;
const int MAX_ANGLE = 180;

// HC-12 Serial communication
SoftwareSerial HC12(7, 6); // HC-12 TX Pin, HC-12 RX Pin

// Variables to store the last input angles for the servos
float lastElevatorAngle = 0.0;
float lastRudderAngle = 0.0;
float lastAileronLeftAngle = 0.0;
float lastAileronRightAngle = 0.0;


float maxPitch, maxYaw, maxRoll;
float minPitch, minYaw, minRoll;
float offsetPitch, offsetYaw, offsetRoll;
float scalePitch, scaleYaw, scaleRoll;


// PID parameters declaration


struct StateValues {

  
}

struct InputValues {
  float delta_e
  float delta_a
  float delta_r

}
float Pa, Ia, Da, Pe, Ie, De, Pr, Ir, Drs

void setup() {
  // Attach servos to pins
  elevator.attach(11);
  rudder.attach(10);
  aileron_left.attach(2);
  aileron_right.attach(3);

  // Set all servos to the central position
  elevator.write(SERVO_CENTER);
  rudder.write(SERVO_CENTER);
  aileron_left.write(SERVO_CENTER);
  aileron_right.write(SERVO_CENTER);

  // Initialize Serial communication for debugging
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Initialize HC-12 Serial communication
  HC12.begin(9600);

  // Print CSV header
  Serial.println("Timestamp,Pitch,Roll,Yaw,Elevator,Rudder,Aileron_Left,Aileron_Right");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Make HMC5883L accessible
  mpu.setI2CBypass(true);
  
  // Initialize HMC5883L compass
  if (!mag.begin()) {
    Serial.println("Failed to connect to HMC5883L compass");
    while (1) {
      delay(10);
    }
  }
  delay(100);

  calibrateSensor();
  // Send ready message via HC-12
  HC12.println("INFO: ready to receive commands");
}

void loop() {
  // Check for incoming HC-12 messages
  if (HC12.available() > 0) {
    String command = HC12.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace
    Serial.print("Received command: ");
    Serial.println(command);

    if (isValidCommand(command)) {
      processCommand(command);
    } else {
      Serial.println("Invalid command received, requesting resend");
      HC12.println("RESEND");
    }
  }

  // Get new sensor events with the readings
  sensors_event_t a, g, temp, magEvent;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&magEvent);

  // Calculate pitch and roll (in degrees)
  float pitch = (a.gyro.pitch - offsetPitch) * 180 / scalePitch;
  float roll = (a.gyro.roll - offsetRoll) * 180 / scaleRoll;

  // Calculate yaw (in degrees)
  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  float yaw = (((heading * 180.0 / M_PI) - offsetYaw)/scaleYaw) * 180;

  // Create log message with timestamp
  String logMessage = String(now()) + "," + String(pitch) + "," + String(roll) + "," + String(yaw) + "," +
                      String(lastElevatorAngle) + "," + String(lastRudderAngle) + "," +
                      String(lastAileronLeftAngle) + "," + String(lastAileronRightAngle);

  // Print log if logging is enabled
  if (log_enabled) {
    Serial.println(logMessage);
    HC12.println(logMessage);
  }

  delay(50); // Delay to simulate 500 ms loop time
}

void processCommand(String command) {
  if (command.equals("START LOG")) {
    log_enabled = true;
    Serial.println("Logging started");
    HC12.println("INFO: Logging started");
  } else if (command.equals("STOP LOG")) {
    log_enabled = false;
    Serial.println("Logging stopped");
    HC12.println("INFO: Logging stopped");
  } else if (command.startsWith("SET_SURFACES ")) {
    processSetSurfacesCommand(command.substring(13));
  } else {
    Serial.println("Unknown command");
    HC12.println("INFO: Unknown command");
  }
}

void processSetSurfacesCommand(String angles) {
  int firstCommaIndex = angles.indexOf(',');
  int secondCommaIndex = angles.indexOf(',', firstCommaIndex + 1);
  int thirdCommaIndex = angles.indexOf(',', secondCommaIndex + 1);

  if (firstCommaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1) {
    Serial.println("Invalid SET_SURFACES command format");
    HC12.println("INFO: Invalid SET_SURFACES command format");
    return;
  }

  float rightAileronAngle = angles.substring(0, firstCommaIndex).toFloat();
  float leftAileronAngle = angles.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  float rudderAngle = angles.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
  float elevatorAngle = angles.substring(thirdCommaIndex + 1).toFloat();

  aileron_right.write(mapServoValue(rightAileronAngle));
  aileron_left.write(mapServoValue(leftAileronAngle));
  rudder.write(mapServoValue(rudderAngle));
  elevator.write(mapServoValue(elevatorAngle));

  // Update the last input angles for the servos
  lastAileronRightAngle = rightAileronAngle;
  lastAileronLeftAngle = leftAileronAngle;
  lastRudderAngle = rudderAngle;
  lastElevatorAngle = elevatorAngle;

  String response = "Surfaces set to " + String(rightAileronAngle) + "," + String(leftAileronAngle) + "," + String(rudderAngle) + "," + String(elevatorAngle);
  Serial.println(response);
  HC12.println(response);
}

bool isValidCommand(String command) {
  if (command.equals("START LOG") || command.equals("STOP LOG")) {
    return true;
  }
  if (command.startsWith("SET_SURFACES ")) {
    String angles = command.substring(13);
    int firstCommaIndex = angles.indexOf(',');
    int secondCommaIndex = angles.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = angles.indexOf(',', secondCommaIndex + 1);

    if (firstCommaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1) {
      return false;
    }

    float rightAileronAngle = angles.substring(0, firstCommaIndex).toFloat();
    float leftAileronAngle = angles.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
    float rudderAngle = angles.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
    float elevatorAngle = angles.substring(thirdCommaIndex + 1).toFloat();

    if (rightAileronAngle >= -1 && rightAileronAngle <= 1 &&
        leftAileronAngle >= -1 && leftAileronAngle <= 1 &&
        rudderAngle >= -1 && rudderAngle <= 1 &&
        elevatorAngle >= -1 && elevatorAngle <= 1) {
      return true;
    }
  }
  return false;
}

float normalizeServoAngle(int angle) {
  return (angle - SERVO_CENTER) / float(SERVO_CENTER);
}

int mapServoValue(float value) {
  return SERVO_CENTER + value * SERVO_CENTER;
}

float

void calibrateSensor() {
  Serial.println("Calibrating...");
  HC12.println("INFO: Start Calibration Procedure");
  int numReadings = 500;
  for (int i = 0; i < numReadings; i++) {
  sensors_event_t a, g, temp, magEvent;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&magEvent);
   // Calculate yaw (in degrees)
  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  float yaw = heading * 180.0 / M_PI;

  if(i == 0) {
    maxYaw = yaw;
    maxPitch = a.gyro.pitch;
    maxRoll = a.gyro.roll;
    minYaw = yaw;
    minPitch = a.gyro.pitch;
    minRoll = a.gyro.roll;
  } else {
    maxYaw = (maxYaw < yaw)? yaw : maxYaw;
    maxPitch = (maxPitch < a.gyro.pitch)? a.gyro.pitch : maxPitch;
    maxRoll = (maxRoll < a.gyro.roll)? a.gyro.roll : maxRoll;

    minYaw = (minYaw > yaw)? yaw : minYaw;
    minPitch = (minPitch > a.gyro.pitch)? a.gyro.pitch : minPitch;
    minRoll = (minRoll > a.gyro.roll)? a.gyro.roll : minRoll;
  }

    delay(100);
  }

  // Calculates the offset and scale amplitudes
  offsetYaw = (minYaw + maxYaw)/2;
  scaleYaw = maxYaw - minYaw;

  offsetRoll = (minRoll + maxRoll)/2;
  scaleRoll = maxRoll - minRoll;

  offsetPitch = (minPitch + maxPitch)/2;
  scalePitch = maxPitch - minPitch;

  // Print Max values
  Serial.print("Max Values: ");
  Serial.print("Pitch: ");
  Serial.print(maxPitch);
  Serial.print(", Roll: ");
  Serial.print(maxRoll);
  Serial.print(", Yaw: ");
  Serial.print(maxYaw);

  // Print Max values
  Serial.print("Min Values: ");
  Serial.print("Pitch: ");
  Serial.print(minPitch);
  Serial.print(", Roll: ");
  Serial.print(minRoll);
  Serial.print(", Yaw: ");
  Serial.print(minYaw);
  Serial.println("Calibration complete.");
  HC12.println("INFO: Calibration Completed!");

}