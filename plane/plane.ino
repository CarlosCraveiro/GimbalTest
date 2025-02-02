// Include the necessary libraries
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TimeLib.h>
#include <RH_NRF24.h>

// A counter for decrease the frequency of the sending messages
int a_counter = 0;
int test_counter = 0;

int current_elevator = 0;
int current_left_aileron = 0;

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

// PID control flag
bool pid_enabled = false;

// test flag
bool testing_enabled = false;

// Angle limits for control surfaces around a central position of 90 degrees
const int AILERON_L_CENTER = 80;
const int AILERON_R_CENTER = 100;
const int ELEVATOR_CENTER = 90;
const int RUDDER_CENTER = 95;
const int SERVO_CENTER = 90;
const int MAX_ANGLE = 180;

// NRF24 radio communication
RH_NRF24 nrf24(7,8);

int max_aileron = 20;
int max_elevator = 20;

// Variables to store the last input angles for the servos
float lastElevatorAngle = 0.0;
float lastRudderAngle = 0.0;
float lastAileronLeftAngle = 0.0;
float lastAileronRightAngle = 0.0;

float maxPitch, maxYaw, maxRoll;
float minPitch, minYaw, minRoll;
float offsetPitch, offsetYaw, offsetRoll;
float scalePitch, scaleYaw, scaleRoll;

// PID coefficients
double P_pitch = 20, I_pitch = 0.0, D_pitch = 2.0;
double P_roll = 10 , I_roll = 0.0, D_roll = 2.0;
double P_yaw = 0.0, I_yaw = 0.0, D_yaw = 0.0;

/* Code for MPU Gyro and Kalman*/
const int MPU_addr = 0x68; // I2C address of the MPU-6050

double Q = 4*4;
double R = 3*3;
long loop_timer;

int16_t GyX, GyY, GyZ, AcX, AcY, AcZ;
double AngleRoll, AnglePitch, RateRoll, RatePitch, RatYaw, GyX_cal, GyY_cal, GyZ_cal, AccelX, AccelY, AccelZ, AngleGyroRoll, AngleGyroPitch;

//Kalman parameters
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=4;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=4;
float Kalman1DOutput[]={0,0};
/* END */


class PIDController {
private:
    double Kp, Kd, Ki;     // PID coefficients
    double setpoint;       // Desired setpoint
    double last_error;     // Last error for derivative calculation
    double total_error;    // Accumulated error for integral calculation
    double control_signal; // Output control signal
    double max_control;    // Maximum control signal
    double min_control;    // Minimum control signal
    unsigned long last_time; // Last time the control was calculated


public:
    PIDController(double kp, double kd, double ki, double maxCtrl, double minCtrl)
        : Kp(kp), Kd(kd), Ki(ki), max_control(maxCtrl), min_control(minCtrl),
          setpoint(0), last_error(0), total_error(0), control_signal(0), last_time(0) {}

    void setSetpoint(double sp) {
        setpoint = sp;
    }

    double getSetpoint() const {
        return setpoint;
    }

    double getControlSignal() const {
        return control_signal;
    }

    void setMax(double max) {
      max_control = max;
    }

    void setMin(double min) {
      min_control = min;
    }

    void setGains(double p, double i, double d) {
      Kp = p; Ki = i; Kd = d;
    }

    void compute(double sensed_output, double offset, int T) {
        unsigned long current_time = millis(); // Get current time
        int delta_time = current_time - last_time; // Time difference

        //if (delta_time >= T) {
            double error = setpoint - sensed_output;
            total_error = error; // Accumulate error
            //total_error += error; // Accumulate error

            // Limit total error (Integral windup protection)
            if (total_error >= max_control) total_error = max_control;
            else if (total_error <= min_control) total_error = min_control;

            double delta_error = error - last_error; // Change in error

            // Compute control signal
            control_signal = Kp * error + (Kd * delta_error/delta_time) + Ki * (last_error + delta_error/2) * delta_time + offset;
            // Limit control signal
            if (control_signal >= max_control) control_signal = max_control;
            else if (control_signal <= min_control) control_signal = min_control;

            last_error = error; // Store current error for next derivative calculation
            last_time = current_time; // Update last_time
        //}
    }
};


// PID controllers
PIDController pitchPID(P_pitch, D_pitch, I_pitch, 30 + ELEVATOR_CENTER, ELEVATOR_CENTER - 30);
PIDController rollPID(P_roll, D_roll, I_roll, AILERON_R_CENTER + 25, AILERON_R_CENTER - 25);
PIDController yawPID(P_yaw, D_yaw, I_yaw, 30, -30);

void setup() {
  // Attach servos to pins
  elevator.attach(6);
  rudder.attach(5);
  aileron_left.attach(4);
  aileron_right.attach(3);

  // Set all servos to the central position
  elevator.write(ELEVATOR_CENTER);
  rudder.write(RUDDER_CENTER);
  aileron_left.write(AILERON_L_CENTER);
  aileron_right.write(AILERON_R_CENTER);

  // Initialize Serial communication for debugging
  Serial.begin(57600);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Initialize NRF24 communication
  if (!nrf24.init()) {
    Serial.println("NRF24 initialization failed");
    while (1);
  }
  if (!nrf24.setChannel(1)) {
    Serial.println("Failed to set channel");
    while (1);
  }
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
    Serial.println("Failed to set RF parameters");
    while (1);
  }

  // Print  header
  Serial.println("Timestamp,Pitch,Roll,Yaw,Elevator,Rudder,Aileron_Left,Aileron_Right");

  // Initialize MPU6050
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    setupMPU();
    calibrateMPU();
    loop_timer = micros();

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

  //calibrateSensor();

  // Send ready message via NRF24
  Serial.println("INFO: ready to receive commands");
  sendStringOverNRF("INFO: ARMED!");
}

void loop() {
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Check for incoming NRF24 messages
  if (nrf24.available()) {
  //if (Serial.available() > 0) {
    //String command = Serial.readStringUntil('\n');
    
    if (nrf24.recv(buf, &len)) {
      buf[len] = 0; // Null-terminate string
      String command = String((char*)buf);
      command.trim(); // Remove any leading/trailing whitespace
      Serial.print("Received command: ");
      Serial.println(command);

      if (isValidCommand(command)) {
        processCommand(command);
      } else {
        Serial.println("Invalid command received, requesting resend");
        Serial.println("RESEND");
      }
    }
  }

  if(testing_enabled) {
    if(--test_counter == 0) {
      testing_enabled = false;
      log_enabled = false;
       sendStringOverNRF("INFO: Test END");
    }
  }

  // Get new sensor events with the readings
  sensors_event_t magEvent;
  //mpu.getEvent(&a, &g, &temp);
  //mag.getEvent(&magEvent);

  double input_yaw = 0;

  /* KALMAN CODE */
   read_gyro_data();
  RateRoll = double(GyX-GyX_cal)/65.5;
  RatePitch = double(GyY-GyY_cal)/65.5;
  RatYaw = double(GyZ-GyZ_cal)/65.5;
 
 read_accel_data();
 AngleRoll = atan(AccelY/sqrt(AccelX*AccelX+AccelZ*AccelZ))*1/(3.142/180);
 AnglePitch = atan(-AccelX/sqrt(AccelY*AccelY+AccelZ*AccelZ))*1/(3.142/180);

 kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
 KalmanAngleRoll=Kalman1DOutput[0]; 
 KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
 kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
 KalmanAnglePitch=Kalman1DOutput[0]; 
 KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
 //Serial.print("Roll Angle [°] ");
 //Serial.print(KalmanAngleRoll);
 //Serial.print(" Pitch Angle [°] ");
 //Serial.println(KalmanAnglePitch);

 while(micros() - loop_timer < 5000); //Wait untilcelX/sqrt(AccelY*AccelY+AccelZ*AccelZ))*1/(3.142/180);

 kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
 KalmanAngleRoll=Kalman1DOutput[0]; 
 KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
 kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
 KalmanAnglePitch=Kalman1DOutput[0]; 
 KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
 //Serial.print("Roll Angle [°] ");
 //Serial.print(KalmanAngleRoll);
 //Serial.print(" Pitch Angle [°] ");
 //Serial.println(KalmanAnglePitch);

 loop_timer = micros();
  /*END KALMAN CODE */
  /*KalmanAnglePitch and KalmanAngleRoll*/
  
  float input_roll = KalmanAngleRoll;
  float input_pitch = KalmanAnglePitch;

  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  //input_yaw = (((heading * 180.0 / M_PI) - offsetYaw) / scaleYaw) * 180;
  if (pid_enabled) {
    // Compute PID outputs
    pitchPID.compute(input_pitch, 100, ELEVATOR_CENTER); // 100 ms interval
    rollPID.compute(input_roll, 100, AILERON_R_CENTER); ///A AAAA CHANGE THIS AFTER
    //yawPID.compute(input_yaw, 100, RUDDER_CENTER);

    // Get control signals
    double output_pitch = pitchPID.getControlSignal();
    double output_roll = rollPID.getControlSignal();
    //double output_yaw = yawPID.getControlSignal();

    // Set control surfaces based on PID outputs
    //Serial.println("SERVO COMMAND");
    //Serial.println(output_pitch);
      elevator.write(output_pitch);
      current_elevator = output_pitch - ELEVATOR_CENTER;
      aileron_left.write(2*AILERON_L_CENTER - (output_roll-20));
      current_left_aileron = AILERON_L_CENTER - (output_roll-20);
      aileron_right.write(2*AILERON_R_CENTER - output_roll); // Assuming opposite movement for balance
    //rudder.write(mapServoValue(output_yaw));
  }

  // Create log message with timestamp
  String logMessage = String(now()) + "," + String(input_pitch) + "," + String(-input_roll) + "," +
                      String(current_elevator) + "," +
                      String(current_left_aileron);
                     
  // Print log if logging is enabled
  if (log_enabled) {
    Serial.println(logMessage);
    a_counter++;
    if(a_counter = 5) {
      sendStringOverNRF(logMessage);
      a_counter = 0;
    }
  }

  delay(50); // Delay to simulate 50 ms loop time
}

void processCommand(String command) {
  if (command.equals("START LOG")) {
    log_enabled = true;
    Serial.println("Logging started");
    sendStringOverNRF("INFO: Logging started");
  } else if (command.equals("STOP LOG")) {
    log_enabled = false;
    Serial.println("Logging stopped");
    sendStringOverNRF("INFO: Logging stopped");
  } else if (command.equals("PID ON")) {
    pid_enabled = true;
    Serial.println("PID control enabled");
    sendStringOverNRF("INFO: PID control enabled");
  } else if (command.equals("PID OFF")) {
    pid_enabled = false;
    Serial.println("PID control disabled");
    sendStringOverNRF("INFO: PID control disabled");
  } else if (command.equals("RESET SETPOINT")) {
    resetSetpoints();
    Serial.println("Setpoints reset to current mean values");
    sendStringOverNRF("INFO: Setpoints reset");
  } else if (command.startsWith("SET_SURFACES ")) {
    processSetSurfacesCommand(command.substring(13));
  } else if (command.startsWith("SET_PITCH_POSITION ")) {
    pitchPID.setSetpoint(command.substring(19).toFloat());
    Serial.print("Pitch setpoint set to: ");
    Serial.println(pitchPID.getSetpoint());
  } else if (command.startsWith("SET_PID_GAINS ")) {
    processSetPIDGainsCommand(command.substring(14));
  } else if (command.startsWith("TEST ")) {
    processTestCommand(command.substring(5));
  } else if (command.startsWith("CHANGE ")) {
    processChangeCommand(command.substring(7));
  } else {
    Serial.println("Unknown command");
    sendStringOverNRF("INFO: Unknown command");
  }
}

void resetSetpoints() {
  const int numReadings = 500;
  double avgPitch = 0, avgRoll = 0, avgYaw = 0;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t magEvent;

 /* KALMAN CODE */
   read_gyro_data();
  RateRoll = double(GyX-GyX_cal)/65.5;
  RatePitch = double(GyY-GyY_cal)/65.5;
  RatYaw = double(GyZ-GyZ_cal)/65.5;
 
 read_accel_data();
 AngleRoll = atan(AccelY/sqrt(AccelX*AccelX+AccelZ*AccelZ))*1/(3.142/180);
 AnglePitch = atan(-AccelX/sqrt(AccelY*AccelY+AccelZ*AccelZ))*1/(3.142/180);

 kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
 KalmanAngleRoll=Kalman1DOutput[0]; 
 KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
 kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
 KalmanAnglePitch=Kalman1DOutput[0]; 
 KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
 //Serial.print("Roll Angle [°] ");
 //Serial.print(KalmanAngleRoll);
 //Serial.print(" Pitch Angle [°] ");
 //Serial.println(KalmanAnglePitch);

 while(micros() - loop_timer < 5000); //Wait untilcelX/sqrt(AccelY*AccelY+AccelZ*AccelZ))*1/(3.142/180);

 kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
 KalmanAngleRoll=Kalman1DOutput[0]; 
 KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
 kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
 KalmanAnglePitch=Kalman1DOutput[0]; 
 KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
 //Serial.print("Roll Angle [°] ");
 //Serial.print(KalmanAngleRoll);
 //Serial.print(" Pitch Angle [°] ");
 //Serial.println(KalmanAnglePitch);

 loop_timer = micros();
  /*END KALMAN CODE */
  /*KalmanAnglePitch and KalmanAngleRoll*/

    avgPitch += KalmanAnglePitch/numReadings;
    avgRoll += KalmanAngleRoll/ numReadings;
    //sumYaw += yaw;

    delay(10); // Small delay between readings
  }

  pitchPID.setSetpoint(avgPitch);
  rollPID.setSetpoint(avgRoll);
  Serial.println(avgRoll);
  //yawPID.setSetpoint(setpoint_yaw);
}

void processChangeCommand(String params) {
  int firstCommaIndex = params.indexOf(',');

  if (firstCommaIndex == -1) {
    Serial.println("Invalid CHANGE command format");
    sendStringOverNRF("INFO: Invalid CHANGE command format");
    return;
  }

  max_elevator = params.substring(0, firstCommaIndex).toFloat();
  max_aileron = params.substring(firstCommaIndex + 1).toFloat();
/*// PID controllers
PIDController pitchPID(P_pitch, D_pitch, I_pitch, 30 + ELEVATOR_CENTER, ELEVATOR_CENTER - 30);
PIDController rollPID(P_roll, D_roll, I_roll, AILERON_R_CENTER + 25, AILERON_R_CENTER - 25);
*/
  pitchPID.setMax(max_elevator + ELEVATOR_CENTER);
  pitchPID.setMin(-max_elevator + ELEVATOR_CENTER);

  rollPID.setMax(max_aileron + AILERON_R_CENTER);
  rollPID.setMin(-max_elevator + AILERON_R_CENTER);

  sendStringOverNRF("Changes were made");
}

void processTestCommand(String params) {
  int firstSpace = params.indexOf(' ');
  int secondSpace = params.indexOf(' ', firstSpace + 1);

  if (firstSpace == -1 || secondSpace == -1) {
    Serial.println("Invalid TEST command format");
    sendStringOverNRF("INFO: Invalid TEST command format");
    return;
  }

  String surface = params.substring(0, firstSpace);
  float angle = params.substring(firstSpace + 1, secondSpace).toFloat();
  int duration = params.substring(secondSpace + 1).toInt();

  if (surface == "ELEVATOR") {
    elevator.write(angle + ELEVATOR_CENTER);
    current_elevator = angle;
    Serial.println("Testing Elevator");
  } else if (surface == "RUDDER") {
    rudder.write(angle + RUDDER_CENTER);
    Serial.println("Testing Rudder");
  } else if (surface == "AILERON") {
    aileron_left.write(AILERON_L_CENTER + angle);
    current_left_aileron = angle;
    aileron_right.write(AILERON_R_CENTER + angle);
    Serial.println("Testing Ailerons");
  } else {
    Serial.println("Invalid control surface");
    sendStringOverNRF("INFO: Invalid control surface");
    return;
  }


  // Start logging for the specified duration
  testing_enabled = true;
  log_enabled = true;
  test_counter = duration;
}

void processSetSurfacesCommand(String angles) {
  int firstCommaIndex = angles.indexOf(',');
  int secondCommaIndex = angles.indexOf(',', firstCommaIndex + 1);
  int thirdCommaIndex = angles.indexOf(',', secondCommaIndex + 1);

  if (firstCommaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1) {
    Serial.println("Invalid SET_SURFACES command format");
    sendStringOverNRF("INFO: Invalid SET_SURFACES command format");
    return;
  }

  float rightAileronAngle = angles.substring(0, firstCommaIndex).toFloat();
  float leftAileronAngle = angles.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  float rudderAngle = angles.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
  float elevatorAngle = angles.substring(thirdCommaIndex + 1).toFloat();

  aileron_right.write(mapServoValue(rightAileronAngle, AILERON_R_CENTER, 50));
  aileron_left.write(mapServoValue(leftAileronAngle, AILERON_L_CENTER, 50));
  rudder.write(mapServoValue(rudderAngle, RUDDER_CENTER, 25));
  elevator.write(mapServoValue(elevatorAngle, ELEVATOR_CENTER, 70 ));

  // Update the last input angles for the servos
  lastAileronRightAngle = rightAileronAngle;
  lastAileronLeftAngle = leftAileronAngle;
  current_left_aileron = mapServoValue(leftAileronAngle, AILERON_L_CENTER, 50) - AILERON_L_CENTER;
  lastRudderAngle = rudderAngle;
  lastElevatorAngle = elevatorAngle;
  current_elevator = mapServoValue(elevatorAngle, ELEVATOR_CENTER, 70 ) - ELEVATOR_CENTER;

  String response = "Surfaces set to " + String(rightAileronAngle) + "," + String(leftAileronAngle) + "," + String(rudderAngle) + "," + String(elevatorAngle);
  Serial.println(response);
  Serial.println(response);
}

void processSetPIDGainsCommand(String gains) {
  int firstCommaIndex = gains.indexOf(',');
  int secondCommaIndex = gains.indexOf(',', firstCommaIndex + 1);
  int thirdCommaIndex = gains.indexOf(',', secondCommaIndex + 1);
  int fourthCommaIndex = gains.indexOf(',', thirdCommaIndex + 1);
  int fifthCommaIndex = gains.indexOf(',', fourthCommaIndex + 1);
  //int sixthCommaIndex = gains.indexOf(',', fifthCommaIndex + 1);
  //int seventhCommaIndex = gains.indexOf(',', sixthCommaIndex + 1);
  //int eighthCommaIndex = gains.indexOf(',', seventhCommaIndex + 1);

  if (firstCommaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1 ||
      fourthCommaIndex == -1 || fifthCommaIndex == -1) {
    Serial.println("Invalid SET_PID_GAINS command format");
    sendStringOverNRF("INFO: Invalid SET_PID_GAINS command format");
    return;
  }

  P_pitch = gains.substring(0, firstCommaIndex).toFloat();
  I_pitch = gains.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  D_pitch = gains.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
  P_roll = gains.substring(thirdCommaIndex + 1, fourthCommaIndex).toFloat();
  I_roll = gains.substring(fourthCommaIndex + 1, fifthCommaIndex).toFloat();
  D_roll = gains.substring(fifthCommaIndex + 1).toFloat();
  //P_yaw = gains.substring(sixthCommaIndex + 1, seventhCommaIndex).toFloat();
  //I_yaw = gains.substring(seventhCommaIndex + 1, eighthCommaIndex).toFloat();
  //D_yaw = gains.substring(eighthCommaIndex + 1).toFloat();

  // Update PID controllers with new gains
  pitchPID.setGains(P_pitch, I_pitch, D_pitch);
  rollPID.setGains(P_roll, I_roll, D_roll);
  //yawPID.setGains(P_yaw, I_yaw, D_yaw);

  String response = "PID: " + String(int(P_pitch)) + "," + String(int(I_pitch)) + "," + String(int(D_pitch)) + "," +
                   String(int(P_roll)) + "," + String(int(I_roll)) + "," + String(int(D_roll));
  Serial.println(response);
  sendStringOverNRF(response);
}

bool isValidCommand(String command) {
  if (command.equals("START LOG") || command.equals("STOP LOG") || command.equals("PID ON") || command.equals("PID OFF") || command.equals("RESET SETPOINT")) {
    return true;
  }
  if (command.startsWith("SET_SURFACES ") || command.startsWith("SET_PITCH_POSITION ") || command.startsWith("SET_PID_GAINS ")) {
    return true;
  }
  if (command.startsWith("TEST ") || command.startsWith("CHANGE ")) {
    return true;
  }
  return false;
}

float normalizeServoAngle(int angle, int servoCenter) {
  return (angle - servoCenter) / float(servoCenter);
}

int mapServoValue(float value, int servoCenter, int servoAmplitude) {
  return servoCenter + value * servoAmplitude;
}

// Function to send a string via NRF24
void sendStringOverNRF(String message) {
  // Convert the String to a C-style string (null-terminated character array)
  const char* cstr = message.c_str();

  // Send the message
  if (nrf24.send((uint8_t*)cstr, strlen(cstr))) {
    nrf24.waitPacketSent(); // Wait until the packet is actually sent
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Failed to send message");
  }
}

void setupMPU() {
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B); // PWR_MGMT_1 register
 Wire.write(0); // set to zero (wakes up the MPU-6050)
 Wire.endTransmission();

 Wire.beginTransmission(MPU_addr); //Low pass filter
 Wire.write(0x1A);
 Wire.write(0x05);
 Wire.endTransmission();

 Wire.beginTransmission(MPU_addr);
 Wire.write(0x1B);
 Wire.write(0x8);
 Wire.endTransmission();

 Wire.beginTransmission(MPU_addr);
 Wire.write(0x1C);
 Wire.write(0x10);
 Wire.endTransmission();
}

void calibrateMPU() {
 //int n = 10000;
 for (int cal_int = 0; cal_int < 2000; cal_int++) {
 read_gyro_data();
 GyX_cal += GyX;
 GyY_cal += GyY;
 GyZ_cal += GyZ;
 }
 GyX_cal /= 2000;
 GyY_cal /= 2000;
 GyZ_cal /= 2000;
}

void read_gyro_data() {
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x43);
 Wire.endTransmission();
 Wire.requestFrom(MPU_addr, 6);
 GyX = Wire.read() << 8 | Wire.read();
 GyY = Wire.read() << 8 | Wire.read();
 GyZ = Wire.read() << 8 | Wire.read();
}

void read_accel_data() {
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);
 Wire.endTransmission();
 Wire.requestFrom(MPU_addr, 6);
 AcX = Wire.read() << 8 | Wire.read();
 AcY = Wire.read() << 8 | Wire.read();
 AcZ = Wire.read() << 8 | Wire.read();

 AccelX = double(AcX)/4096-0.03;
 AccelY = double(AcY)/4096;
 AccelZ = double(AcZ)/4096-0.04;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
 float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + R);

 KalmanState=KalmanState+0.005*KalmanInput;
 KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
 
 KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
 KalmanUncertainty=KalmanUncertainty + 0.005 * 0.005 * Q;

 Kalman1DOutput[0]=KalmanState; 
 Kalman1DOutput[1]=KalmanUncertainty;
}