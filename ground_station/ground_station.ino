#include <SoftwareSerial.h>

SoftwareSerial HC12(8, 9); // HC-12 TX Pin, HC-12 RX Pin
static bool new_line = false;

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
}

void loop() {
  while (HC12.available()) {        // If HC-12 has data
    String receivedData = HC12.readStringUntil('\n');
    Serial.println(receivedData);   // Send the data to Serial monitor

    // Check if received data is "RESEND"
    if (receivedData.equals("RESEND")) {
      // Resend the last command
      //Serial.println("Resending last command..."); DEBUG
      sendLastCommand();
    } else {
      new_line = true;
    }
  }

  if (new_line) {
    Serial.println("");
    new_line = false;
  }

  while (Serial.available()) {      // If Serial monitor has data
    String command = Serial.readStringUntil('\n'); // Read the command
    sendCommand(command);          // Send the command
  }
}

String lastCommand = ""; // Variable to store the last command

void sendCommand(String command) {
  lastCommand = command;               // Store the command as last command
  HC12.println(command);               // Send the command to HC-12
}

void sendLastCommand() {
  if (lastCommand.length() > 0) {
    HC12.println(lastCommand);         // Resend the last command
  }
}
