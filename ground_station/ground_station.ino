#include <RH_NRF24.h>
#include <SPI.h>

// NRF24 radio communication
RH_NRF24 nrf24;

static bool new_line = false;
String lastCommand = ""; // Variable to store the last command

void setup() {
  Serial.begin(57600); // Serial port to computer

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

  Serial.println("NRF24 initialized and ready");
}

void loop() {
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Check for incoming NRF24 messages
  if (nrf24.available()) {
    if (nrf24.recv(buf, &len)) {
      buf[len] = 0; // Null-terminate string
      String receivedData = String((char*)buf);
      Serial.println(receivedData); // Send the data to Serial monitor

      // Check if received data is "RESEND"
      if (receivedData.equals("RESEND")) {
        sendLastCommand(); // Resend the last command
      } else {
        new_line = true;
      }
    }
  }

  if (new_line) {
    Serial.println("");
    new_line = false;
  }

  while (Serial.available()) { // If Serial monitor has data
    String command = Serial.readStringUntil('\n'); // Read the command
    sendCommand(command); // Send the command
  }
}

void sendCommand(String command) {
  lastCommand = command; // Store the command as last command
  uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
  command.getBytes(data, sizeof(data)); // Convert the command to a byte array
  nrf24.send(data, command.length()); // Send the command via NRF24
  nrf24.waitPacketSent(); // Wait for the packet to be sent
}

void sendLastCommand() {
  if (lastCommand.length() > 0) {
    sendCommand(lastCommand); // Resend the last command
  }
}
