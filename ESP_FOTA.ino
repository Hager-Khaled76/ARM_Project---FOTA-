
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

#define CURRENT_FIRMWARE_VERSION "1.0.0"
#define EXPECTED_TOKEN "aya_123"
#define JSON_URL "https://github.com/ayatarek7/Iot-fota/blob/main/firmware.json"

#define CBL_FLASH_WRITE_CMD 0x17
#define CBL_JUMP_TO_APP     0x20
#define CBL_GET_VER_CMD     0x10
#define ACK_BYTE            0xCD
#define NACK_BYTE           0xAB


const char* ssid = "ITI_Students";
const char* password = "ITI@$024";

HardwareSerial STM32Serial(2); // UART2

void setup() {
  Serial.begin(115200);
  STM32Serial.begin(115200, SERIAL_8N1, 16, 17); // ESP32 RX , TX 

  connectToWiFi();
  delay(1000);

  checkFirmwareUpdate();
  sendJumpToAppCommand();


}

void loop() {
  // Nothing here
}

void connectToWiFi() {
  Serial.print(" Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n Connected to WiFi.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void checkFirmwareUpdate() {
  HTTPClient http;
  http.begin(JSON_URL);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    Serial.println(" JSON received:");
    Serial.println(payload);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("Failed to parse JSON");
      return;
    }

    String newVersion = doc["version"];
    String fwUrl = doc["url"];
    String token = doc["token"];

    if (token != EXPECTED_TOKEN) {
      Serial.println("Invalid token! Update rejected.");
      return;
    }

    if (newVersion != CURRENT_FIRMWARE_VERSION) {
      Serial.printf("New Version Available: %s\n", newVersion.c_str());
      sendGetVersionCommand(); // optional version check
      delay(500);

      downloadAndSendFirmware(fwUrl);
      sendJumpToAppCommand();

    } else {
      Serial.println("Firmware is up to date.");
    }
  } else {
    Serial.print("HTTP Error: ");
    Serial.println(httpCode);
  }

  http.end();
}

void downloadAndSendFirmware(String fwUrl) {
  HTTPClient http;
  http.begin(fwUrl);
  int httpCode = http.GET();

  if (httpCode == 200) {
    WiFiClient* stream = http.getStreamPtr();
    const int chunkSize = 256;
    uint8_t buffer[chunkSize];
    uint32_t address = 0x08008000; // start of user flash (skip bootloader section)

    Serial.println("Downloading firmware and sending to STM32...");

    while (http.connected() && stream->available()) {
      int bytesRead = stream->readBytes(buffer, chunkSize);
      sendFlashWriteCommand(address, buffer, bytesRead);
      address += bytesRead;
      delay(10);
    }

    Serial.println("Firmware sent successfully.");
  } else {
    Serial.print(" Failed to download firmware: ");
    Serial.println(httpCode);
  }

  http.end();
}

void sendFlashWriteCommand(uint32_t address, uint8_t* data, uint16_t len) {
  uint8_t Address[4]={0x00,0x80,0x00,0x08};
  uint8_t packet[300];
  STM32Serial.write(Address,4);
  packet[0] = 0x01 + 4 + 2 + len + 4; // total length
  packet[1] = CBL_FLASH_WRITE_CMD;

  packet[2] = (address >> 24) & 0xFF;
  packet[3] = (address >> 16) & 0xFF;
  packet[4] = (address >> 8) & 0xFF;
  packet[5] = address & 0xFF;

  packet[6] = (len >> 8) & 0xFF;
  packet[7] = len & 0xFF;

  memcpy(&packet[8], data, len);

  uint32_t crc = calculateCRC(&packet[1], 7 + len);
  packet[8 + len] = (crc >> 24) & 0xFF;
  packet[9 + len] = (crc >> 16) & 0xFF;
  packet[10 + len] = (crc >> 8) & 0xFF;
  packet[11 + len] = crc & 0xFF;

  STM32Serial.write(packet, 12 + len);
  delay(50);

  if (STM32Serial.available() >= 1) {
    uint8_t response = STM32Serial.read();
    if (response == ACK_BYTE) {
      Serial.printf("Chunk @ 0x%08X written.\n", address);
    } else {
      Serial.printf("Write failed @ 0x%08X\n", address);
    }
  }
}

void sendGetVersionCommand() {
  uint8_t cmdPacket[6];
  uint8_t command = CBL_GET_VER_CMD;

  uint32_t crc = calculateCRC((uint8_t*)&command, 1);
  cmdPacket[0] = 0x05;
  cmdPacket[1] = CBL_GET_VER_CMD;
  cmdPacket[2] = (crc >> 24) & 0xFF;
  cmdPacket[3] = (crc >> 16) & 0xFF;
  cmdPacket[4] = (crc >> 8) & 0xFF;
  cmdPacket[5] = crc & 0xFF;

  STM32Serial.write(cmdPacket, 6);
  delay(100);

  if (STM32Serial.available() >= 2) {
    uint8_t ack[2];
    STM32Serial.readBytes(ack, 2);
    if (ack[0] == ACK_BYTE) {
      uint8_t len = ack[1];
      uint8_t data[10];
      STM32Serial.readBytes(data, len);
      Serial.printf("STM32 Version: %d.%d.%d (Vendor: %d)\n", data[1], data[2], data[3], data[0]);
    } else {
      Serial.println("NACK from STM32");
    }
  } else {
    Serial.println(" No response from STM32");
  }
}

void sendJumpToAppCommand() {
  uint8_t packet[6];
  uint8_t command = CBL_JUMP_TO_APP;
  uint32_t crc = calculateCRC((uint8_t*)&command, 1);
  packet[0] = 0x05;
  packet[1] = CBL_JUMP_TO_APP;
  packet[2] = (crc >> 24) & 0xFF;
  packet[3] = (crc >> 16) & 0xFF;
  packet[4] = (crc >> 8) & 0xFF;
  packet[5] = crc & 0xFF;

  STM32Serial.write(packet, 6);
  Serial.println(" Jump to new firmware sent.");
}

uint32_t calculateCRC(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint32_t)data[i] << 24);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80000000)
        crc = (crc << 1) ^ 0x04C11DB7;
      else
        crc <<= 1;
    }
  }
  return crc;
}


