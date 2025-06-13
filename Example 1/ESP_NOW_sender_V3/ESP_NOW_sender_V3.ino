// V1 verified okay in 1-to-1 ESP-NOW communication, can receive MPU-6050 data 
// V2 reconfigure to send stepper angle to slave boards, verified working with ESP_NOW_receiver_V2
// V3 clean up based on working V2 

#include <WiFi.h>
#include <esp_now.h>
#include <iostream>
#include <cmath>

struct struct_stepper { // for outgoing message
  float stepper_angle;
};

struct struct_message { // for incoming message
  int id;
  float angle_X;
  float angle_Y;
};

// ESP32-1 MAC Address: C0:4E:30:90:F2:34
// ESP32-2 MAC Address: C0:4E:30:90:5B:5C
uint8_t slave1[] = {0xC0, 0x4E, 0x30, 0x90, 0xF2, 0x34};
uint8_t slave2[] = {0xC0, 0x4E, 0x30, 0x90, 0x5B, 0x5C};

struct_stepper outgoingMessage = {0.0};
struct_message incomingMessage;

float normalizeAngle(float angle) {
    float result = std::fmod(angle, 360.0);
    if (result < 0.0)
        result += 360.0;
    return result;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print("📤 Sent to: ");
  Serial.print(macStr);
  Serial.print(" | Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
           recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5]);

  Serial.print("Data from: ");
  Serial.println(macStr);

  // Check length to avoid buffer overrun
  if (len == sizeof(incomingMessage)) {
    memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
    
    // Process the data
    Serial.print("✅ Incoming ID: ");
    Serial.println(incomingMessage.id);
    Serial.println("Value: ");
    Serial.println(incomingMessage.angle_X);
    Serial.println(incomingMessage.angle_Y);

    // You can now use incomingMessage.id or value to trigger logic
  } else {
    Serial.print("❌ Unexpected data length: ");
    Serial.println(len);
  }
}

void addPeer(uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_add_peer(&peerInfo);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  addPeer(slave1);
  addPeer(slave2);
}

void loop() {
  outgoingMessage.stepper_angle = normalizeAngle(outgoingMessage.stepper_angle + 30.0);
  esp_now_send(slave1, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  delay(600); // Give time for callback
  outgoingMessage.stepper_angle = normalizeAngle(outgoingMessage.stepper_angle + 30.0);
  esp_now_send(slave2, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  delay(800); // Give time for slaves to respond

}
