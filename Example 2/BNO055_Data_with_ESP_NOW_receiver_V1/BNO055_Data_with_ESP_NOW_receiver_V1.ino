// V1 - works with Gyro_Data_Display_Euler_Quaternions_with_ESP_NOW_sender_V2

#include <esp_now.h>
#include <WiFi.h>

typedef struct {
  float orientX, orientY, orientZ;
  // float accX, accY, accZ;
  // float gyroX, gyroY, gyroZ;
  // float magX, magY, magZ;
} IMUData;

IMUData incomingData;
bool newDataReceived = false;

void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingDataBytes, int len) {
  if (len == sizeof(IMUData)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(IMUData));
    newDataReceived = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Receiver Ready");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);  // Register receive callback
}

void loop() {
  if (newDataReceived) {
    Serial.print(incomingData.orientX);
    Serial.print(",");
    Serial.print(incomingData.orientY);
    Serial.print(",");
    Serial.println(incomingData.orientZ);

    newDataReceived = false;
  }
}


