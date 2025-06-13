// V1 verified okay in 1-to-1 ESP-NOW communication, can receive info and send out MPU-6050 data 
// V2 reconfigure to receive stepper angle from master, and drive the stepper to that angle, , verified working with ESP_NOW_sender_V2 
// V3 clean up based on working V2 

#include <WiFi.h>
#include <esp_now.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <AccelStepper.h>

// Define pin connections
#define stepper_1_stepPin 6
#define stepper_1_dirPin 7

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 1

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;  // milliseconds

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL};

const int stepsPerRev = 200;  // Full steps per revolution
const int microstepping = 8;  // 1/8 microstepping

MPU6050 mpu(Wire);
unsigned long timer = 0;

struct struct_stepper { // for outgoing message
  float stepper_angle;
};

struct struct_message { // for incoming message
  int id;
  float angle_X;
  float angle_Y;
};

struct_stepper incomingMessage;
struct_message outgoingMessage;

// ESP32 S3 devkit (master) MAC Address: 30:ED:A0:BB:74:38
uint8_t master[] = {0x30, 0xED, 0xA0, 0xBB, 0x74, 0x38}; // <-- Replace with Master's MAC

void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
           recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5]);
  
  Serial.print("📥 Received data from: ");
  Serial.println(macStr);

  // Deserialize your struct
  struct_message msg;
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  Serial.println(incomingMessage.stepper_angle);

  long targetSteps = incomingMessage.stepper_angle * (stepsPerRev * microstepping) / 360.0;
  steppers[0]->moveTo(targetSteps); // Set target position

}

void setup() {

  // Create stepper instances
  steppers[0] = new AccelStepper(motorInterfaceType, stepper_1_stepPin, stepper_1_dirPin);

  if (steppers[0]) {
      steppers[0]->setMaxSpeed(8000); //steps per second
      steppers[0]->setAcceleration(4000.0); //desired acceleration in steps per second per second
  }

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Wire.begin(1, 0); // (GPIO1 - SDA, GPIO0 - SCL)
  delay(1000);

  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
   
  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, master, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(master)) {
    esp_now_add_peer(&peerInfo);
  }
}

void loop() {
  
  // Passive receive/respond mode
  mpu.update();

	// Move the motor one step
	steppers[0]->run();

  // ESP-NOW send every 1000ms
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;

    // Prepare and send response
    outgoingMessage.id = 2; // id according to specific slave device
    outgoingMessage.angle_X = mpu.getAngleX();
    outgoingMessage.angle_Y = mpu.getAngleY();

    esp_err_t result = esp_now_send(master, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));

    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
  }


}
