// V2 as a ESP-NOW sender

#include "rm67162.h"
#include <TFT_eSPI.h>
#include "NotoSansBold15.h"
#include "NotoSansMonoSCB20.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "jason_h_engineering_logo_16bit.h"
#include <cmath>
#include <esp_now.h>
#include <WiFi.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

#define SDA_1 43
#define SCL_1 44

#define up 21   // top button for brightness control
#define down 0  // lower button is not used
#define led 38  // LED is not used

#define WIDTH  536
#define HEIGHT 240

//colors
// https://rgbcolorpicker.com/565
#define gray 0x2A0A
#define ash_red 0xc9ec
#define ash_green 0x2424
#define ash_blue 0x5b9a

//graph variables
int fromTop = 80; //20 was original, aligned to top
int box_offset = 3;
int fromLeft = 175;
int w = 350;
int h = 55;
int logo_size = 60;

// X, Y, Z axis lines (origin to end point)
float xAxis[2][3] = {{0, 0, 0}, {2, 0, 0}};  // X-axis (red)
float yAxis[2][3] = {{0, 0, 0}, {0, 2, 0}};  // Y-axis (green)
float zAxis[2][3] = {{0, 0, 0}, {0, 0, 2}};  // Z-axis (blue)

//scalefactor for axes projection
int scalefactor = 25;
int offset = -40;

// logo related
String logo_string_1 = "Jason H +";
String logo_string_2 = "Engineering";
String logo_string_3 = "BNO055 9-DOF IMU";

String euler_string = "Euler Angles";
String quat_string = "Quaternions";
String machine_string = "Machine State";

String sensor_state[4] = { "SYS", "GYR", "ACC", "MAG" };
String orientation[3] = { "X", "Y", "Z" };
String quats_sequence[4] = { "qw", "qx", "qy", "qz" };
byte system_status[4] = { 0, 0, 0, 0 };
float euler_angles[3] = { 0.0, 0.0, 0.0 };
float ref_euler_angles[3] = { 0.0, 0.0, 0.0 };
float relative_euler_angles[3] = { 0.0, 0.0, 0.0 };

float refYaw, refPitch, refRoll;
// float quaternion_wxyz[4] = { 0.0, 0.0, 0.0, 0.0};

//Battery
String battery;
float batteryVoltage; 
// How frequent to update battery V
long prevMillis;
int interval=500;

// For controlling amoled brightness
bool deb = 0;
bool deb2 = 0;
byte bright = 5;
byte brightness[7] = {100, 120, 140, 180, 200, 230, 254 };

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Replace with own receiver MAC address
// ESP32-1 MAC Address: C0:4E:30:90:F2:34
// ESP32-2 MAC Address: C0:4E:30:90:5B:5C
uint8_t slave1[] = {0xC0, 0x4E, 0x30, 0x90, 0xF2, 0x34};
uint8_t slave2[] = {0xC0, 0x4E, 0x30, 0x90, 0x5B, 0x5C};

struct struct_stepper { // for outgoing message
  float stepper_angle;
};

struct_stepper outgoingMessage_1 = {0.0};
struct_stepper outgoingMessage_2 = {0.0};

// Struct to store quaternions
struct Quaternion {
  float w, x, y, z;
};

// Store reference quaternion
Quaternion refQuat = { 0.0, 0.0, 0.0, 0.0};
Quaternion currQuat = { 0.0, 0.0, 0.0, 0.0}; 
Quaternion relativeQuat = { 0.0, 0.0, 0.0, 0.0}; 

// Create an array of pointers to struct members
//float* quaternion_wxyz[] = {&currQuat.w, &currQuat.x, &currQuat.y, &currQuat.z};
float* relativeQuat_wxyz[] = {&relativeQuat.w, &relativeQuat.x, &relativeQuat.y, &relativeQuat.z};

Quaternion quaternionMultiply(Quaternion q1, Quaternion q2) {
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}

Quaternion quaternionInverse(Quaternion q) {
  Quaternion inv;
  inv.w = q.w;
  inv.x = -q.x;
  inv.y = -q.y;
  inv.z = -q.z;
  return inv;
}


void setup() {
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  //pinMode(led, OUTPUT);
  sprite.createSprite(WIDTH, HEIGHT);
  sprite.setSwapBytes(1);
  rm67162_init();  // amoled lcd initialization
  lcd_setRotation(1);

  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_1, SCL_1);           
  delay(500);

  WiFi.mode(WIFI_STA);
  delay(1000);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(500);

  bno.setExtCrystalUse(true);

  delay(1000);// Through trial and error, need this delay for getQuat and getEvent to work properly, suspect may be minimum time required after bno.setExtCrystalUse(1) and bno.begin()

  zeroing_event();

  // Print relative quaternion
  // Serial.print("refQuat: w="); Serial.print(refQuat.w);
  // Serial.print(", x="); Serial.print(refQuat.x);
  // Serial.print(", y="); Serial.print(refQuat.y);
  // Serial.print(", z="); Serial.println(refQuat.z);

  delay(100);
  //Serial.println("End Zeroing Quat"); 

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  delay(100);

  esp_now_register_send_cb(OnDataSent);  // ✅ fixed registration

  delay(100);

  // Configure peer
  addPeer(slave1);
  addPeer(slave2);

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

void addPeer(uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_add_peer(&peerInfo);
  }
}

void drawSprite() {

  sprite.fillSprite(TFT_BLACK);
  //sprite.setTextDatum(0);

  //-----------------------Set Font----------------------------------------
  sprite.setTextColor(TFT_WHITE, gray);
  sprite.loadFont(NotoSansBold15);

  //-----------------------Euler Angle State----------------------------------------
  sprite.drawString(euler_string, fromLeft, fromTop - 15);
  for (int i = 0; i < 3; i++) {
    sprite.fillSmoothRoundRect(fromLeft + (i * 90), fromTop + (h*0), 85, 25, 6, ash_green, TFT_BLACK);
    sprite.drawString(orientation[i]+": "+String(relative_euler_angles[i])+"°", fromLeft + (i * 90) + box_offset, fromTop + (h*0) + 5);
  }
  //-----------------------End Euler Angle State----------------------------------------

  //---- batt state ----- 
  sprite.drawString(battery, fromLeft + (3 * 90) + 5, fromTop + 5);

  //-----------------------Start Quaternion State----------------------------------------
  sprite.drawString(quat_string, fromLeft, fromTop + (h*1) - 15);
  for (int i = 0; i < 4; i++) {
    sprite.fillSmoothRoundRect(fromLeft + (i * 90), fromTop + (h*1), 85, 25, 6, ash_red, TFT_BLACK);
    //sprite.drawString(quats_sequence[i]+": "+String(*quaternion_wxyz[i],4), fromLeft + (i * 90) + box_offset, fromTop + (h*1) + 5);
    sprite.drawString(quats_sequence[i]+": "+String(*relativeQuat_wxyz[i],4), fromLeft + (i * 90) + box_offset, fromTop + (h*1) + 5);

  }
  //-----------------------End Quaternion State----------------------------------------

  //-----------------------Start GYRO State----------------------------------------
  sprite.drawString(machine_string, fromLeft, fromTop + (h*2) - 15);  
  for (int i = 0; i < 4; i++) {
    sprite.fillSmoothRoundRect(fromLeft + (i * 90), fromTop + (h*2), 85, 25, 6, ash_blue, TFT_BLACK);
    sprite.drawString(sensor_state[i]+": "+String(system_status[i]), fromLeft + (i * 90) + box_offset, fromTop + (h*2) + 5);
  }
  //-----------------------End GYRO State----------------------------------------
  sprite.unloadFont();

  // Draw the 3D axes with arrowheads
  drawAxes(relative_euler_angles[0], relative_euler_angles[1], relative_euler_angles[2]);

  //-----------------------Start Logo and Header----------------------------------------
  sprite.pushImage(0, 0, logo_size, logo_size, (uint16_t *)gImage_true_color);
  sprite.setTextColor(TFT_WHITE, gray);
  sprite.loadFont(NotoSansMonoSCB20);
  sprite.drawString(logo_string_1, logo_size + 5, 5);
  sprite.drawString(logo_string_2, logo_size + 5, 30);
  sprite.drawLine(logo_size + 140, 5, logo_size + 140, 55, TFT_WHITE);
  sprite.drawString(logo_string_3, logo_size + 155, 5);
  sprite.unloadFont();
  //-----------------------End Logo and Header----------------------------------------

  lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t*)sprite.getPointer());

}

void readButtons(){

  // change screen brightness
  if (digitalRead(up) == 0) {
    if (deb == 0) {
      deb = 1;
      bright++;
      if (bright == 7) bright = 0;
      lcd_brightness(brightness[bright]);
    }
  } else deb = 0;

  // rezero sensor baseline readings
  if (digitalRead(down) == 0) {
    if (deb2 == 0) {
      deb2 = 1;
      zeroing_event();
    }
  } else deb2 = 0;

}

void zeroing_event(){

  // Get reference Euler angles (yaw, pitch, roll) for zeroing
  sensors_event_t event;
  bno.getEvent(&event);
  
  ref_euler_angles[0] = event.orientation.x;    // Yaw
  ref_euler_angles[1] = event.orientation.y;  // Pitch
  ref_euler_angles[2] = event.orientation.z;   // Roll

// Get reference quaternions for zeroing
  imu::Quaternion quat = bno.getQuat();
  refQuat.w = quat.w();
  refQuat.x = quat.x();
  refQuat.y = quat.y();
  refQuat.z = quat.z();

  // Invert the reference quaternion
  refQuat = quaternionInverse(refQuat);
}

void update_BNO055_Status(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  system_status[0] = system;
  system_status[1] = gyro;
  system_status[2] = accel;
  system_status[3] = mag;

  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  euler_angles[0] = event.orientation.x;
  euler_angles[1] = event.orientation.y;
  euler_angles[2] = event.orientation.z;

  outgoingMessage_1.stepper_angle = event.orientation.y;
  outgoingMessage_2.stepper_angle = event.orientation.z;
  //data.orientY = event.orientation.y;
  //data.orientZ = event.orientation.z;

  // Zeroing the angles
  for (int i = 0; i < 3; i++) {
    relative_euler_angles[i] = fmod((euler_angles[i] - ref_euler_angles[i] + 360), 360);
  }



  // float relativeYaw = fmod((euler_angles[0] - refYaw + 360), 360);
  // float relativePitch = fmod((euler_angles[1] - refPitch + 360), 360);
  // float relativeRoll = fmod((euler_angles[2] - refRoll + 360), 360);

  // // Print relative angles
  // Serial.print("Yaw: "); Serial.print(relative_euler_angles[0]);
  // Serial.print("\tPitch: "); Serial.print(relative_euler_angles[1]);
  // Serial.print("\tRoll: "); Serial.println(relative_euler_angles[2]);

  imu::Quaternion quat = bno.getQuat();

  // quaternion_wxyz[0] = quat.w();
  // quaternion_wxyz[1] = quat.x();
  // quaternion_wxyz[2] = quat.y();
  // quaternion_wxyz[3] = quat.z();
  
  // Quaternion currQuat;
  currQuat.w = quat.w();
  currQuat.x = quat.x();
  currQuat.y = quat.y();
  currQuat.z = quat.z();

  // Get relative quaternion by multiplying the current quaternion with the inverse of the reference quaternion
  //Quaternion relativeQuat = quaternionMultiply(refQuat, currQuat);
  relativeQuat = quaternionMultiply(refQuat, currQuat);

  // // Print relative quaternion
  // Serial.print("refQuat: w="); Serial.print(refQuat.w);
  // Serial.print(", x="); Serial.print(refQuat.x);
  // Serial.print(", y="); Serial.print(refQuat.y);
  // Serial.print(", z="); Serial.println(refQuat.z);

  // // Print relative quaternion
  // Serial.print("currQuat: w="); Serial.print(currQuat.w);
  // Serial.print(", x="); Serial.print(currQuat.x);
  // Serial.print(", y="); Serial.print(currQuat.y);
  // Serial.print(", z="); Serial.println(currQuat.z);

  // // Print relative quaternion
  // Serial.print("relativeQuat: w="); Serial.print(relativeQuat.w);
  // Serial.print(", x="); Serial.print(relativeQuat.x,4);
  // Serial.print(", y="); Serial.print(relativeQuat.y,4);
  // Serial.print(", z="); Serial.println(relativeQuat.z,4);

  //delay(500);

}


void drawArrowhead(int x, int y, int length, float angle, uint16_t color) {
  // Arrowhead size and angle
  float arrowAngle = radians(30);  // 30 degrees for arrowhead wings
  int x1 = x - length * cos(angle + arrowAngle);
  int y1 = y - length * sin(angle + arrowAngle);
  int x2 = x - length * cos(angle - arrowAngle);
  int y2 = y - length * sin(angle - arrowAngle);

  // Draw arrowhead as a filled triangle
  sprite.fillTriangle(x, y, x1, y1, x2, y2, color);
}

void drawAxes(float roll, float pitch, float yaw) {
  int x[2], y[2];
  int arrowSize = 10;  // Arrowhead size

  // Rotate and project the X-axis
  for (int i = 0; i < 2; i++) {
    float v[3] = {xAxis[i][0], xAxis[i][1], xAxis[i][2]};
    rotateVertex(v, roll, pitch, yaw);
    project(v, x[i], y[i], scalefactor);
  }
  sprite.drawLine(x[0]+offset, y[0], x[1]+offset, y[1], TFT_RED);  // Draw X-axis in red
  drawArrowhead(x[1]+offset, y[1], arrowSize, atan2(y[1] - y[0], x[1] - x[0]), TFT_RED);  // Add arrowhead
  sprite.setTextColor(TFT_RED);  // Set text color to red
  sprite.drawString("X", x[1]-20, y[1] - 10, 2);  // Add "X" annotation near the arrowhead


  // Rotate and project the Y-axis
  for (int i = 0; i < 2; i++) {
    float v[3] = {yAxis[i][0], yAxis[i][1], yAxis[i][2]};
    rotateVertex(v, roll, pitch, yaw);
    project(v, x[i], y[i], scalefactor);
  }
  sprite.drawLine(x[0]+offset, y[0], x[1]+offset, y[1], TFT_GREEN);  // Draw Y-axis in green
  drawArrowhead(x[1]+offset, y[1], arrowSize, atan2(y[1] - y[0], x[1] - x[0]), TFT_GREEN);  // Add arrowhead
  sprite.setTextColor(TFT_GREEN);  // Set text color to green
  sprite.drawString("Y", x[1]-20, y[1] - 10, 2);  // Add "Y" annotation near the arrowhead


  // Rotate and project the Z-axis
  for (int i = 0; i < 2; i++) {
    float v[3] = {zAxis[i][0], zAxis[i][1], zAxis[i][2]};
    rotateVertex(v, roll, pitch, yaw);
    project(v, x[i], y[i], scalefactor);
  }
  sprite.drawLine(x[0]+offset, y[0], x[1]+offset, y[1], TFT_BLUE);  // Draw Z-axis in blue
  drawArrowhead(x[1]+offset, y[1], arrowSize, atan2(y[1] - y[0], x[1] - x[0]), TFT_BLUE);  // Add arrowhead
  sprite.setTextColor(TFT_BLUE);  // Set text color to blue
  sprite.drawString("Z", x[1]-20, y[1] - 10, 2);  // Add "Z" annotation near the arrowhead
}


void rotateVertex(float *v, float roll, float pitch, float yaw) {
  float cosX = cos(radians(roll)), sinX = sin(radians(roll));
  float cosY = cos(radians(pitch)), sinY = sin(radians(pitch));
  float cosZ = cos(radians(yaw)), sinZ = sin(radians(yaw));

  float x = v[0], y = v[1], z = v[2];

  // Rotation around X-axis (pitch)
  float newY = y * cosX - z * sinX;
  float newZ = y * sinX + z * cosX;
  y = newY; z = newZ;

  // Rotation around Y-axis (yaw)
  float newX = x * cosY + z * sinY;
  z = z * cosY - x * sinY;
  x = newX;

  // Rotation around Z-axis (roll)
  newX = x * cosZ - y * sinZ;
  newY = x * sinZ + y * cosZ;
  
  v[0] = newX;
  v[1] = newY;
  v[2] = z;
}


// Projection function: projects 3D points to 2D space
void project(float *v, int &x, int &y, float scaleFactor) {
  float distance = 4.0;  // Distance from "camera"
  float factor = distance / (distance - v[2]);

  x = (int)(v[0] * factor * scaleFactor) + tft.width() / 2;
  y = (int)(v[1] * factor * scaleFactor) + tft.height() / 2;
}


// read battery voltage
void readbatt() {
  batteryVoltage  = ((analogRead(4) * 2 * 3.3 * 1000) / 4096)/1000;
  battery="BAT: "+String(batteryVoltage);
}



void loop() {

  readButtons();
  
  if (millis() - prevMillis > interval){
  prevMillis = millis();
  readbatt(); 
  }

  update_BNO055_Status();
  drawSprite();

  esp_now_send(slave1, (uint8_t *) &outgoingMessage_1, sizeof(outgoingMessage_1));
  esp_now_send(slave2, (uint8_t *) &outgoingMessage_2, sizeof(outgoingMessage_2));
  delay(50); // Try 20 Hz

}
