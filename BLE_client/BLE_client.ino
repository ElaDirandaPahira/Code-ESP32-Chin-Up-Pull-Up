#include <Wire.h>
#include <MPU6050_tockn.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

MPU6050 mpu6050(Wire);
BLEClient *pClient = nullptr;
BLERemoteService* pRemoteService = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

const char *SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char *CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

const int thresholdAngle = 50; // Sudut untuk pull up
const int chinUpThresholdAngle = 40; // Sudut untuk chin up
bool angle50Reached = false;  // kondisi pull up
bool angle40Reached = false;  // kondisi chin up

float angleX = 0;
unsigned long previousTime = 0;
float dt = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  BLEDevice::init("ESP32_MPU6050_Client");
  pClient = BLEDevice::createClient();
  Serial.println("Client initialized, searching for server...");
  connectToServer();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0; // Konversi ms ke detik
  previousTime = currentTime;

  mpu6050.update();
  float accelX = mpu6050.getAccX();
  float accelY = mpu6050.getAccY();
  float accelZ = mpu6050.getAccZ();
  float gyroX = mpu6050.getGyroX();

  // Menggunakan accelerometer untuk memperkirakan sudut
  float accelAngleX = atan2(accelY, accelZ) * 180 / M_PI;

  // Menggunakan gyroscope untuk memperkirakan sudut
  angleX += gyroX * dt;

  // Menggabungkan kedua sudut dengan Complementary Filter
  angleX = 0.98 * angleX + 0.02 * accelAngleX;

  // Mengukur roll dan pitch
  float roll = atan2(accelY, accelZ) * 180.0 / M_PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;

  // Menampilkan sudut roll dan pitch di Serial Monitor saat tidak mendeteksi angle 50 atau 40
  if (angleX < thresholdAngle && !angle50Reached && angleX < chinUpThresholdAngle && !angle40Reached) {
    Serial.printf("Roll: %.2f degrees, Pitch: %.2f degrees\n", roll, pitch);
  }

  // pull up condition at 50 degrees
  if (angleX >= thresholdAngle && !angle50Reached) {
    angle50Reached = true;
    notifyServer("Angle 50 detected");
  } else if (angleX < thresholdAngle && angle50Reached) {
    angle50Reached = false;
  }

  // chin up condition at 40 degrees
  if (angleX >= chinUpThresholdAngle && !angle40Reached) {
    angle40Reached = true;
    notifyServer("Angle 40 detected");
  } else if (angleX < chinUpThresholdAngle && angle40Reached) {
    angle40Reached = false;
  }

  delay(10); // Loop runs every 10ms
}

void notifyServer(String message) {
  if (pClient->isConnected() && pRemoteService != nullptr && pRemoteCharacteristic != nullptr) {
    pRemoteCharacteristic->writeValue(message.c_str(), message.length());
    Serial.println("Notification sent: " + message);
  }
}

void connectToServer() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults* pFoundDevices = pBLEScan->start(5);

  for (int i = 0; i < pFoundDevices->getCount(); i++) {
    BLEAdvertisedDevice device = pFoundDevices->getDevice(i);
    if (device.haveServiceUUID() && device.getServiceUUID().equals(BLEUUID(SERVICE_UUID))) {
      Serial.println("Server found, connecting...");
      pClient->connect(&device);
      Serial.println("Connected to server");

      pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
      if (pRemoteService != nullptr) {
        pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
        if (pRemoteCharacteristic != nullptr && pRemoteCharacteristic->canWrite()) {
          Serial.println("Found characteristic, ready to send data");
        }
      }
      break;
    }
  }
}