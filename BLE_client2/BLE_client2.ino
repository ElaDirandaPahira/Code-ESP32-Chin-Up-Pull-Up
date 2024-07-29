#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

MPU6050 mpu;
BLEClient *pClient = nullptr;
BLERemoteService* pRemoteService = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

const char *SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char *CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

bool isBent = false; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  BLEDevice::init("ESP32_MPU6050_Client");
  pClient = BLEDevice::createClient();
  Serial.println("Client initialized, searching for server...");
  connectToServer();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  float roll, pitch;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  roll = atan2(ay, az) * 180.0 / M_PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" degrees, Pitch: ");
  Serial.print(pitch);
  Serial.println(" degrees");

  if (roll >= 95 && roll <= 120 && pitch >= -12 && pitch <= -8) {
    if (isBent) {
      isBent = false;
    }
  }

  // Kirimkan data roll dan pitch ke server setelah memeriksa kondisi
  sendRollPitchData(roll, pitch);

  delay(1000); // Delay untuk mengurangi laju pembacaan
}

void connectToServer() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults* pFoundDevices = pBLEScan->start(5); // Changed to pointer

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

void sendRollPitchData(float roll, float pitch) {
  if (pClient->isConnected() && pRemoteService != nullptr && pRemoteCharacteristic != nullptr) {
    byte rollPitchData[8]; // Array untuk menyimpan byte dari dua float

    // Konversi float ke array byte
    memcpy(&rollPitchData[0], &roll, sizeof(float));
    memcpy(&rollPitchData[4], &pitch, sizeof(float));

    // Kirimkan data
    pRemoteCharacteristic->writeValue(rollPitchData, sizeof(rollPitchData));
    Serial.println("Roll and pitch data sent");
  }
}