#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
 
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "1234"; // Change this to reflect the pin expected by the real slave BT device

MPU6050 mpu;
Adafruit_MPU6050 mpu2;

String device_name = "ESP32-BT-Slave";
BluetoothSerial SerialBT;

int16_t ax_ref, ay_ref, az_ref, gx_ref, gy_ref, gz_ref;
float angleX = 0, angleY = 0, angleZ = 0; // Rotation angles
unsigned long previousTime = 0;

#define STATUS 26
float elapsedTime = 0;

void setup() {
  Serial.begin(9600);

  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  #ifndef USE_NAME
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
  pinMode(STATUS,  OUTPUT);

  Wire.begin();
  Serial.println("Initializing mpu26050...");
  if (!mpu2.begin()) {
    Serial.println("Could not find a valid mpu26050 sensor, check wiring!");
    while (1);
  }
  mpu.initialize();
  Serial.println("mpu26050 initialized!");
  mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu2.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("Ax: "); Serial.print(ax); Serial.print(" ");
  Serial.print("Ay: "); Serial.print(ay); Serial.print(" ");
  Serial.print("Az: "); Serial.print(az); Serial.print(" ");
  Serial.print("Gx: "); Serial.print(gx); Serial.print(" ");
  Serial.print("Gy: "); Serial.print(gy); Serial.print(" ");
  Serial.print("Gz: "); Serial.println(gz);

  calibrateMPU();
}
 
void loop() {
  if (true) {
    //SerialBT.write(Serial.read());
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (elapsedTime < 160) {
      elapsedTime = elapsedTime + 1;
      digitalWrite(STATUS, HIGH);
    } 
    if (elapsedTime == 160) {
      digitalWrite(STATUS, LOW);
    }

    int16_t ax_rel = ax - ax_ref;
    int16_t ay_rel = ay - ay_ref;
    int16_t az_rel = az - az_ref;
    int16_t gx_rel = gx - gx_ref;
    int16_t gy_rel = gy - gy_ref;
    int16_t gz_rel = gz - gz_ref;

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
    previousTime = currentTime;

    // Calculate accelerometer angles
    float accelAngleX = atan2(ay, az) * 180 / PI;
    float accelAngleY = atan2(ax, az) * 180 / PI;
    float accelAngleZ = atan2(ax, ay) * 180 / PI;

    // Integrate gyroscope data
    angleX += gx_rel * deltaTime / 131.0; // 131 is the sensitivity scale factor for the gyroscope
    angleY += gy_rel * deltaTime / 131.0;
    angleZ += gz_rel * deltaTime / 131.0;

    // Complementary filter to combine accelerometer and gyroscope data
    angleX = 0.98 * angleX + 0.02 * accelAngleX;
    angleY = 0.98 * angleY + 0.02 * accelAngleY;
    angleZ = 0.98 * angleZ + 0.02 * accelAngleZ;


    String angleData = "AngleX: " + String(angleX) + " AngleY: " + String(angleY) + " AngleZ: " + String(angleZ);
    String timeData = String(elapsedTime);


    Serial.println(angleData);
    Serial.println(timeData);


    sensors_event_t a, g, temp;
    mpu2.getEvent(&a, &g, &temp);

    float xAcceleration = a.acceleration.z;
    float yAcceleration = a.acceleration.y;
    float zAcceleration = a.acceleration.z;

    Serial.print("Acceleration X: ");
    Serial.print(xAcceleration);
    Serial.print(" Acceleration Y: ");
    Serial.print(yAcceleration);
    Serial.print(" Acceleration Z: ");
    Serial.println(zAcceleration);

    // Send angle and elapsed time data via Bluetooth
    SerialBT.println(String(angleZ) + " " + timeData + "," + String(zAcceleration));
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(10);
}

void calibrateMPU() {
  int numReadings = 500; // Increase the number of readings for better calibration
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(1); // Small delay between readings
  }

  ax_ref = ax_sum / numReadings;
  ay_ref = ay_sum / numReadings;
  az_ref = az_sum / numReadings;
  gx_ref = gx_sum / numReadings;
  gy_ref = gy_sum / numReadings;
  gz_ref = gz_sum / numReadings;
}