#include <BleMouse.h>
#include <Adafruit_MPU6050.h>

#define SPEED 10
#define SINGLE_CLICK_TIME 1500  // 1.5 seconds for single click
#define DOUBLE_CLICK_TIME 3000  // 3 seconds for double click
#define STEADY_THRESHOLD 0.1
#define DEAD_ZONE 0.05
#define CLICK_COOLDOWN 2000  // 2 seconds between auto clicks

Adafruit_MPU6050 mpu;
BleMouse bleMouse;

unsigned long steadyStart = 0;
unsigned long lastClickTime = 0;
bool isSteady = false;
bool singleClickDone = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // ESP32 I2C pins

  bleMouse.begin();
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
}

void loop() {
  if (bleMouse.isConnected()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gyroX = g.gyro.x;
    float gyroZ = g.gyro.z;

    // Apply dead zone to avoid small drift
    float adjustedGyroX = abs(gyroX) > DEAD_ZONE ? gyroX : 0;
    float adjustedGyroZ = abs(gyroZ) > DEAD_ZONE ? gyroZ : 0;

    // Move cursor
    bleMouse.move(adjustedGyroZ * -SPEED, adjustedGyroX * -SPEED);

    // Detect steady head
    if (abs(gyroX) < STEADY_THRESHOLD && abs(gyroZ) < STEADY_THRESHOLD) {
      if (!isSteady) {
        steadyStart = millis();
        isSteady = true;
        singleClickDone = false;
      } else {
        unsigned long steadyDuration = millis() - steadyStart;

        // Single click after 1.5 seconds
        if (steadyDuration > SINGLE_CLICK_TIME && !singleClickDone && millis() - lastClickTime > CLICK_COOLDOWN) {
          Serial.println("Auto Single Left Click - Head steady for 1.5 sec");
          bleMouse.click(MOUSE_LEFT);
          singleClickDone = true;
          lastClickTime = millis();
        }

        // Double click after 3 seconds
        if (steadyDuration > DOUBLE_CLICK_TIME && millis() - lastClickTime > CLICK_COOLDOWN) {
          Serial.println("Auto Double Left Click - Head steady for 3 sec");
          bleMouse.click(MOUSE_LEFT);
          delay(100);
          bleMouse.click(MOUSE_LEFT);
          isSteady = false;  // Reset after double click
          lastClickTime = millis();
        }
      }
    } else {
      isSteady = false;
    }
  }
}
