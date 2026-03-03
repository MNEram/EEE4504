#include <Wire.h>
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>

#define LEFTBUTTON 19
#define RIGHTBUTTON 18
#define SPEED 10

Adafruit_MPU6050 mpu;
BleMouse bleMouse;


void setup() {
  Serial.begin(115200);
  
  // 1. Start I2C FIRST
  Wire.begin(21, 22); 
  delay(1000); // Give the hardware a full second to power up

  // 2. Try to initialize MPU BEFORE starting Bluetooth
  Serial.println("Attempting to find MPU6050...");
  if (!mpu.begin(0x68, &Wire)) { // Pass &Wire explicitly
    Serial.println("Failed to find MPU6050 chip");
    // Don't stop the whole program, let's see if we can bypass the loop
  } else {
    Serial.println("MPU6050 Found!");
  }

  // 3. Configure MPU while it's fresh
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 4. Start Bluetooth last
  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);
  bleMouse.begin();
  
  Serial.println("Setup complete!");
}

void loop() {
  if (bleMouse.isConnected()) {
    // 1. Handle Movement (This now runs every single loop without stopping)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gravityX = 0;
    float gravityZ = 9.8;
    float gyroBiasX = 0;
    float gyroBiasZ = 0;

    // Filter out tiny movements (deadzone) to prevent shaking
    const float alphaAcc = 0.85; // High value = ignores tilt better
    gravityX = (alphaAcc * gravityX) + ((1.0 - alphaAcc) * a.acceleration.x);
    gravityZ = (alphaAcc * gravityZ) + ((1.0 - alphaAcc) * a.acceleration.z);

    // 2. Filter Gyroscope (Drift/Bias)
    // This slowly "learns" what the sensor says when it's still
    const float alphaGyro = 0.85; 
    gyroBiasX = (alphaGyro * gyroBiasX) + ((1.0 - alphaGyro) * g.gyro.x);
    gyroBiasZ = (alphaGyro * gyroBiasZ) + ((1.0 - alphaGyro) * g.gyro.z);

    // 3. Subtract the Errors
    float cleanAccX = a.acceleration.x - gravityX;
    float cleanAccZ = a.acceleration.z - gravityZ;
    float cleanGyrX = g.gyro.x - gyroBiasX;
    float cleanGyrZ = g.gyro.z - gyroBiasZ;

    // 4. Combine them for the final move
    float moveX = (cleanGyrZ * -SPEED) + (cleanAccX * 2);
    float moveY = (cleanGyrX * SPEED) + (cleanAccZ * 2); 

    // 5. Smart Deadzone
    // If the movement is tiny, we force it to zero so the cursor stays locked
    if (abs(moveX) < 0.😎 moveX = 0;
    if (abs(moveY) < 0.😎 moveY = 0;

    if (moveX != 0 || moveY != 0) {
       bleMouse.move(moveX, moveY);
    }

    // 2. Handle Left Button (Continuous)
    if (!digitalRead(LEFTBUTTON)) {
      if (!bleMouse.isPressed(MOUSE_LEFT)) {
        bleMouse.press(MOUSE_LEFT); // Hold the button down
      }
    } else {
      if (bleMouse.isPressed(MOUSE_LEFT)) {
        bleMouse.release(MOUSE_LEFT); // Let it go
      }
    }

    // 3. Handle Right Button (Continuous)
    if (!digitalRead(RIGHTBUTTON)) {
      if (!bleMouse.isPressed(MOUSE_RIGHT)) {
        bleMouse.press(MOUSE_RIGHT);
      }
    } else {
      if (bleMouse.isPressed(MOUSE_RIGHT)) {
        bleMouse.release(MOUSE_RIGHT);
      }
    }
    
    // A tiny delay (10-20ms) is okay to keep the Bluetooth stack stable,
    // but never use 500ms!
    delay(10); 
  }
}
