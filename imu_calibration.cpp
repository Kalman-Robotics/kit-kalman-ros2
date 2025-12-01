#include <Arduino.h>
#include "IMU6500.h"

// IMU Configuration
const int IMU_SDA_PIN = 48;
const int IMU_SCL_PIN = 47;
const uint32_t IMU_I2C_FREQ = 400000; // 400 kHz

// Create IMU instance
IMU6500 imu;

// Conversion factors
const float ACCEL_SCALE = 16384.0;  // LSB/g for ±2g range (default)
const float GYRO_SCALE = 131.0;     // LSB/(°/s) for ±250°/s range (default)

// Calibration offsets (will be calculated during calibration)
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;

// Print interval
const unsigned long PRINT_INTERVAL = 100; // ms
unsigned long lastPrintTime = 0;
unsigned long lastGyroUpdateTime = 0;

// Euler angles (in degrees)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Complementary filter coefficient
const float ALPHA = 0.96;

// ========================================
// CALIBRATION FUNCTION
// ========================================
void calibrateIMU(int samples = 1000) {
  Serial.println("====================================");
  Serial.println("🔧 Starting IMU Calibration...");
  Serial.println("⚠️  Keep the sensor STILL and FLAT!");
  Serial.println("====================================");
  
  delay(2000); // Give user time to read
  
  // Accumulators for averaging
  long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  long accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  
  // Collect samples
  for (int i = 0; i < samples; i++) {
    if (imu.read()) {
      int16_t ax, ay, az, gx, gy, gz;
      imu.getData(ax, ay, az, gx, gy, gz);
      
      gyro_x_sum += gx;
      gyro_y_sum += gy;
      gyro_z_sum += gz;
      
      accel_x_sum += ax;
      accel_y_sum += ay;
      accel_z_sum += az;
      
      // Progress indicator
      if (i % 100 == 0) {
        Serial.print(".");
      }
    }
    delay(5); // 5ms between samples
  }
  Serial.println();
  
  // ========================================
  // Calculate gyroscope offsets (average of samples)
  // ========================================
  gyro_offset_x = (float)gyro_x_sum / samples / GYRO_SCALE;
  gyro_offset_y = (float)gyro_y_sum / samples / GYRO_SCALE;
  gyro_offset_z = (float)gyro_z_sum / samples / GYRO_SCALE;
  
  // ========================================
  // Calculate accelerometer offsets
  // ========================================
  // X and Y should average to 0g when flat
  accel_offset_x = (float)accel_x_sum / samples / ACCEL_SCALE;
  accel_offset_y = (float)accel_y_sum / samples / ACCEL_SCALE;
  
  // Z should average to 1g when flat (gravity), so offset is (measured - 1.0)
  accel_offset_z = ((float)accel_z_sum / samples / ACCEL_SCALE) - 1.0;
  
  // ========================================
  // Print calibration results
  // ========================================
  Serial.println("====================================");
  Serial.println("✅ Calibration Complete!");
  Serial.println("====================================");
  
  Serial.println("Gyroscope Offsets (°/s):");
  Serial.print("  X: "); Serial.print(gyro_offset_x, 4);
  Serial.print("  Y: "); Serial.print(gyro_offset_y, 4);
  Serial.print("  Z: "); Serial.println(gyro_offset_z, 4);
  
  Serial.println("Accelerometer Offsets (g):");
  Serial.print("  X: "); Serial.print(accel_offset_x, 4);
  Serial.print("  Y: "); Serial.print(accel_offset_y, 4);
  Serial.print("  Z: "); Serial.println(accel_offset_z, 4);
  
  Serial.println("====================================");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("====================================");
  Serial.println("MPU6500 IMU - Calibrated Demo");
  Serial.println("====================================");
  
  // Initialize IMU
  Serial.print("Initializing IMU on SDA:");
  Serial.print(IMU_SDA_PIN);
  Serial.print(", SCL:");
  Serial.print(IMU_SCL_PIN);
  Serial.print(", Freq:");
  Serial.print(IMU_I2C_FREQ);
  Serial.println(" Hz");
  
  if (!imu.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ)) {
    Serial.println("❌ Failed to initialize IMU!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✅ IMU initialized successfully!");
  Serial.println();
  
  // ========================================
  // PERFORM CALIBRATION
  // ========================================
  calibrateIMU(1000); // Use 1000 samples (takes ~5 seconds)
  
  Serial.println("Starting measurements...");
  Serial.println();
  
  delay(1000);
  lastGyroUpdateTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensor data
  if (imu.read()) {
    // Get raw data
    int16_t ax, ay, az, gx, gy, gz;
    imu.getData(ax, ay, az, gx, gy, gz);
    
    // ========================================
    // APPLY CALIBRATION (Subtract offsets)
    // ========================================
    float accel_x_g = (ax / ACCEL_SCALE) - accel_offset_x;
    float accel_y_g = (ay / ACCEL_SCALE) - accel_offset_y;
    float accel_z_g = (az / ACCEL_SCALE) - accel_offset_z;
    
    float gyro_x_dps = (gx / GYRO_SCALE) - gyro_offset_x;
    float gyro_y_dps = (gy / GYRO_SCALE) - gyro_offset_y;
    float gyro_z_dps = (gz / GYRO_SCALE) - gyro_offset_z;
    
    // Calculate time delta for gyroscope integration
    float dt = (currentTime - lastGyroUpdateTime) / 1000.0; // seconds
    lastGyroUpdateTime = currentTime;
    
    // ========================================
    // STEP 1: Calculate angles from accelerometer
    // ========================================
    float accel_roll = atan2(accel_y_g, accel_z_g) * 180.0 / PI;
    float accel_pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180.0 / PI;
    
    // ========================================
    // STEP 2: Integrate gyroscope rates
    // ========================================
    float gyro_roll_delta = gyro_x_dps * dt;
    float gyro_pitch_delta = gyro_y_dps * dt;
    float gyro_yaw_delta = gyro_z_dps * dt;
    
    // ========================================
    // STEP 3: Complementary filter (fuse accel + gyro)
    // ========================================
    roll = ALPHA * (roll + gyro_roll_delta) + (1.0 - ALPHA) * accel_roll;
    pitch = ALPHA * (pitch + gyro_pitch_delta) + (1.0 - ALPHA) * accel_pitch;
    yaw += gyro_yaw_delta;
    
    // Normalize yaw to -180 to +180 degrees
    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
    
    // ========================================
    // STEP 4: Print results at specified interval
    // ========================================
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      lastPrintTime = currentTime;
      
      // Calibrated sensor data
      Serial.println("Accelerometer (calibrated, g):");
      Serial.print("  X: "); Serial.print(accel_x_g, 3);
      Serial.print("  Y: "); Serial.print(accel_y_g, 3);
      Serial.print("  Z: "); Serial.print(accel_z_g, 3);
      Serial.println();
      
      Serial.println("Gyroscope (calibrated, °/s):");
      Serial.print("  X: "); Serial.print(gyro_x_dps, 2);
      Serial.print("  Y: "); Serial.print(gyro_y_dps, 2);
      Serial.print("  Z: "); Serial.print(gyro_z_dps, 2);
      Serial.println();
      
      // Euler angles
      Serial.println("Euler Angles (degrees):");
      Serial.print("  Roll:  "); Serial.print(roll, 2); Serial.println("°");
      Serial.print("  Pitch: "); Serial.print(pitch, 2); Serial.println("°");
      Serial.print("  Yaw:   "); Serial.print(yaw, 2); Serial.println("°");
      Serial.println("---");
    }
  } else {
    Serial.println("❌ Failed to read IMU data!");
  }
  
  delay(10);
}