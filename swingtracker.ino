#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float rotation_radius = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
   sensors_event_t a, g, temp;
   mpu.getEvent(&a, &g, &temp);

  // Read raw accelerometer and gyroscope data
  float accel_x = a.acceleration.x; // Centripetal acceleration (X-axis)
  float accel_y = a.acceleration.y;
  float accel_z = a.acceleration.z;
  float angular_velocity = g.gyro.z; // Z-axis angular velocity (rad/s)
  // float angular_velocityX = g.gyro.x; // X-axis angular velocity (rad/s)
  // float angular_velocityY = g.gyro.y; // Y-axis angular velocity (rad/s)


  float radius = accel_y / (angular_velocity*angular_velocity);
  // Serial.print('x');
  // Serial.print(angular_velocityX);
  // Serial.println();
  // Serial.print('y');
  // Serial.print(angular_velocityY);
  // Serial.println();
  // Serial.print('z');
  // Serial.print(angular_velocity);
  // Serial.println();
  if(angular_velocity>=2){
    Serial.print("radius");
    Serial.print(radius);
    Serial.println();
  }
  // // Calculate the magnitude of filtered centripetal and tangential acceleration
  // float cent_accel_magnitude = accel_y;
  // float fix_radius = 1.3;
  // float resultant_angular_velocity = sqrt((g.gyro.z * g.gyro.z) + (g.gyro.x * g.gyro.x));

  // // Calculate swing speed (v = r * omega)
  // float swing_speed = (accel_y / resultant_angular_velocity) * 2.23694;
  // float fixed_swing_speed = resultant_angular_velocity * fix_radius * 2.23694;

  // // Print results

  // Serial.print("Angular Velocity: ");
  // Serial.print(resultant_angular_velocity);
  // Serial.println(" rad/s");

  // Serial.print("Centripetal Acceleration: ");
  // Serial.print(cent_accel_magnitude);
  // Serial.println(" m/s2");

  // Serial.print("Swing Speed: ");
  // Serial.print(swing_speed);
  // Serial.println(" mph\n\n");
  
  // Serial.print("Fixed Swing Speed: ");
  // Serial.print(fixed_swing_speed);
  // Serial.println(" mph\n\n");



  // Add delay to avoid multiple detections for the same impact
  delay(500);
  }

