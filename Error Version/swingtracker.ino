#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Threshold for impact detection
const float IMPACT_THRESHOLD = 12; // Adjust based on testing

// Low-pass filter smoothing factor
const float alpha = 0.2; // Start with 0.2 and adjust as needed

// Filtered acceleration variables
float accel_x_filtered = 0;
float accel_y_filtered = 0;
float accel_z_filtered = 0;

// Variables for gyroscope integration
float clubface_angle = 0; // Cumulative rotation around Z-axis
float dt = 0.05;          // Time step (50 ms)

// Rotation radius variable
float rotation_radius = 0;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 initialized!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read raw accelerometer and gyroscope data
  float accel_x = a.acceleration.x; // Centripetal acceleration (X-axis)
  float accel_y = a.acceleration.y;
  float accel_z = a.acceleration.z;
  float angular_velocity = g.gyro.z; // Z-axis angular velocity (rad/s)

  // Apply low-pass filter to accelerometer values
  accel_x_filtered = alpha * accel_x + (1 - alpha) * accel_x_filtered;
  accel_y_filtered = alpha * accel_y + (1 - alpha) * accel_y_filtered;
  accel_z_filtered = alpha * accel_z + (1 - alpha) * accel_z_filtered;

  // Integrate gyroscope Y-axis for clubface angle
  clubface_angle += g.gyro.y * dt; // Rotation in radians (convert to degrees if needed)

  // Calculate the magnitude of filtered centripetal and tangential acceleration
  float cent_accel_magnitude = accel_y;
  float tang_accel_magnitude = sqrt(pow(accel_x_filtered, 2) +
                               pow(accel_z_filtered, 2));

  // Calculate rotation radius dynamically
  if (angular_velocity != 0) {
    rotation_radius = tang_accel_magnitude / pow(angular_velocity, 2);
  }

  // Detect impact based on acceleration spike
  if (tang_accel_magnitude > IMPACT_THRESHOLD) {
    Serial.println("Impact detected!");

    // Calculate swing speed (v = r * omega)
    float swing_speed = rotation_radius * angular_velocity;

    // Convert clubface angle to degrees
    float clubface_angle_deg = clubface_angle * 180 / PI;

    // Determine whether the clubface is open, closed, or square
    String clubface_status = "Square";
    if (clubface_angle_deg > 0) {
      clubface_status = "Closed";
    } else if (clubface_angle_deg < 0) {
      clubface_status = "Open";
    }

    // Print results

    Serial.print("Swing Speed: ");
    Serial.print(swing_speed);
    Serial.println(" m/s");

    Serial.print("Clubface Angle: ");
    Serial.print(clubface_angle_deg);
    Serial.print(" degrees (");
    Serial.print(clubface_status);
    Serial.println(")");

    Serial.print("Rotation Radius: ");
    Serial.print(rotation_radius);
    Serial.println(" meters");

    // Add delay to avoid multiple detections for the same impact
    delay(500);
  }

  delay(50); // Adjust sampling rate as needed
}
