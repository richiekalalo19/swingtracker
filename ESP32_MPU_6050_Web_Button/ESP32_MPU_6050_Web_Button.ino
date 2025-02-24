#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

Adafruit_MPU6050 mpu;

// High-pass filter parameters
float alpha = 0.98; 

// Variables definition
float accelX_filtered = 0, accelY_filtered = 0, accelZ_filtered = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float radius = 0;
float fix_radius = 1.3; // Change according to club being used
float swing_speed = 0;
float fix_swing_speed = 0;
float cent_accel_magnitude = 0;
float angular_velocity = 0;

// Wi-Fi credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Web server and SSE
AsyncWebServer server(80);
AsyncEventSource events("/events");
JSONVar readings;

unsigned long lastOutputTime = 0;
const unsigned long outputInterval = 100; // 100 ms interval

// Boolean for Button
bool isReadingActive = false;

// HTML Page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Swing Tracker Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    .container { display: flex; flex-wrap: wrap; gap: 20px; }
    .card { border: 1px solid #ccc; border-radius: 8px; padding: 20px; width: 300px; box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1); }
    .card h3 { margin-top: 0; }
    .value { font-size: 1.5em; color: #333; }
    .button { padding: 10px 20px; font-size: 16px; margin-top: 20px; cursor: pointer; border-radius: 5px; border: none; background-color: #4CAF50; color: white; }
    .button.stop { background-color: #f44336; }
  </style>
</head>
<body>
  <h1>Swing Tracker Live Readings</h1>
  <button id="toggleButton" class="button">Start Tracking</button>
  <div class="container">
    <div class="card"><h3>Radius (m)</h3><div class="value" id="radius">0.00</div></div>
    <div class="card"><h3>Angular Velocity (rad/s)</h3><div class="value" id="angular_velocity">0.00</div></div>
    <div class="card"><h3>Centripetal Acceleration (m/s²)</h3><div class="value" id="cent_accel_magnitude">0.00</div></div>
    <div class="card"><h3>Swing Speed (mph)</h3><div class="value" id="swing_speed">0.00</div></div>
    <div class="card"><h3>Fixed Swing Speed (mph)</h3><div class="value" id="fix_swing_speed">0.00</div></div>
    <div class="card"><h3>Accel X Filtered (m/s²)</h3><div class="value" id="accelX_filtered">0.00</div></div>
    <div class="card"><h3>Accel Y Filtered (m/s²)</h3><div class="value" id="accelY_filtered">0.00</div></div>
    <div class="card"><h3>Accel Z Filtered (m/s²)</h3><div class="value" id="accelZ_filtered">0.00</div></div>
  </div>

  <script>
    const eventSource = new EventSource('/events');
    let isReading = false;
    const toggleButton = document.getElementById('toggleButton');

    toggleButton.addEventListener('click', function() {
      isReading = !isReading;
      fetch(`/toggle?state=${isReading ? 'on' : 'off'}`);
      toggleButton.textContent = isReading ? 'Stop Tracking' : 'Start Tracking';
      toggleButton.classList.toggle('stop', isReading);
    });

    eventSource.addEventListener('new_readings', function(event) {
      if (isReading) {
        const data = JSON.parse(event.data);
        document.getElementById('radius').textContent = data.radius.toFixed(2);
        document.getElementById('angular_velocity').textContent = data.angular_velocity.toFixed(2);
        document.getElementById('cent_accel_magnitude').textContent = data.cent_accel_magnitude.toFixed(2);
        document.getElementById('swing_speed').textContent = data.swing_speed.toFixed(2);
        document.getElementById('fix_swing_speed').textContent = data.fix_swing_speed.toFixed(2);
        document.getElementById('accelX_filtered').textContent = data.accelX_filtered.toFixed(2);
        document.getElementById('accelY_filtered').textContent = data.accelY_filtered.toFixed(2);
        document.getElementById('accelZ_filtered').textContent = data.accelZ_filtered.toFixed(2);
      }
    });
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  lastAccelX = a.acceleration.x;
  lastAccelY = a.acceleration.y;
  lastAccelZ = a.acceleration.z;
  accelX_filtered = a.acceleration.x;
  accelY_filtered = a.acceleration.y;
  accelZ_filtered = a.acceleration.z;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP address: http://");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("state")) {
      String state = request->getParam("state")->value();
      isReadingActive = (state == "on");
      Serial.println(isReadingActive ? "Data reading started." : "Data reading stopped.");
    }
    request->send(200, "text/plain", "OK");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->connected()) {
      Serial.println("Client connected");
    }
  });
  server.addHandler(&events);
  server.begin();
}

void sendSensorData() {
  readings["radius"] = radius;
  readings["angular_velocity"] = angular_velocity;
  readings["cent_accel_magnitude"] = cent_accel_magnitude;
  readings["swing_speed"] = swing_speed;
  readings["fix_swing_speed"] = fix_swing_speed;
  readings["accelX_filtered"] = accelX_filtered;
  readings["accelY_filtered"] = accelY_filtered;
  readings["accelZ_filtered"] = accelZ_filtered;

  String jsonString = JSON.stringify(readings);
  events.send(jsonString.c_str(), "new_readings", millis());
}

void loop() {
  if (isReadingActive) { // If button is turned on then data reading starts
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelX_filtered = alpha * (accelX_filtered + a.acceleration.x - lastAccelX);
    accelY_filtered = alpha * (accelY_filtered + a.acceleration.y - lastAccelY);
    accelZ_filtered = alpha * (accelZ_filtered + a.acceleration.z - lastAccelZ);

    lastAccelX = a.acceleration.x;
    lastAccelY = a.acceleration.y;
    lastAccelZ = a.acceleration.z;

    angular_velocity = sqrt((g.gyro.z * g.gyro.z) + (g.gyro.x * g.gyro.x));
    radius = accelY_filtered / (angular_velocity * angular_velocity);
    cent_accel_magnitude = accelY_filtered;
    swing_speed = angular_velocity * radius * 2.23694;
    fix_swing_speed = angular_velocity * fix_radius * 2.23694;

    if (millis() - lastOutputTime >= outputInterval) {
      lastOutputTime = millis();
      sendSensorData();
    }
  }
  else { // If button is off all variables are reset to 0
    swing_speed = 0;
    fix_swing_speed = 0;
    cent_accel_magnitude = 0;
    angular_velocity = 0;
    radius = 0;
    lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
    accelX_filtered = 0, accelY_filtered = 0, accelZ_filtered = 0;
  }

  delay(10);
}