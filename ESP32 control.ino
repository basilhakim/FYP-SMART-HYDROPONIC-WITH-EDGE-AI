#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"
#include "HX711.h"




// ---------------------------------------------------------------------------
// WiFi & MQTT Configuration
// ---------------------------------------------------------------------------
const char* ssid        = "-";
const char* password    = "-";
const char* mqtt_server = "192.00.xxxx";




WiFiClient    espClient;
PubSubClient  client(espClient);




// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define RELAY_PUMP_NUTRIENT_A   26
#define RELAY_PUMP_NUTRIENT_B   25
#define RELAY_PUMP_PH_UP        19
#define RELAY_PUMP_PH_DOWN      23
#define RELAY_PUMP_WATER         5
#define RELAY_PUMP_DELIVERY     27
#define RELAY_LED_GROW          17




#define WATER_LEVEL_MIN_PIN     34
#define WATER_LEVEL_MAX_PIN     35
#define EC_PIN                  33
#define PH_PIN                  32
#define DHT_PIN                  4
#define DHTTYPE                 DHT22




// Load Cell Pins
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN  = 18;




// ---------------------------------------------------------------------------
// Controller & Filter Parameters
// ---------------------------------------------------------------------------
float         ecSetpoint       = 1.3f;
float         ecDeadband       = 0.30f;
unsigned long waterDoseDuration = 5000;
unsigned long mixingWaitTime    = 30000;




HX711 scale;
float calibration_factor = -143.66;
float lastStableWeight   = 0.0;
unsigned long timeNearZero = 0;
const float         ZERO_DRIFT_THRESHOLD = 2.0;
const unsigned long AUTO_TARE_DELAY      = 4000;




// ---------------------------------------------------------------------------
// State Flags & Timers
// ---------------------------------------------------------------------------
bool          isDiluting      = false;
bool          piOverrideWater = false;
unsigned long doseStartTime   = 0;
unsigned long waitStartTime   = 0;
unsigned long lastMqttPublish = 0;
const long    mqttInterval    = 1000;   // 1 Hz telemetry




// [FIX 10] Reconnect failure counter
int       mqttFailCount      = 0;
const int MQTT_MAX_FAILURES  = 10;




// [FIX 11] Water pump state for telemetry
bool waterPumpOn = false;




// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
RTC_DS3231 rtc;
DHT        dht(DHT_PIN, DHTTYPE);




float PH_CALIBRATION_VALUE = 21.14f;
float EC_K_CELL  = 1.00f;
float EC_SLOPE   = 6.2050f;
float EC_OFFSET  =  0.2336f;




// Forward declaration
void executeCommand(char id, char state);




// ===========================================================================
// [FIX 9] setup_wifi() — timeout + restart instead of infinite loop
// ===========================================================================
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);




  int attempts = 0;
  // [FIX 9] Max ~20 seconds before giving up (was: infinite loop)
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }




  if (WiFi.status() != WL_CONNECTED) {
    // [FIX 9] Restart instead of hanging — protects relay hardware
    Serial.println("\nWiFi failed to connect. Restarting ESP32...");
    delay(1000);
    ESP.restart();
  }




  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}




// ===========================================================================
// MQTT Command Callback
// ===========================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  if (length >= 2) {
    char id    = (char)payload[0];
    char state = (char)payload[1];
    executeCommand(id, state);
  }
}




// ===========================================================================
// [FIX 10] reconnect() — non-blocking single attempt.
// loop() continues running between attempts so dilution timers and
// water pump safety logic are never frozen.
// After MQTT_MAX_FAILURES consecutive failures, ESP.restart() is triggered.
// ===========================================================================
void reconnect() {
  if (client.connected()) {
    mqttFailCount = 0;   // Reset counter when already connected
    return;
  }




  Serial.print("Attempting MQTT connection... ");




  // [FIX 10] Single attempt — return immediately on failure
  if (client.connect("ESP32_Hydro_Node")) {
    Serial.println("connected.");
    client.subscribe("esp32/commands");
    mqttFailCount = 0;   // [FIX 10] Reset on success
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    mqttFailCount++;   // [FIX 10] Count consecutive failures




    // [FIX 10] After too many failures, restart to recover cleanly
    if (mqttFailCount >= MQTT_MAX_FAILURES) {
      Serial.println("MQTT broker unreachable — restarting ESP32...");
      delay(1000);
      ESP.restart();
    }
  }
  // [FIX 10] Return immediately — loop() continues regardless
}




// ===========================================================================
// setup()
// ===========================================================================
void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin();
  rtc.begin();
  analogReadResolution(12);




  setup_wifi();
  client.setServer(mqtt_server, 1884);
  client.setCallback(callback);




  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();




  pinMode(RELAY_PUMP_NUTRIENT_A,  OUTPUT);
  pinMode(RELAY_PUMP_NUTRIENT_B,  OUTPUT);
  pinMode(RELAY_PUMP_PH_UP,       OUTPUT);
  pinMode(RELAY_PUMP_PH_DOWN,     OUTPUT);
  pinMode(RELAY_PUMP_WATER,       OUTPUT);
  pinMode(RELAY_PUMP_DELIVERY,    OUTPUT);
  pinMode(RELAY_LED_GROW,         OUTPUT);




  pinMode(WATER_LEVEL_MIN_PIN, INPUT_PULLUP);
  pinMode(WATER_LEVEL_MAX_PIN, INPUT_PULLUP);




  // =========================================================================
  // [FIX 7] Active High Logic — LOW = OFF, HIGH = ON
  //         (was labelled "Active Low Logic" — corrected)
  // =========================================================================
  digitalWrite(RELAY_PUMP_NUTRIENT_A, LOW);   // OFF at boot
  digitalWrite(RELAY_PUMP_NUTRIENT_B, LOW);   // OFF at boot
  digitalWrite(RELAY_PUMP_PH_UP,      LOW);   // OFF at boot
  digitalWrite(RELAY_PUMP_PH_DOWN,    LOW);   // OFF at boot
  digitalWrite(RELAY_PUMP_WATER,      LOW);   // OFF at boot
  digitalWrite(RELAY_LED_GROW,        LOW);   // OFF at boot
  digitalWrite(RELAY_PUMP_DELIVERY,   HIGH);  // ON at boot — constant circulation
}




// ===========================================================================
// loop()
// ===========================================================================
void loop() {
  // [FIX 10] Non-blocking reconnect — if MQTT is down, continue processing
  // sensors and relay safety every iteration
  if (!client.connected()) {
    reconnect();
  }
  client.loop();




  DateTime now  = rtc.now();
  float    temp = dht.readTemperature();
  float    hum  = dht.readHumidity();
  if (isnan(temp)) temp = 25.0f;
  if (isnan(hum))  hum  = 50.0f;




  // Day/Night LED cycle (07:00–19:00)
  if (now.hour() >= 7 && now.hour() < 19)
    digitalWrite(RELAY_LED_GROW, HIGH);
  else
    digitalWrite(RELAY_LED_GROW, LOW);




  // =========================================================================
  // [FIX 6] pH Bubble Sort Filter
  // Collection loop i < 9 → i < 10 so all 10 buffer slots are filled.
  // Previously ph_buffer[9] was uninitialised garbage that corrupted the
  // middle-6 average used for pH calculation.
  // =========================================================================
  int ph_buffer[10];
  for (int i = 0; i < 10; i++) {   // [FIX 6] was: i < 9
    ph_buffer[i] = analogRead(PH_PIN);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (ph_buffer[i] > ph_buffer[j]) {
        int tV        = ph_buffer[i];
        ph_buffer[i]  = ph_buffer[j];
        ph_buffer[j]  = tV;
      }
    }
  }
  long ph_avg = 0;
  for (int i = 2; i < 8; i++) ph_avg += ph_buffer[i];
  float vPH    = (float)ph_avg * 3.3 / 4095.0 / 6;
  float phValue = -5.70 * vPH + PH_CALIBRATION_VALUE;




  // -------------------------------------------------------------------------
  // EC Processing
  // -------------------------------------------------------------------------
  long ec_acc = 0;
  for (int i = 0; i < 64; i++) {
    ec_acc += analogRead(EC_PIN);
    delayMicroseconds(1000);
  }
  float vEC     = ((float)ec_acc / 64.0f) * 3.3f / 4095.0f;
  float ecValue = 0.0f;
  if (vEC > 0.02f) {
    float ec_raw = (vEC * EC_SLOPE * EC_K_CELL) + EC_OFFSET;
    ecValue      = ec_raw / (1.0f + 0.02f * (temp - 25.0f));
    ecValue      = constrain(ecValue, 0.0f, 5.0f);
  }




  // -------------------------------------------------------------------------
  // Load Cell Auto-Tare Logic
  // -------------------------------------------------------------------------
  float rawWeight = scale.get_units(30);
  if (rawWeight > -ZERO_DRIFT_THRESHOLD && rawWeight < ZERO_DRIFT_THRESHOLD) {
    if (timeNearZero == 0)
      timeNearZero = millis();
    else if (millis() - timeNearZero > AUTO_TARE_DELAY) {
      scale.tare();
      timeNearZero   = 0;
      rawWeight      = 0.0;
      lastStableWeight = 0.0;
    }
  } else {
    timeNearZero = 0;
    if (abs(rawWeight) < ZERO_DRIFT_THRESHOLD)
      rawWeight = 0.0;
    if (abs(rawWeight - lastStableWeight) > 0.8)
      lastStableWeight = rawWeight;
  }




  // -------------------------------------------------------------------------
  // Water Level Sensors (INPUT_PULLUP: triggered state = LOW)
  // -------------------------------------------------------------------------
  bool isFull = (digitalRead(WATER_LEVEL_MAX_PIN) == LOW);
  bool isLow  = (digitalRead(WATER_LEVEL_MIN_PIN) == LOW);




  // -------------------------------------------------------------------------
  // Automatic Dilution Rule
  // -------------------------------------------------------------------------
  if (!isDiluting &&
      ecValue > (ecSetpoint + ecDeadband) &&
      (millis() - waitStartTime > mixingWaitTime)) {
    isDiluting    = true;
    doseStartTime = millis();
    digitalWrite(RELAY_PUMP_WATER, HIGH);
    waterPumpOn = true;   // [FIX 11] track water pump state
  }




  if (isDiluting) {
    if (millis() - doseStartTime > waterDoseDuration) {
      digitalWrite(RELAY_PUMP_WATER, LOW);
      isDiluting  = false;
      waterPumpOn = false;   // [FIX 11]




      // ======================================================================
      // [FIX 8] Reset piOverrideWater when dilution dose ends.
      // Previously never cleared here — if Pi sent W1 and then disconnected,
      // the isFull safety shutoff was permanently disabled for the rest of
      // the uptime.
      // ======================================================================
      piOverrideWater = false;   // [FIX 8] added
      waitStartTime   = millis();
    }
  } else {
    // Level Maintenance
    if (isLow) {
      digitalWrite(RELAY_PUMP_WATER, HIGH);
      waterPumpOn = true;   // [FIX 11]
    }
    if (isFull && !piOverrideWater) {
      digitalWrite(RELAY_PUMP_WATER, LOW);
      waterPumpOn = false;   // [FIX 11]
    }
  }




  // =========================================================================
  // 1 Hz Telemetry for Pi
  // [FIX 11] Water pump state (waterPumpOn + isDiluting) added to JSON
  //          payload so Python SCADA sees ALL water activity, including
  //          ESP32-autonomous dilution and level refills that bypass the
  //          Pi command channel entirely.
  // =========================================================================
  if (millis() - lastMqttPublish >= mqttInterval) {
    lastMqttPublish = millis();




// --- YOUR ADDED SERIAL PRINT DRILL-DOWN ---
    Serial.println("--- SYSTEM SENSOR STATE ---");
    Serial.print("Temperature: ");    Serial.print(temp, 1); Serial.println(" °C");
   Serial.print("Humidity: "); Serial.print(hum, 1); Serial.println(" %");
   Serial.print("EC Value: "); Serial.print(ecValue, 3); Serial.println(" mS/cm");
   Serial.print("pH Value: "); Serial.println(phValue, 3);
   Serial.print("Weight: "); Serial.print(lastStableWeight, 1); Serial.println(" g");
   Serial.print("Water Pump: "); Serial.println(waterPumpOn ? "ON" : "OFF");
   Serial.print("Diluting: "); Serial.println(isDiluting ? "YES" : "NO");
   Serial.print("RTC Time: ");
   Serial.print(now.hour());
   Serial.print(":");
   Serial.println(now.minute());
   Serial.print("EC Voltage: ");
   Serial.println(vEC, 4);
   Serial.println("---------------------------\n");
// ------------------------------------------








    String payload = "{";
    payload += "\"temp\":"    + String(temp,          1) + ",";
    payload += "\"hum\":"     + String(hum,           1) + ",";
    payload += "\"tds\":"     + String(ecValue,        3) + ",";
    payload += "\"ph\":"      + String(phValue,        3) + ",";
    payload += "\"weight\":"  + String(lastStableWeight, 1) + ",";
    // [FIX 11] Added water pump fields — were missing entirely
    payload += "\"water_pump\":" + String(waterPumpOn ? 1 : 0) + ",";
    payload += "\"diluting\":"   + String(isDiluting  ? 1 : 0);
    payload += "}";




    client.publish("esp32/sensors", payload.c_str());
  }
}




// ===========================================================================
// executeCommand() — handles Pi-issued pump commands
// ===========================================================================
void executeCommand(char id, char state) {
  int pin = -1;
  int s   = (state == '1') ? HIGH : LOW;   // Active high: '1'=HIGH=ON




  switch (id) {
    case 'A': pin = RELAY_PUMP_NUTRIENT_A; break;
    case 'B': pin = RELAY_PUMP_NUTRIENT_B; break;
    case 'U': pin = RELAY_PUMP_PH_UP;      break;
    case 'D': pin = RELAY_PUMP_PH_DOWN;    break;
    case 'W':
      if (!isDiluting) {
        pin             = RELAY_PUMP_WATER;
        piOverrideWater = (state == '1');
        waterPumpOn     = (state == '1');   // [FIX 11] track Pi-commanded water state
      }
      break;
  }




  if (pin != -1) digitalWrite(pin, s);
}
