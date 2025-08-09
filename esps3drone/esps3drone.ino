/*
  ESP32 Brushed-Quad Minimal Autopilot (API-triggered)

  Hardware (from LiteWing-style schematic):
    - SoC: ESP32-WROOM
    - IMU: MPU6050 (I2C, SDA=21, SCL=22)
    - Motors: 4x brushed DC via SI2302 MOSFETs
        M1 -> GPIO18 (FL)
        M2 -> GPIO19 (FR)
        M3 -> GPIO23 (RR)
        M4 -> GPIO5  (RL)
    - Optional Altitude sensor on I2C: MS5611 or BMP280, or ToF VL53L1X

  HTTP API:
    GET /start  -> run mission: self-test -> climb to +1.0 m -> hover 10 s -> land
    GET /abort  -> immediate throttle cut; stays ABORTED
    GET /status -> {"state":"...","alt_m":x.xx,"thr":duty}

  !!! SAFETY !!!
  - Test without propellers first.
  - Tune gains and baseThrottleDuty for your airframe.
  - Add battery brownout & link-loss failsafes before real flight.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <esp32-hal-ledc.h>

// ===== Altitude sensor selection (choose ONE) =====
// #define USE_MS5611
// #define USE_VL53L1X
#define USE_BMP280

#ifdef USE_MS5611
  #include <MS5611.h>         // Rob Tillaart/MS5611
  MS5611 ms5611;
#endif
#ifdef USE_VL53L1X
  #include <SparkFun_VL53L1X.h> // SparkFun VL53L1X
  SFEVL53L1X vl53;
#endif
#ifdef USE_BMP280
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp;
#endif

#include <MPU6050.h>
MPU6050 imu;

// ===== Wi-Fi (EDIT THESE) =====
#define WIFI_SSID "RAID"
#define WIFI_PASS "8_302831_2"

// ===== Motor PWM (brushed DC via MOSFETs) =====
#define M1_PIN 18  // Front-Left
#define M2_PIN 19  // Front-Right
#define M3_PIN 23  // Rear-Right
#define M4_PIN 5   // Rear-Left
const int M1_CH = 0, M2_CH = 1, M3_CH = 2, M4_CH = 3;

#define PWM_FREQ_HZ   20000    // 20 kHz (inaudible)
#define PWM_RES_BITS  8        // 0..255 duty
#define DUTY_MIN      0
#define DUTY_MAX      255
static inline uint8_t clampDuty(int v){ return (uint8_t)max(DUTY_MIN, min(DUTY_MAX, v)); }

// ===== Flight profile & control =====
const float TARGET_ALT_M = 1.0f;          // climb to +1.0 m above baseline
const uint32_t HOVER_TIME_MS = 10000;     // hover 10s
const uint32_t LOOP_DT_MS    = 10;        // 100 Hz loop
int baseThrottleDuty = 130;               // hover-ish baseline (TUNE for your frame)
int throttleDuty     = 0;                 // current throttle duty
const int THROTTLE_SLEW_PER_TICK = 2;     // duty change limit per loop tick

// ===== PID (outputs are duty deltas) =====
struct PID;
float pidStep(struct PID& p, float err, float dt);

struct PID {
  float Kp, Ki, Kd;
  float iTerm=0, prevErr=0;
  float outMin, outMax;
  float iMin, iMax;
};

PID altPID  { 1.2f, 0.25f, 0.5f, 0, 0, -60, 60, -30, 30 };  // start safe; TUNE
PID rollPID { 0.9f, 0.05f, 0.25f,0, 0, -25, 25, -15, 15 };  // gentle leveling
PID pitchPID{ 0.9f, 0.05f, 0.25f,0, 0, -25, 25, -15, 15 };  // gentle leveling

static inline float fconstrain(float v, float a, float b) { return v<a?a:(v>b?b:v); }

// Execute one PID controller step
float pidStep(struct PID& p, float err, float dt) {
  p.iTerm = fconstrain(p.iTerm + err * p.Ki * dt, p.iMin, p.iMax);
  const float d = (err - p.prevErr) / dt;
  p.prevErr = err;
  return fconstrain(p.Kp*err + p.iTerm + p.Kd*d, p.outMin, p.outMax);
}

// ===== State machine =====
enum FlightState { IDLE, TEST_MOTORS, TAKEOFF, HOVER, LANDING, DISARMED, ABORTED };
void enterState(enum FlightState s);
volatile FlightState fsm = IDLE;
uint32_t stateStartMs = 0;

// ===== Baselines =====
float altBaseline = 0.0f;  // absolute altitude at start of mission

// ===== HTTP server =====
WebServer server(80);

// ===== Helpers =====
void writeAll(int d) {
  d = clampDuty(d);
  ledcWrite(M1_CH, d);
  ledcWrite(M2_CH, d);
  ledcWrite(M3_CH, d);
  ledcWrite(M4_CH, d);
}

void motorSelfTest() {
  // props OFF; brief spin each motor
  const int t = 80;   // ~31% duty
  const int ms = 300;
  writeAll(0); delay(200);

  ledcWrite(M1_CH, t); delay(ms); ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, t); delay(ms); ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, t); delay(ms); ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, t); delay(ms); ledcWrite(M4_CH, 0);
}

void enterState(enum FlightState s) { fsm = s; stateStartMs = millis(); }

String stateName() {
  switch (fsm) {
    case IDLE: return "IDLE";
    case TEST_MOTORS: return "TEST_MOTORS";
    case TAKEOFF: return "TAKEOFF";
    case HOVER: return "HOVER";
    case LANDING: return "LANDING";
    case DISARMED: return "DISARMED";
    case ABORTED: return "ABORTED";
  }
  return "?";
}

// ===== Altitude sensors =====
bool initAlt() {
#ifdef USE_MS5611
  if (ms5611.begin() != MS5611_OK) return false;
  ms5611.setOversampling(MS5611_OSR_4096);
  ms5611.read();
  return true;
#endif
#ifdef USE_VL53L1X
  if (vl53.begin() != 0) return false;
  vl53.setDistanceModeShort();        // indoor short
  vl53.setIntermeasurementPeriod(20); // ms
  vl53.startRanging();
  return true;
#endif
#ifdef USE_BMP280
  if (!bmp.begin(0x76)) return false;
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X8,
                  Adafruit_BMP280::SAMPLING_X8,
                  Adafruit_BMP280::FILTER_X16,
                  // Use the closest available standby interval
                  Adafruit_BMP280::STANDBY_MS_1);
  return true;
#endif
}

float readAltAbsolute() {
#ifdef USE_MS5611
  ms5611.read();
  float P = ms5611.getPressure();    // Pa
  const float P0 = 101325.0f;        // sea-level ref
  return 44330.0f * (1.0f - powf(P / P0, 0.1903f));
#endif
#ifdef USE_VL53L1X
  uint16_t mm = vl53.getDistance();
  return (float)mm / 1000.0f;
#endif
#ifdef USE_BMP280
  return bmp.readAltitude(1013.25f);
#endif
}

float readAltRelative() { return readAltAbsolute() - altBaseline; }

// ===== IMU & complementary filter =====
bool initIMU() {
  imu.initialize();
  return imu.testConnection();
}

void estimateAngles(float& rollDeg, float& pitchDeg, float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accelerometer -> angle
  const float axf = ax / 16384.0f;
  const float ayf = ay / 16384.0f;
  const float azf = az / 16384.0f;

  const float accRoll  = atan2f(ayf, azf) * 57.2958f;
  const float accPitch = atan2f(-axf, sqrtf(ayf*ayf + azf*azf)) * 57.2958f;

  // gyro (deg/s)
  const float gr = gx / 131.0f;
  const float gp = gy / 131.0f;

  static float r=0, p=0;
  r = 0.98f * (r + gr * dt) + 0.02f * accRoll;
  p = 0.98f * (p + gp * dt) + 0.02f * accPitch;

  rollDeg = r; pitchDeg = p;
}

// ===== HTTP handlers =====
void handleRoot() {
  server.send(200, "text/plain", "Endpoints: /start /abort /status");
}

void handleStatus() {
  float alt = readAltRelative();
  String js = String("{\"state\":\"") + stateName() + "\",\"alt_m\":" + String(alt, 2)
              + ",\"thr\":" + String(throttleDuty) + "}";
  server.send(200, "application/json", js);
}

void handleStart() {
  if (fsm != IDLE && fsm != DISARMED) {
    server.send(409, "application/json", "{\"error\":\"busy\"}");
    return;
  }
  fsm = TEST_MOTORS; stateStartMs = millis();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleAbort() {
  fsm = ABORTED;
  writeAll(0);
  server.send(200, "application/json", "{\"aborted\":true}");
}

// ===== Setup blocks =====
void setupMotors() {
  ledcSetup(M1_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(M2_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(M3_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(M4_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(M1_PIN, M1_CH);
  ledcAttachPin(M2_PIN, M2_CH);
  ledcAttachPin(M3_PIN, M3_CH);
  ledcAttachPin(M4_PIN, M4_CH);
  writeAll(0);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(200);
  }
}

// ===== Arduino lifecycle =====
void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 default on many ESP32 boards

  if (!initAlt())  Serial.println("[WARN] Alt sensor init failed (will read 0)");
  if (!initIMU())  Serial.println("[WARN] MPU6050 init failed");

  altBaseline = readAltAbsolute(); // define "ground" altitude now

  setupMotors();
  setupWiFi();

  server.on("/", handleRoot);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/abort", HTTP_GET, handleAbort);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.println("Ready. Call /start");
}

void loop() {
  server.handleClient();

  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < LOOP_DT_MS) return;
  const float dt = (now - last) / 1000.0f;
  last = now;

  // Attitude estimate (roll/pitch)
  float rollDeg=0, pitchDeg=0;
  estimateAngles(rollDeg, pitchDeg, dt);

  // Leveling PIDs (target is 0 deg => level)
  const float rollOut  = pidStep(rollPID,  -rollDeg,  dt);
  const float pitchOut = pidStep(pitchPID, -pitchDeg, dt);

  switch (fsm) {
    case IDLE:
      writeAll(0);
      break;

    case TEST_MOTORS:
      motorSelfTest();
      altBaseline = readAltAbsolute();     // reset baseline now
      throttleDuty = 0;
      enterState(TAKEOFF);
      break;

    case TAKEOFF: {
      const float altRel = readAltRelative();
      const float err    = TARGET_ALT_M - altRel;
      const float altOut = pidStep(altPID, err, dt);   // duty delta

      const int desired = baseThrottleDuty + (int)altOut;
      int step = desired - throttleDuty;
      step = constrain(step, -THROTTLE_SLEW_PER_TICK, THROTTLE_SLEW_PER_TICK);
      throttleDuty = clampDuty(throttleDuty + step);

      // Quad-X mix with gentle leveling
      const int m1 = clampDuty(throttleDuty - (int)pitchOut + (int)rollOut); // FL
      const int m2 = clampDuty(throttleDuty - (int)pitchOut - (int)rollOut); // FR
      const int m3 = clampDuty(throttleDuty + (int)pitchOut - (int)rollOut); // RR
      const int m4 = clampDuty(throttleDuty + (int)pitchOut + (int)rollOut); // RL
      ledcWrite(M1_CH, m1); ledcWrite(M2_CH, m2); ledcWrite(M3_CH, m3); ledcWrite(M4_CH, m4);

      if (fabsf(err) < 0.10f || (millis() - stateStartMs) > 8000) enterState(HOVER);
    } break;

    case HOVER: {
      const float altRel = readAltRelative();
      const float err    = TARGET_ALT_M - altRel;
      const float altOut = pidStep(altPID, err, dt);

      const int desired = baseThrottleDuty + (int)altOut;
      int step = desired - throttleDuty;
      step = constrain(step, -THROTTLE_SLEW_PER_TICK, THROTTLE_SLEW_PER_TICK);
      throttleDuty = clampDuty(throttleDuty + step);

      const int m1 = clampDuty(throttleDuty - (int)pitchOut + (int)rollOut);
      const int m2 = clampDuty(throttleDuty - (int)pitchOut - (int)rollOut);
      const int m3 = clampDuty(throttleDuty + (int)pitchOut - (int)rollOut);
      const int m4 = clampDuty(throttleDuty + (int)pitchOut + (int)rollOut);
      ledcWrite(M1_CH, m1); ledcWrite(M2_CH, m2); ledcWrite(M3_CH, m3); ledcWrite(M4_CH, m4);

      if ((millis() - stateStartMs) >= HOVER_TIME_MS) enterState(LANDING);
    } break;

    case LANDING: {
      throttleDuty = clampDuty(throttleDuty - 1);  // smooth ramp-down
      writeAll(throttleDuty);
      if (throttleDuty == 0) enterState(DISARMED);
    } break;

    case DISARMED:
      writeAll(0);
      if (millis() - stateStartMs > 2000) enterState(IDLE);
      break;

    case ABORTED:
      writeAll(0);  // stay cut until reset/power cycle
      break;
  }
}
