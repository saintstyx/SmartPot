#define BLYNK_TEMPLATE_NAME "Smart Pot"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"
#define BLYNK_TEMPLATE_ID "YOUR_BLYNK_TEMPLATE_ID"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Preferences.h>
#include <time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// ============================================================
// WIFI
// ============================================================
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// ============================================================
// OPERATING MODE
// ============================================================
// true  = always awake/online; best for presentation and testing
// false = autonomous cycle + Deep Sleep
const bool PRESENTATION_MODE = false;

const uint32_t LIVE_UPDATE_INTERVAL_MS = 2000;
const uint32_t WARNING_CHECK_INTERVAL_MS = 10000;
const uint32_t SCHEDULE_CHECK_INTERVAL_MS = 15000;

// Presentation mode may re-check light more frequently for demonstration.
const uint32_t LIGHT_RECHECK_INTERVAL_MS = 30UL * 60UL * 1000UL;

// Autonomous mode does NOT continuously chase the light.
// It evaluates the light orientation at most once every 12 hours.
// A 12-hour evaluation does not imply movement: the motor runs only
// when the light is sufficient and the selected orientation is not already acceptable.
const uint64_t AUTONOMOUS_LIGHT_INTERVAL_SECONDS = 12ULL * 60ULL * 60ULL;

// If no Watering Time is configured, autonomous mode falls back
// to one automatic moisture check every 12 hours.
const uint64_t FALLBACK_AUTO_INTERVAL_SECONDS = 12ULL * 60ULL * 60ULL;
const uint32_t FALLBACK_AUTO_INTERVAL_MS = 12UL * 60UL * 60UL * 1000UL;

// ============================================================
// GPIO
// ============================================================
const uint8_t LED_PIN   = 23;
const uint8_t PUMP_PIN  = 25;
const uint8_t MOTOR_PIN = 26;
const uint8_t LDR_RIGHT_PIN = 32;
const uint8_t FLOAT_PIN = 33;
const uint8_t SOIL_PIN  = 34;
const uint8_t LDR_LEFT_PIN  = 35;
const uint8_t BAT_PIN   = 36;

// ============================================================
// SOIL MOISTURE
// ============================================================
// Current calibration values. Recalibrate in the final substrate.
const int SOIL_DRY_CAL = 2266; // ~0 % display reference
const int SOIL_WET_CAL = 1158; // ~100 % display reference

// Target Moisture comes from V4. Watering starts only after the
// moisture falls this many percentage points below the target.
// Example: target 60 % -> new watering sequence starts at <=30 %.
const uint8_t MOISTURE_HYSTERESIS_PERCENT = 30;
const uint8_t DEFAULT_TARGET_MOISTURE = 60;

// Critical-dry notification thresholds (with hysteresis).
const uint8_t CRITICALLY_DRY_PERCENT = 10;
const uint8_t DRY_ALERT_RESET_PERCENT = 20;

// ============================================================
// WATERING
// ============================================================
const uint32_t PUMP_DURATION_MS = 2000;
const uint32_t SOAK_TIME_MS     = 30000;
const uint8_t MAX_WATERING_CYCLES = 5;

// At most ONE additional five-minute retry after a local sequence.
const uint8_t MAX_WATERING_RETRIES = 1;
const uint64_t RETRY_SLEEP_SECONDS = 5ULL * 60ULL;

// ============================================================
// LIGHT TRACKING
// ============================================================
// LDR topology: 3V3 -> LDR -> ADC node -> 1k -> GND
// More light => higher ADC value.
const int LDR_DEADBAND = 150;
const int LDR_MIN_LIGHT = 250;

// Optional per-channel calibration offsets.
// The control algorithm and Blynk display will both use
// the corrected readings.
const int LDR_LEFT_CAL_OFFSET  = -193;
const int LDR_RIGHT_CAL_OFFSET = 0;

// CENTER mode starts moving only after the imbalance is confirmed in
// several consecutive readings with the same sign. This rejects brief
// shadows/noise without changing the normal closed-loop search.
const uint8_t LDR_CENTER_START_CONFIRM_COUNT = 3;
const uint32_t LDR_CONFIRM_INTERVAL_MS = 80;

// A short single-direction movement. The algorithm uses feedback
// after every pulse, not an assumed angular position.
const uint32_t MOTOR_PULSE_MS = 1000;   
const uint32_t MOTOR_SETTLE_MS = 500;   

// Safety limits only; they are NOT used to estimate position.
const uint16_t MAX_CENTER_SEARCH_PULSES = 80;
const uint16_t MAX_PEAK_SEARCH_PULSES   = 120;

// Noise / peak-recognition margins for LEFT/RIGHT peak seeking.
const int LDR_PEAK_DROP = 60;
const int LDR_PEAK_RETURN_TOLERANCE = 70;
const uint8_t LDR_DECLINE_CONFIRM_COUNT = 2;

// ============================================================
// BATTERY
// ============================================================
const float BATTERY_DIVIDER_RATIO = 2.0f; // 100k / 100k
// After the multimeter comparison set:
// BATTERY_CAL_FACTOR = U_multimeter / U_ESP32_uncalibrated
const float BATTERY_CAL_FACTOR = 0.99246f;

const float BATTERY_LOW_V      = 3.45f;
const float BATTERY_CRITICAL_V = 3.25f;
const float BATTERY_RESET_V    = 3.60f;

// ============================================================
// TIME / SLEEP
// ============================================================
const uint64_t US_PER_SECOND = 1000000ULL;
const uint32_t SCHEDULE_GRACE_SECONDS = 5UL * 60UL;
const time_t VALID_EPOCH_MIN = 1700000000; // sanity check

// ============================================================
// GLOBAL STATE
// ============================================================
Preferences preferences;

bool cloudConnected = false;
bool autoWateringEnabled = true;
bool lightTrackingEnabled = true;
// V13: when ON in autonomous mode, Deep Sleep is suppressed and the
// ESP32 stays online for manual control. It does not auto-wake a sleeping
// ESP32; the state is synchronized at the next timer/EXT0 wake.
bool manualStayAwakeEnabled = false;
uint8_t targetMoisturePercent = DEFAULT_TARGET_MOISTURE;

// 0=CENTER, 1=LEFT, 2=RIGHT
uint8_t lightMode = 0;

// Blynk Time Input (daily schedule)
bool scheduleConfigured = false;
int32_t wateringTimeSec = -1;      // seconds from local midnight
int32_t timezoneOffsetSec = 0;     // offset from UTC, supplied by V10
String scheduleTimezone = "UTC";

// Command flags: BLYNK_WRITE callbacks remain short/non-blocking.
volatile bool manualWateringRequested = false;
bool manualRotationActive = false;
volatile bool lightTrackingRequested = false;

// Prevent repeated Blynk notifications if Manual Watering is pressed
// several times while the reservoir is still empty.
const uint32_t MANUAL_EMPTY_REMINDER_COOLDOWN_MS = 60UL * 1000UL;
uint32_t lastManualEmptyReminderMs = 0;

uint32_t lastLiveUpdateMs = 0;
uint32_t lastWarningCheckMs = 0;
uint32_t lastScheduleCheckMs = 0;
uint32_t lastLightRecheckMs = 0;
uint32_t lastFallbackAutoCheckMs = 0;
uint32_t lastReconnectAttemptMs = 0;

// UI state caches: avoid flooding Blynk with property changes.
int8_t lastSoilColorZone = -1;
int8_t lastBatteryColorZone = -1;
int8_t lastWaterColorState = -1;

// ============================================================
// RTC STATE (survives Deep Sleep)
// ============================================================
RTC_DATA_ATTR bool tankAlertSent = false;
// Remembers that an empty reservoir was observed, including across
// Deep Sleep, so a single refill confirmation can be sent afterwards.
RTC_DATA_ATTR bool tankWasEmpty = false;
RTC_DATA_ATTR bool batteryAlertSent = false;
RTC_DATA_ATTR bool drySoilAlertSent = false;
RTC_DATA_ATTR bool wateringRetryPending = false;
RTC_DATA_ATTR uint8_t wateringRetryCount = 0;

// Remaining time until the next autonomous light evaluation.
// 0 means: evaluate light on the next normal autonomous cycle.
// This value survives Deep Sleep. It is reset to 12 h after every
// light evaluation, regardless of whether the motor actually moves.
RTC_DATA_ATTR uint64_t autonomousLightRemainingSeconds = 0;

// ============================================================
// ENUMS
// ============================================================
enum WateringResult
{
  WATERING_NOT_NEEDED,
  WATERING_TARGET_REACHED,
  WATERING_INCOMPLETE,
  WATERING_ABORTED
};

enum LightMode
{
  LIGHT_CENTER = 0,
  LIGHT_LEFT   = 1,
  LIGHT_RIGHT  = 2
};

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void serviceDelay(uint32_t durationMs);
void sendBlynkData();
void processWarnings();
bool connectToBlynk();

int readSoil();
int readLDRRaw(uint8_t pin);
int readLDR(uint8_t pin);
float readBatteryVoltageRaw();
float readBatteryVoltage();
int soilPercent(int raw);
int lightPercent(int raw);
int batteryPercent(float volts);
bool tankHasWater();

bool runPump(uint32_t durationMs);
bool runMotorPulse(uint32_t durationMs);
WateringResult automaticWatering(bool continuation);
void automaticLightTracking();
void clearWateringRetryState();

bool timeIsValid();
bool scheduledWateringDueNow();
uint64_t secondsUntilNextScheduledWatering();
void handleScheduledWatering();

void sleepFor(uint64_t seconds);
void sleepUntilTankRefilled();
void runManualStayAwakeMode();
bool manualControlsAllowed();
void stopManualRotation(const char* reason);
bool centerImbalanceConfirmed();
bool autonomousLightOrientationNeeded();
uint64_t secondsUntilNextAutonomousWake();

// ============================================================
// ADC HELPERS
// ============================================================
int readAnalogAverage(uint8_t pin, uint8_t samples = 16)
{
  uint32_t total = 0;
  for (uint8_t i = 0; i < samples; i++)
  {
    total += analogRead(pin);
    delay(2);
  }
  return (int)(total / samples);
}

int readSoil()
{
  return readAnalogAverage(SOIL_PIN, 16);
}

int readLDRRaw(uint8_t pin)
{
  return readAnalogAverage(pin, 12);
}

int readLDR(uint8_t pin)
{
  int raw = readLDRRaw(pin);
  int offset = 0;

  if (pin == LDR_LEFT_PIN) offset = LDR_LEFT_CAL_OFFSET;
  else if (pin == LDR_RIGHT_PIN) offset = LDR_RIGHT_CAL_OFFSET;

  return constrain(raw + offset, 0, 4095);
}

// ============================================================
// SENSOR CONVERSIONS
// ============================================================
int soilPercent(int raw)
{
  long value = map(raw, SOIL_DRY_CAL, SOIL_WET_CAL, 0, 100);
  return constrain((int)value, 0, 100);
}

int lightPercent(int raw)
{
  long value = map(raw, 0, 4095, 0, 100);
  return constrain((int)value, 0, 100);
}

// Approximate state-of-charge from open-circuit-ish Li-ion voltage.
// This is intentionally presented as an estimate, not a coulomb count.
int batteryPercent(float v)
{
  if (v >= 4.20f) return 100;
  if (v >= 4.10f) return 90 + (int)((v - 4.10f) / 0.10f * 10.0f);
  if (v >= 4.00f) return 80 + (int)((v - 4.00f) / 0.10f * 10.0f);
  if (v >= 3.90f) return 70 + (int)((v - 3.90f) / 0.10f * 10.0f);
  if (v >= 3.80f) return 55 + (int)((v - 3.80f) / 0.10f * 15.0f);
  if (v >= 3.70f) return 40 + (int)((v - 3.70f) / 0.10f * 15.0f);
  if (v >= 3.60f) return 25 + (int)((v - 3.60f) / 0.10f * 15.0f);
  if (v >= 3.50f) return 12 + (int)((v - 3.50f) / 0.10f * 13.0f);
  if (v >= 3.40f) return 5  + (int)((v - 3.40f) / 0.10f * 7.0f);
  if (v >= 3.30f) return      (int)((v - 3.30f) / 0.10f * 5.0f);
  return 0;
}

bool tankHasWater()
{
  // Verified final logic: LOW=water present, HIGH=empty.
  return digitalRead(FLOAT_PIN) == LOW;
}

float readBatteryVoltageRaw()
{
  const uint8_t samples = 24;
  uint32_t totalMilliVolts = 0;

  for (uint8_t i = 0; i < samples; i++)
  {
    totalMilliVolts += analogReadMilliVolts(BAT_PIN);
    delay(3);
  }

  float adcVoltage = (totalMilliVolts / (float)samples) / 1000.0f;
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

float readBatteryVoltage()
{
  return readBatteryVoltageRaw() * BATTERY_CAL_FACTOR;
}

// ============================================================
// BLYNK-SAFE DELAY
// ============================================================
void serviceDelay(uint32_t durationMs)
{
  uint32_t start = millis();
  while (millis() - start < durationMs)
  {
    if (cloudConnected && Blynk.connected())
    {
      Blynk.run();
    }
    delay(10);
  }
}

// ============================================================
// OUTPUT SAFETY / STATUS
// ============================================================
void setPumpState(bool on)
{
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V5, on ? 1 : 0);
  }
}

void setMotorStatus(bool on)
{
  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V6, on ? 1 : 0);
  }
}

void allOutputsOff()
{
  manualRotationActive = false;
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V5, 0);
    Blynk.virtualWrite(V6, 0);
  }
}

// ============================================================
// PUMP
// ============================================================
bool runPump(uint32_t durationMs)
{
  if (!tankHasWater())
  {
    Serial.println("PUMP BLOCKED: reservoir empty");
    return false;
  }

  if (readBatteryVoltage() <= BATTERY_CRITICAL_V)
  {
    Serial.println("PUMP BLOCKED: battery critical");
    return false;
  }

  Serial.println("PUMP ON");
  setPumpState(true);

  uint32_t start = millis();
  while (millis() - start < durationMs)
  {
    if (!tankHasWater())
    {
      setPumpState(false);
      Serial.println("PUMP STOPPED: reservoir became empty");
      processWarnings();
      return false;
    }

    if (cloudConnected && Blynk.connected()) Blynk.run();
    delay(20);
  }

  setPumpState(false);
  Serial.println("PUMP OFF");
  return true;
}

// ============================================================
// MOTOR
// ============================================================
bool runMotorPulse(uint32_t durationMs)
{
  if (readBatteryVoltage() <= BATTERY_CRITICAL_V)
  {
    Serial.println("MOTOR BLOCKED: battery critical");
    return false;
  }

  setMotorStatus(true);
  digitalWrite(MOTOR_PIN, HIGH);
  serviceDelay(durationMs);
  digitalWrite(MOTOR_PIN, LOW);
  setMotorStatus(false);

  return true;
}

// Pulse used inside a long light-search sequence. V6 remains ON for
// the whole sequence, instead of sending ON/OFF for every tiny step.
bool motorSearchPulse()
{
  if (readBatteryVoltage() <= BATTERY_CRITICAL_V)
  {
    digitalWrite(MOTOR_PIN, LOW);
    return false;
  }

  digitalWrite(MOTOR_PIN, HIGH);
  serviceDelay(MOTOR_PULSE_MS);
  digitalWrite(MOTOR_PIN, LOW);
  serviceDelay(MOTOR_SETTLE_MS);
  return true;
}

// ============================================================
// LOW-BATTERY LED
// ============================================================
void lowBatteryBlink()
{
  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    serviceDelay(120);
    digitalWrite(LED_PIN, LOW);
    serviceDelay(180);
  }
}

// ============================================================
// WATERING
// ============================================================
void clearWateringRetryState()
{
  wateringRetryPending = false;
  wateringRetryCount = 0;
}

WateringResult automaticWatering(bool continuation)
{
  if (!autoWateringEnabled)
  {
    Serial.println("Automatic watering disabled.");
    clearWateringRetryState();
    return WATERING_NOT_NEEDED;
  }

  if (!tankHasWater())
  {
    Serial.println("Automatic watering aborted: reservoir empty.");
    return WATERING_ABORTED;
  }

  int raw = readSoil();
  int moisture = soilPercent(raw);
  int startThreshold = max(0, (int)targetMoisturePercent - (int)MOISTURE_HYSTERESIS_PERCENT);

  Serial.print("Soil moisture: ");
  Serial.print(moisture);
  Serial.print(" %, target: ");
  Serial.print(targetMoisturePercent);
  Serial.print(" %, start threshold: ");
  Serial.print(startThreshold);
  Serial.println(" %");

  if (moisture >= targetMoisturePercent)
  {
    Serial.println("Target moisture already reached.");
    clearWateringRetryState();
    return WATERING_TARGET_REACHED;
  }

  if (!continuation && moisture > startThreshold)
  {
    Serial.println("Moisture inside hysteresis band - no new watering cycle.");
    clearWateringRetryState();
    return WATERING_NOT_NEEDED;
  }

  if (continuation)
  {
    Serial.println("Continuing previous bounded watering sequence.");
  }
  else
  {
    Serial.println("Dry soil detected - starting automatic watering.");
  }

  for (uint8_t cycle = 1; cycle <= MAX_WATERING_CYCLES; cycle++)
  {
    if (!tankHasWater()) return WATERING_ABORTED;

    Serial.print("Watering cycle ");
    Serial.print(cycle);
    Serial.print("/");
    Serial.println(MAX_WATERING_CYCLES);

    if (!runPump(PUMP_DURATION_MS)) return WATERING_ABORTED;

    Serial.println("Soaking...");
    serviceDelay(SOAK_TIME_MS);

    raw = readSoil();
    moisture = soilPercent(raw);

    Serial.print("Moisture after cycle: ");
    Serial.print(moisture);
    Serial.println(" %");

    sendBlynkData();

    if (moisture >= targetMoisturePercent)
    {
      Serial.println("Target moisture reached.");
      clearWateringRetryState();
      return WATERING_TARGET_REACHED;
    }
  }

  Serial.println("Local watering safety limit reached before target.");
  return WATERING_INCOMPLETE;
}

// ============================================================
// LIGHT TRACKING - CENTER MODE
// ============================================================
bool centerImbalanceConfirmed()
{
  int expectedSign = 0;

  for (uint8_t i = 0; i < LDR_CENTER_START_CONFIRM_COUNT; i++)
  {
    int left = readLDR(LDR_LEFT_PIN);
    int right = readLDR(LDR_RIGHT_PIN);
    int difference = left - right;

    Serial.print("CENTER CONFIRM ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(LDR_CENTER_START_CONFIRM_COUNT);
    Serial.print(" | L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.print(right);
    Serial.print(" d=");
    Serial.println(difference);

    if (max(left, right) < LDR_MIN_LIGHT)
    {
      Serial.println("Too dark for reliable orientation.");
      return false;
    }

    if (abs(difference) <= LDR_DEADBAND)
    {
      Serial.println("Light already inside CENTER deadband.");
      return false;
    }

    int currentSign = (difference > 0) ? 1 : -1;
    if (i == 0) expectedSign = currentSign;
    else if (currentSign != expectedSign)
    {
      Serial.println("CENTER imbalance changed sign - treating as transient.");
      return false;
    }

    if (i + 1 < LDR_CENTER_START_CONFIRM_COUNT)
      serviceDelay(LDR_CONFIRM_INTERVAL_MS);
  }

  return true;
}

void seekLightBalance()
{
  Serial.println("LIGHT MODE: CENTER");

  if (!centerImbalanceConfirmed())
  {
    digitalWrite(MOTOR_PIN, LOW);
    setMotorStatus(false);
    return;
  }

  setMotorStatus(true);

  for (uint16_t step = 0; step < MAX_CENTER_SEARCH_PULSES; step++)
  {
    if (!lightTrackingEnabled)
    {
      Serial.println("Light tracking switched OFF - stopping.");
      break;
    }

    int left = readLDR(LDR_LEFT_PIN);
    int right = readLDR(LDR_RIGHT_PIN);
    int difference = left - right;

    Serial.print("CENTER | L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.print(right);
    Serial.print(" d=");
    Serial.println(difference);

    if (max(left, right) < LDR_MIN_LIGHT)
    {
      Serial.println("Too dark for reliable orientation.");
      break;
    }

    if (abs(difference) <= LDR_DEADBAND)
    {
      Serial.println("Light balanced - CENTER position accepted.");
      break;
    }

    if (!motorSearchPulse())
    {
      Serial.println("CENTER search stopped: motor safety condition.");
      break;
    }
  }

  digitalWrite(MOTOR_PIN, LOW);
  setMotorStatus(false);
}

// ============================================================
// LIGHT TRACKING - LEFT / RIGHT PEAK MODE
// ============================================================
// With a one-direction motor we cannot simply step backwards after
// passing the maximum. Instead the sensor itself closes the loop:
// 1) rotate in small pulses while remembering the best value;
// 2) detect that the best point has been passed (confirmed drop);
// 3) continue in the same direction around the mechanism;
// 4) stop when the selected LDR returns close to the recorded peak.
// No absolute angle sensor and no assumed full-rotation time are used.
void seekSelectedSidePeak(LightMode mode)
{
  const uint8_t selectedPin =
      (mode == LIGHT_LEFT) ? LDR_LEFT_PIN : LDR_RIGHT_PIN;

  Serial.print("LIGHT MODE: ");
  Serial.println(mode == LIGHT_LEFT ? "LEFT" : "RIGHT");

  int bestValue = readLDR(selectedPin);
  int currentValue = bestValue;
  uint8_t declineCount = 0;
  bool peakPassed = false;

  if (bestValue < LDR_MIN_LIGHT)
  {
    Serial.println("Too dark for reliable peak search.");
    return;
  }

  setMotorStatus(true);

  for (uint16_t step = 1; step <= MAX_PEAK_SEARCH_PULSES; step++)
  {
    if (!lightTrackingEnabled)
    {
      Serial.println("Light tracking switched OFF - stopping.");
      break;
    }

    if (!motorSearchPulse())
    {
      Serial.println("Peak search stopped: motor safety condition.");
      break;
    }

    currentValue = readLDR(selectedPin);

    Serial.print(mode == LIGHT_LEFT ? "LEFT" : "RIGHT");
    Serial.print(" PEAK | step=");
    Serial.print(step);
    Serial.print(" value=");
    Serial.print(currentValue);
    Serial.print(" best=");
    Serial.println(bestValue);

    if (!peakPassed)
    {
      if (currentValue > bestValue)
      {
        bestValue = currentValue;
        declineCount = 0;
      }
      else if (currentValue <= bestValue - LDR_PEAK_DROP)
      {
        declineCount++;
        if (declineCount >= LDR_DECLINE_CONFIRM_COUNT)
        {
          peakPassed = true;
          Serial.println("Peak passed; continuing until the selected LDR returns near its best value.");
        }
      }
      else
      {
        // Small variation/noise does not confirm a decline.
        declineCount = 0;
      }
    }
    else
    {
      // We already passed the peak once. Continue in the same
      // direction and stop on the next approach to that peak.
      if (currentValue >= bestValue - LDR_PEAK_RETURN_TOLERANCE)
      {
        Serial.println("Selected-side best-light position reached.");
        break;
      }
    }
  }

  digitalWrite(MOTOR_PIN, LOW);
  setMotorStatus(false);
}

// Autonomous pre-check: decide whether a mechanical correction is actually
// justified. This avoids rotating the pot merely because 12 hours elapsed.
// Three consecutive observations are required. If any observation shows
// insufficient light or an already acceptable orientation, no movement is made.
bool autonomousLightOrientationNeeded()
{
  int expectedCenterSign = 0;

  for (uint8_t i = 0; i < LDR_CENTER_START_CONFIRM_COUNT; i++)
  {
    int left = readLDR(LDR_LEFT_PIN);
    int right = readLDR(LDR_RIGHT_PIN);
    int difference = left - right;

    Serial.print("AUTO LIGHT CHECK ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(LDR_CENTER_START_CONFIRM_COUNT);
    Serial.print(" | L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.print(right);
    Serial.print(" d=");
    Serial.println(difference);

    if (max(left, right) < LDR_MIN_LIGHT)
    {
      Serial.println("Autonomous light check: too dark -> no movement.");
      return false;
    }

    if (lightMode == LIGHT_CENTER)
    {
      if (abs(difference) <= LDR_DEADBAND)
      {
        Serial.println("Autonomous light check: CENTER already acceptable -> no movement.");
        return false;
      }

      int currentSign = (difference > 0) ? 1 : -1;
      if (i == 0) expectedCenterSign = currentSign;
      else if (currentSign != expectedCenterSign)
      {
        Serial.println("Autonomous light check: imbalance is not stable -> no movement.");
        return false;
      }
    }
    else if (lightMode == LIGHT_LEFT)
    {
      if (difference >= LDR_DEADBAND)
      {
        Serial.println("Autonomous light check: LEFT orientation already acceptable -> no movement.");
        return false;
      }
    }
    else
    {
      if (difference <= -LDR_DEADBAND)
      {
        Serial.println("Autonomous light check: RIGHT orientation already acceptable -> no movement.");
        return false;
      }
    }

    if (i + 1 < LDR_CENTER_START_CONFIRM_COUNT)
      serviceDelay(LDR_CONFIRM_INTERVAL_MS);
  }

  Serial.println("Autonomous light check: correction is justified.");
  return true;
}

void automaticLightTracking()
{
  if (!lightTrackingEnabled)
  {
    Serial.println("Light tracking is OFF.");
    return;
  }

  Serial.println();
  Serial.println("--- AUTOMATIC LIGHT ORIENTATION ---");

  if (lightMode == LIGHT_CENTER)
  {
    seekLightBalance();
  }
  else if (lightMode == LIGHT_LEFT)
  {
    seekSelectedSidePeak(LIGHT_LEFT);
  }
  else
  {
    seekSelectedSidePeak(LIGHT_RIGHT);
  }

  sendBlynkData();
}

// ============================================================
// BLYNK DASHBOARD DATA + DYNAMIC COLORS
// ============================================================
void updateDashboardColors(int moisture, bool waterPresent, int battPct)
{
  if (!cloudConnected || !Blynk.connected()) return;

  // Soil: red when very dry, amber in hysteresis band, green near target.
  int startThreshold = max(0, (int)targetMoisturePercent - (int)MOISTURE_HYSTERESIS_PERCENT);
  int8_t soilZone = (moisture <= startThreshold) ? 0 :
                    (moisture < targetMoisturePercent) ? 1 : 2;

  if (soilZone != lastSoilColorZone)
  {
    const char* color = (soilZone == 0) ? "#D3435C" :
                        (soilZone == 1) ? "#F5A623" : "#23C48E";
    Blynk.setProperty(V0, "color", color);
    lastSoilColorZone = soilZone;
  }

  int8_t batteryZone = (battPct <= 15) ? 0 : (battPct <= 40 ? 1 : 2);
  if (batteryZone != lastBatteryColorZone)
  {
    const char* color = (batteryZone == 0) ? "#D3435C" :
                        (batteryZone == 1) ? "#F5A623" : "#23C48E";
    Blynk.setProperty(V7, "color", color);
    lastBatteryColorZone = batteryZone;
  }

  int8_t waterState = waterPresent ? 1 : 0;
  if (waterState != lastWaterColorState)
  {
    Blynk.setProperty(V1, "color", waterPresent ? "#23C48E" : "#D3435C");
    lastWaterColorState = waterState;
  }
}

void sendBlynkData()
{
  if (!cloudConnected || !Blynk.connected()) return;

  int soilRaw = readSoil();
  int moisture = soilPercent(soilRaw);
  int leftRaw = readLDR(LDR_LEFT_PIN);
  int rightRaw = readLDR(LDR_RIGHT_PIN);
  float batteryV = readBatteryVoltage();
  int battPct = batteryPercent(batteryV);
  bool waterPresent = tankHasWater();

  Blynk.virtualWrite(V0, moisture);
  Blynk.virtualWrite(V1, waterPresent ? 1 : 0);
  Blynk.virtualWrite(V2, lightPercent(leftRaw));
  Blynk.virtualWrite(V3, lightPercent(rightRaw));
  Blynk.virtualWrite(V7, battPct);

  updateDashboardColors(moisture, waterPresent, battPct);
}

// ============================================================
// WARNINGS / PUSH NOTIFICATIONS
// ============================================================
void processWarnings()
{
  float batteryV = readBatteryVoltage();
  int moisture = soilPercent(readSoil());

  // ---------- WATER RESERVOIR ----------
  bool waterPresent = tankHasWater();

  if (!waterPresent)
  {
    // Preserve this information across Deep Sleep so a refill can be
    // acknowledged after EXT0 wakes the controller.
    tankWasEmpty = true;

    if (!tankAlertSent && cloudConnected && Blynk.connected())
    {
      Blynk.logEvent("water_empty", "Water reservoir is empty.");
      tankAlertSent = true;
    }
  }
  else
  {
    // Send one confirmation after an empty -> refilled transition.
    // If the cloud is unavailable, keep tankWasEmpty=true and retry
    // after the connection is restored.
    if (tankWasEmpty && cloudConnected && Blynk.connected())
    {
      Blynk.logEvent("water_refilled", "Water reservoir has been refilled. Normal operation restored.");
      tankWasEmpty = false;
    }

    tankAlertSent = false;
  }

  // ---------- BATTERY ----------
  if (batteryV <= BATTERY_LOW_V)
  {
    if (!batteryAlertSent)
    {
      lowBatteryBlink();

      if (cloudConnected && Blynk.connected())
      {
        Blynk.logEvent(
          "low_battery",
          String("Battery is low: ") + String(batteryV, 2) + " V"
        );
      }

      batteryAlertSent = true;
    }
  }
  else if (batteryV >= BATTERY_RESET_V)
  {
    batteryAlertSent = false;
  }

  // ---------- CRITICALLY DRY SOIL ----------
  if (moisture <= CRITICALLY_DRY_PERCENT)
  {
    if (!drySoilAlertSent && cloudConnected && Blynk.connected())
    {
      Blynk.logEvent(
        "critically_dry_soil",
        String("Soil moisture is critically low: ") + moisture + " %"
      );
      drySoilAlertSent = true;
    }
  }
  else if (moisture >= DRY_ALERT_RESET_PERCENT)
  {
    drySoilAlertSent = false;
  }
}

// ============================================================
// TIME / SCHEDULE
// ============================================================
bool timeIsValid()
{
  return time(nullptr) >= VALID_EPOCH_MIN;
}

void startNtpClock()
{
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  uint32_t start = millis();
  while (!timeIsValid() && millis() - start < 4000)
  {
    if (Blynk.connected()) Blynk.run();
    delay(100);
  }

  Serial.println(timeIsValid() ? "NTP time synchronized." : "NTP time not yet available.");
}

int32_t currentLocalDayKey()
{
  if (!timeIsValid()) return -1;
  int64_t localEpoch = (int64_t)time(nullptr) + timezoneOffsetSec;
  return (int32_t)(localEpoch / 86400LL);
}

int32_t currentLocalSecondOfDay()
{
  if (!timeIsValid()) return -1;
  int64_t localEpoch = (int64_t)time(nullptr) + timezoneOffsetSec;
  int32_t sec = (int32_t)(localEpoch % 86400LL);
  if (sec < 0) sec += 86400;
  return sec;
}

bool scheduledWateringDueNow()
{
  if (!scheduleConfigured || !timeIsValid()) return false;

  int32_t secNow = currentLocalSecondOfDay();
  int32_t dayKey = currentLocalDayKey();
  if (secNow < 0 || dayKey < 0) return false;

  int32_t delta = secNow - wateringTimeSec;
  if (delta < 0 || delta > (int32_t)SCHEDULE_GRACE_SECONDS) return false;

  int32_t lastDay = preferences.getInt("lastSchedDay", -1);
  return dayKey != lastDay;
}

void markScheduledWateringTriggered()
{
  int32_t dayKey = currentLocalDayKey();
  if (dayKey >= 0)
  {
    preferences.putInt("lastSchedDay", dayKey);
  }
}

uint64_t secondsUntilNextScheduledWatering()
{
  if (!scheduleConfigured || !timeIsValid())
  {
    return FALLBACK_AUTO_INTERVAL_SECONDS;
  }

  int32_t secNow = currentLocalSecondOfDay();
  if (secNow < 0) return FALLBACK_AUTO_INTERVAL_SECONDS;

  int32_t delta = wateringTimeSec - secNow;
  if (delta <= 0) delta += 86400;

  // Never request a zero/very tiny sleep because of rounding.
  if (delta < 5) delta = 5;
  return (uint64_t)delta;
}

void handleScheduledWatering()
{
  if (!scheduledWateringDueNow()) return;

  Serial.println();
  Serial.println("=== SCHEDULED WATERING CHECK ===");
  markScheduledWateringTriggered();

  // Important: this is a CHECK at the selected time. The pump runs
  // only if the moisture logic decides watering is actually needed.
  WateringResult result = automaticWatering(false);

  if (result == WATERING_INCOMPLETE)
  {
    if (wateringRetryCount < MAX_WATERING_RETRIES)
    {
      wateringRetryCount++;
      wateringRetryPending = true;
      Serial.println("Target not reached; one bounded retry is pending.");
    }
    else
    {
      clearWateringRetryState();
    }
  }
  else
  {
    clearWateringRetryState();
  }

  processWarnings();
  sendBlynkData();
}

// ============================================================
// MANUAL CONTROL POLICY
// ============================================================
bool manualControlsAllowed()
{
  // Presentation mode is always interactive.
  // Autonomous mode accepts momentary manual actuator commands only
  // while V13 Manual Mode / Stay Awake is enabled.
  return PRESENTATION_MODE || manualStayAwakeEnabled;
}

// ============================================================
// BLYNK INPUTS
// ============================================================
BLYNK_WRITE(V4)
{
  int value = constrain(param.asInt(), 20, 90);
  targetMoisturePercent = (uint8_t)value;
  preferences.putUChar("target", targetMoisturePercent);

  Serial.print("Target moisture set to ");
  Serial.print(targetMoisturePercent);
  Serial.println(" %");
}

BLYNK_WRITE(V8)
{
  autoWateringEnabled = param.asInt() == 1;
  preferences.putBool("autoWater", autoWateringEnabled);

  Serial.print("Auto Watering: ");
  Serial.println(autoWateringEnabled ? "ON" : "OFF");
}

BLYNK_WRITE(V9)
{
  bool newState = param.asInt() == 1;
  bool wasEnabled = lightTrackingEnabled;

  lightTrackingEnabled = newState;
  preferences.putBool("lightTrack", lightTrackingEnabled);

  Serial.print("Light Tracking: ");
  Serial.println(lightTrackingEnabled ? "ON" : "OFF");

  // Do not reset the 12-hour timer merely because Blynk synchronizes
  // the same value after every wake. Only a real OFF->ON transition
  // makes an autonomous evaluation due immediately.
  if (lightTrackingEnabled && !wasEnabled)
  {
    if (PRESENTATION_MODE) lightTrackingRequested = true;
    else autonomousLightRemainingSeconds = 0;
  }
}

BLYNK_WRITE(V10)
{
  // Current Blynk Time Input format:
  // param[0] = selected start time in seconds from midnight
  // param[2] = timezone name
  // param[4] = timezone offset from UTC in seconds
  // This firmware intentionally treats the selected time as DAILY.
  int32_t newTime = param[0].asInt();

  if (newTime >= 0 && newTime < 86400)
  {
    wateringTimeSec = newTime;
    scheduleConfigured = true;

    scheduleTimezone = param[2].asStr();
    timezoneOffsetSec = param[4].asInt();

    preferences.putInt("waterTime", wateringTimeSec);
    preferences.putInt("tzOffset", timezoneOffsetSec);
    preferences.putBool("schedSet", true);
    preferences.putString("tzName", scheduleTimezone);

    Serial.print("Watering-check time set: ");
    Serial.print(wateringTimeSec / 3600);
    Serial.print(":");
    int minute = (wateringTimeSec % 3600) / 60;
    if (minute < 10) Serial.print("0");
    Serial.print(minute);
    Serial.print(" | UTC offset: ");
    Serial.println(timezoneOffsetSec);
  }
  else
  {
    scheduleConfigured = false;
    wateringTimeSec = -1;
    preferences.putBool("schedSet", false);
    Serial.println("Watering schedule cleared/invalid.");
  }
}

BLYNK_WRITE(V11)
{
  if (param.asInt() == 1)
  {
    if (!manualControlsAllowed())
    {
      Serial.println("Manual Watering ignored: enable V13 Manual Mode first.");
      if (cloudConnected && Blynk.connected()) Blynk.virtualWrite(V11, 0);
      return;
    }

    manualWateringRequested = true;
  }
}

BLYNK_WRITE(V12)
{
  const bool pressed = (param.asInt() == 1);

  // Manual Rotation is a HOLD-TO-RUN control:
  // press/hold -> motor ON, release -> motor OFF.
  if (pressed)
  {
    if (!manualControlsAllowed())
    {
      Serial.println("Manual Rotation ignored: enable V13 Manual Mode first.");
      manualRotationActive = false;
      digitalWrite(MOTOR_PIN, LOW);
      setMotorStatus(false);
      if (cloudConnected && Blynk.connected()) Blynk.virtualWrite(V12, 0);
      return;
    }

    if (readBatteryVoltage() <= BATTERY_CRITICAL_V)
    {
      Serial.println("MANUAL ROTATION BLOCKED: battery critical");
      manualRotationActive = false;
      digitalWrite(MOTOR_PIN, LOW);
      setMotorStatus(false);
      Blynk.virtualWrite(V12, 0);
      return;
    }

    manualRotationActive = true;
    digitalWrite(MOTOR_PIN, HIGH);
    setMotorStatus(true);
    Serial.println("Manual rotation START - hold button to keep rotating.");
  }
  else
  {
    manualRotationActive = false;
    digitalWrite(MOTOR_PIN, LOW);
    setMotorStatus(false);
    Serial.println("Manual rotation STOP - button released.");
  }
}

BLYNK_WRITE(V13)
{
  bool newState = (param.asInt() == 1);
  bool changed = (newState != manualStayAwakeEnabled);
  manualStayAwakeEnabled = newState;

  if (changed)
  {
    Serial.print("Manual Mode / Stay Awake: ");
    Serial.println(manualStayAwakeEnabled ? "ON" : "OFF");
  }

  if (manualStayAwakeEnabled)
  {
    // Entering manual mode must never start an actuator by itself.
    manualWateringRequested = false;
    setPumpState(false);
    manualRotationActive = false;
    digitalWrite(MOTOR_PIN, LOW);
    setMotorStatus(false);

    if (cloudConnected && Blynk.connected())
    {
      Blynk.virtualWrite(V11, 0);
      Blynk.virtualWrite(V12, 0);
    }
  }
  else
  {
    // Leaving manual mode immediately returns all manual outputs to safe OFF.
    manualWateringRequested = false;
    setPumpState(false);
    if (manualRotationActive) stopManualRotation("Manual Mode OFF");
    else
    {
      digitalWrite(MOTOR_PIN, LOW);
      setMotorStatus(false);
    }

    if (cloudConnected && Blynk.connected())
    {
      Blynk.virtualWrite(V11, 0);
      Blynk.virtualWrite(V12, 0);
    }
  }
}

BLYNK_WRITE(V14)
{
  int value = constrain(param.asInt(), 0, 2);
  uint8_t newMode = (uint8_t)value;
  bool modeChanged = newMode != lightMode;

  lightMode = newMode;
  preferences.putUChar("lightMode", lightMode);

  Serial.print("Light orientation mode: ");
  if (lightMode == LIGHT_CENTER) Serial.println("CENTER");
  else if (lightMode == LIGHT_LEFT) Serial.println("LEFT");
  else Serial.println("RIGHT");

  if (lightTrackingEnabled && modeChanged)
  {
    if (PRESENTATION_MODE) lightTrackingRequested = true;
    else autonomousLightRemainingSeconds = 0;
  }
}

BLYNK_CONNECTED()
{
  // Restore only persistent controls. Never sync momentary buttons
  // V11/V12, otherwise a stale '1' could actuate hardware.
  Blynk.syncVirtual(V4);
  Blynk.syncVirtual(V8);
  Blynk.syncVirtual(V9);
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V13);
  Blynk.syncVirtual(V14);

  // Ensure status indicators start in a safe state.
  Blynk.virtualWrite(V5, 0);
  Blynk.virtualWrite(V6, 0);
}

// ============================================================
// NETWORK
// ============================================================
bool connectToBlynk()
{
  cloudConnected = false;

  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
  {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi unavailable - hardware remains locally autonomous.");
    return false;
  }

  Serial.print("Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (!Blynk.connect(5000))
  {
    Serial.println("Blynk unavailable - hardware remains locally autonomous.");
    return false;
  }

  cloudConnected = true;
  Serial.println("Blynk connected.");

  // Allow BLYNK_CONNECTED() sync callbacks to arrive.
  serviceDelay(1500);
  startNtpClock();
  return true;
}

// ============================================================
// MANUAL COMMAND HANDLERS
// ============================================================
void handleManualWatering()
{
  if (!manualWateringRequested) return;
  manualWateringRequested = false;

  Serial.println("Manual watering command.");

  // Safety has priority over the manual command:
  // never allow the pump to start with an empty reservoir.
  if (!tankHasWater())
  {
    setPumpState(false);
    Serial.println("MANUAL WATERING BLOCKED: reservoir empty");

    if (cloudConnected && Blynk.connected())
    {
      // Return the momentary Manual Watering control to OFF.
      Blynk.virtualWrite(V11, 0);

      // Give the user a specific reminder that the manual command
      // was rejected because the reservoir needs refilling.
      const uint32_t now = millis();
      if (lastManualEmptyReminderMs == 0 ||
          now - lastManualEmptyReminderMs >= MANUAL_EMPTY_REMINDER_COOLDOWN_MS)
      {
        Blynk.logEvent(
          "water_empty",
          "Manual watering blocked. Please refill the water reservoir."
        );
        lastManualEmptyReminderMs = now;
      }
    }

    // Keep the normal empty-tank alert state consistent so that
    // processWarnings() does not immediately duplicate the same event.
    tankAlertSent = true;
    tankWasEmpty = true;

    sendBlynkData();
    return;
  }

  // Water is present: execute the normal bounded manual pump pulse.
  runPump(PUMP_DURATION_MS);

  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V11, 0);
  }

  processWarnings();
  sendBlynkData();
}

void stopManualRotation(const char* reason)
{
  if (!manualRotationActive) return;

  manualRotationActive = false;
  digitalWrite(MOTOR_PIN, LOW);
  setMotorStatus(false);

  Serial.print("Manual rotation stopped: ");
  Serial.println(reason);
}


// ============================================================
// AUTONOMOUS MANUAL / STAY-AWAKE MODE (V13)
// ============================================================
void runManualStayAwakeMode()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println(" MANUAL MODE ACTIVE - DEEP SLEEP OFF");
  Serial.println(" Automatic watering/light movement paused");
  Serial.println(" V11 Manual Watering and V12 Rotation active");
  Serial.println("========================================");

  // A manual session always starts from a safe output state.
  allOutputsOff();
  manualWateringRequested = false;

  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V11, 0);
    Blynk.virtualWrite(V12, 0);
    sendBlynkData();
  }

  uint32_t lastManualReconnectAttemptMs = 0;
  uint32_t lastManualLiveUpdateMs = 0;
  uint32_t lastManualWarningCheckMs = 0;

  while (manualStayAwakeEnabled)
  {
    if (cloudConnected && Blynk.connected())
    {
      Blynk.run();
    }
    else if (millis() - lastManualReconnectAttemptMs >= 15000UL)
    {
      lastManualReconnectAttemptMs = millis();
      connectToBlynk();
    }

    handleManualWatering();

    // HOLD-to-run rotation must stop if remote control is lost.
    if (manualRotationActive && (!cloudConnected || !Blynk.connected()))
    {
      stopManualRotation("Blynk disconnected");
    }

    if (millis() - lastManualLiveUpdateMs >= LIVE_UPDATE_INTERVAL_MS)
    {
      lastManualLiveUpdateMs = millis();
      sendBlynkData();
    }

    if (millis() - lastManualWarningCheckMs >= WARNING_CHECK_INTERVAL_MS)
    {
      lastManualWarningCheckMs = millis();
      processWarnings();

      // Battery safety has priority over Stay Awake. At the critical
      // threshold the actuators are already blocked; force a return to
      // autonomous sleep so Manual Mode cannot drain the cell further.
      if (readBatteryVoltage() <= BATTERY_CRITICAL_V)
      {
        Serial.println("Critical battery: leaving Manual Mode and allowing Deep Sleep.");
        allOutputsOff();
        manualStayAwakeEnabled = false;
        if (cloudConnected && Blynk.connected())
        {
          Blynk.virtualWrite(V13, 0);
          Blynk.virtualWrite(V11, 0);
          Blynk.virtualWrite(V12, 0);
        }
        break;
      }
    }

    delay(10);
  }

  allOutputsOff();
  manualWateringRequested = false;

  if (cloudConnected && Blynk.connected())
  {
    Blynk.virtualWrite(V11, 0);
    Blynk.virtualWrite(V12, 0);
    sendBlynkData();
  }

  Serial.println("Manual Mode ended. Returning to autonomous logic.");
}

// ============================================================
// DEEP SLEEP
// ============================================================
void sleepFor(uint64_t seconds)
{
  if (!PRESENTATION_MODE && manualStayAwakeEnabled)
  {
    Serial.println("Deep Sleep suppressed: V13 Manual Mode is ON.");
    return;
  }

  allOutputsOff();

  // Timer sleep has a known duration, so it can safely count toward
  // the 12-hour autonomous light interval. EXT0 refill sleep is handled
  // separately because its duration is unknown.
  if (!PRESENTATION_MODE && autonomousLightRemainingSeconds > 0)
  {
    if (seconds >= autonomousLightRemainingSeconds)
      autonomousLightRemainingSeconds = 0;
    else
      autonomousLightRemainingSeconds -= seconds;
  }

  Serial.print("Deep Sleep for ");
  Serial.print(seconds);
  Serial.println(" s");
  Serial.flush();

  if (Blynk.connected()) Blynk.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  preferences.end();

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(seconds * US_PER_SECOND);
  esp_deep_sleep_start();
}

void sleepUntilTankRefilled()
{
  if (!PRESENTATION_MODE && manualStayAwakeEnabled)
  {
    Serial.println("EXT0 refill sleep suppressed: V13 Manual Mode is ON.");
    return;
  }

  allOutputsOff();
  Serial.println("Reservoir empty. Sleeping until float switch becomes LOW (refilled).");
  Serial.flush();

  if (Blynk.connected()) Blynk.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  preferences.end();

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  rtc_gpio_init(GPIO_NUM_33);
  rtc_gpio_set_direction(GPIO_NUM_33, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_33);
  rtc_gpio_pulldown_dis(GPIO_NUM_33);

  // Verified float logic: LOW means water present.
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
  esp_deep_sleep_start();
}

void printWakeupReason()
{
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  switch (cause)
  {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup: timer.");
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup: reservoir refilled.");
      break;
    default:
      Serial.println("Startup / power-on reset.");
      break;
  }
}

// Return the next useful autonomous wake-up time. The system wakes for
// whichever is due first: the watering check or the 12-hour light evaluation.
// This keeps light evaluation independent of a once-daily Watering Time.
uint64_t secondsUntilNextAutonomousWake()
{
  uint64_t wateringSleep = scheduleConfigured
      ? secondsUntilNextScheduledWatering()
      : FALLBACK_AUTO_INTERVAL_SECONDS;

  if (!lightTrackingEnabled)
    return wateringSleep;

  uint64_t lightSleep = autonomousLightRemainingSeconds;
  if (lightSleep == 0) lightSleep = AUTONOMOUS_LIGHT_INTERVAL_SECONDS;

  return min(wateringSleep, lightSleep);
}

// ============================================================
// AUTONOMOUS ONE-SHOT CYCLE
// ============================================================
void runAutonomousCycle()
{
  processWarnings();
  sendBlynkData();

  // V13 is synchronized immediately after Blynk connection. When it is
  // ON, the system becomes an interactive manual terminal: automatic
  // watering and automatic light movement are paused and Deep Sleep is
  // suppressed until V13 is turned OFF.
  if (manualStayAwakeEnabled)
  {
    Serial.println("Autonomous cycle paused: V13 Manual Mode is ON.");
    return;
  }

  if (!tankHasWater())
  {
    serviceDelay(400);
    sleepUntilTankRefilled();
    if (manualStayAwakeEnabled) return;
  }

  float batteryV = readBatteryVoltage();
  if (batteryV <= BATTERY_CRITICAL_V)
  {
    Serial.println("Critical battery: actuators disabled.");
    serviceDelay(400);
    sleepFor(secondsUntilNextScheduledWatering());
    if (manualStayAwakeEnabled) return;
  }

  WateringResult wateringResult = WATERING_NOT_NEEDED;

  if (wateringRetryPending)
  {
    wateringResult = automaticWatering(true);
  }
  else if (scheduleConfigured)
  {
    if (scheduledWateringDueNow())
    {
      markScheduledWateringTriggered();
      wateringResult = automaticWatering(false);
    }
    else
    {
      Serial.println("Not at scheduled watering-check time.");
    }
  }
  else
  {
    // No V10 time configured: retain the classic periodic mode.
    wateringResult = automaticWatering(false);
  }

  if (wateringResult == WATERING_INCOMPLETE)
  {
    if (wateringRetryCount < MAX_WATERING_RETRIES)
    {
      wateringRetryCount++;
      wateringRetryPending = true;
      processWarnings();
      sendBlynkData();
      serviceDelay(400);
      sleepFor(RETRY_SLEEP_SECONDS);
      if (manualStayAwakeEnabled) return;
    }
    else
    {
      Serial.println("Retry safety limit reached; no further 5-minute retries.");
      clearWateringRetryState();
    }
  }
  else
  {
    clearWateringRetryState();
  }

  if (!tankHasWater())
  {
    processWarnings();
    sendBlynkData();
    serviceDelay(400);
    sleepUntilTankRefilled();
    if (manualStayAwakeEnabled) return;
  }

  if (manualStayAwakeEnabled)
  {
    Serial.println("Manual Mode requested during autonomous cycle; automatic actions paused.");
    allOutputsOff();
    return;
  }

  // ----------------------------------------------------------
  // AUTONOMOUS LIGHT ORIENTATION
  // ----------------------------------------------------------
  // The pot does not continuously chase the light. A new evaluation
  // is allowed only when the 12-hour interval has elapsed. Even then,
  // the motor moves only if the light is sufficient and a stable,
  // meaningful orientation error is confirmed.
  if (lightTrackingEnabled)
  {
    if (autonomousLightRemainingSeconds == 0)
    {
      Serial.println();
      Serial.println("=== 12-HOUR AUTONOMOUS LIGHT EVALUATION ===");

      if (autonomousLightOrientationNeeded())
      {
        automaticLightTracking();
      }
      else
      {
        Serial.println("Light orientation retained; motor remains OFF.");
      }

      // Reset after every evaluation, not only after movement.
      // Therefore no new automatic orientation is allowed for 12 h.
      autonomousLightRemainingSeconds = AUTONOMOUS_LIGHT_INTERVAL_SECONDS;
    }
    else
    {
      Serial.print("Next autonomous light evaluation in approximately ");
      Serial.print(autonomousLightRemainingSeconds);
      Serial.println(" s.");
    }
  }

  processWarnings();
  sendBlynkData();
  serviceDelay(500);

  if (manualStayAwakeEnabled)
  {
    Serial.println("Manual Mode requested; staying awake instead of Deep Sleep.");
    allOutputsOff();
    return;
  }

  uint64_t nextSleep = secondsUntilNextAutonomousWake();
  sleepFor(nextSleep);
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" SMARTPOT FINAL - 12H + MANUAL MODE");
  Serial.println("====================================");

  // GPIO33 may still be in RTC mode after EXT0 wake.
  rtc_gpio_deinit(GPIO_NUM_33);

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
  allOutputsOff();

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_RIGHT_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);

  preferences.begin("smartpot", false);

  autoWateringEnabled = preferences.getBool("autoWater", true);
  lightTrackingEnabled = preferences.getBool("lightTrack", true);
  targetMoisturePercent = preferences.getUChar("target", DEFAULT_TARGET_MOISTURE);
  lightMode = preferences.getUChar("lightMode", LIGHT_CENTER);

  scheduleConfigured = preferences.getBool("schedSet", false);
  wateringTimeSec = preferences.getInt("waterTime", -1);
  timezoneOffsetSec = preferences.getInt("tzOffset", 0);
  scheduleTimezone = preferences.getString("tzName", "UTC");

  printWakeupReason();

  Serial.println("--- INITIAL SENSOR STATUS ---");
  Serial.print("Soil raw: "); Serial.println(readSoil());
  Serial.print("Soil %: "); Serial.println(soilPercent(readSoil()));
  Serial.print("Water: "); Serial.println(tankHasWater() ? "PRESENT" : "EMPTY");
  int ldrLeftRaw = readLDRRaw(LDR_LEFT_PIN);
  int ldrRightRaw = readLDRRaw(LDR_RIGHT_PIN);
  Serial.print("LDR Left raw: "); Serial.println(ldrLeftRaw);
  Serial.print("LDR Right raw: "); Serial.println(ldrRightRaw);
  Serial.print("LDR Left corrected: "); Serial.println(constrain(ldrLeftRaw + LDR_LEFT_CAL_OFFSET, 0, 4095));
  Serial.print("LDR Right corrected: "); Serial.println(constrain(ldrRightRaw + LDR_RIGHT_CAL_OFFSET, 0, 4095));

  float batteryRawV = readBatteryVoltageRaw();
  Serial.print("Battery uncalibrated: "); Serial.print(batteryRawV, 3); Serial.println(" V");
  Serial.print("Battery calibrated: "); Serial.print(batteryRawV * BATTERY_CAL_FACTOR, 3); Serial.println(" V");

  connectToBlynk();
  sendBlynkData();
  processWarnings();

  if (!PRESENTATION_MODE)
  {
    if (manualStayAwakeEnabled)
    {
      runManualStayAwakeMode();
    }

    runAutonomousCycle();
    return;
  }

  Serial.println("PRESENTATION MODE: online and responsive.");

  // In presentation mode, no-schedule fallback may perform an
  // immediate automatic check once; subsequent checks are 12 h apart.
  lastFallbackAutoCheckMs = millis() - FALLBACK_AUTO_INTERVAL_MS;
  lastLightRecheckMs = millis();
}

// ============================================================
// LOOP - PRESENTATION MODE
// ============================================================
void loop()
{
  if (!PRESENTATION_MODE)
  {
    // Normally unreachable because autonomous mode enters Deep Sleep.
    // It becomes reachable only when V13 was enabled during the short
    // awake window and sleep was intentionally suppressed.
    if (manualStayAwakeEnabled)
    {
      runManualStayAwakeMode();
      runAutonomousCycle();
    }

    delay(100);
    return;
  }

  // Keep cloud alive / reconnect without requiring reset.
  if (cloudConnected && Blynk.connected())
  {
    Blynk.run();
  }
  else if (millis() - lastReconnectAttemptMs >= 15000)
  {
    lastReconnectAttemptMs = millis();
    connectToBlynk();
  }

  handleManualWatering();

  // If the phone/Blynk connection disappears while the button is held,
  // stop the motor instead of leaving it running without a release command.
  if (manualRotationActive && (!cloudConnected || !Blynk.connected()))
  {
    stopManualRotation("Blynk disconnected");
  }

  if (lightTrackingRequested && !manualRotationActive)
  {
    lightTrackingRequested = false;
    automaticLightTracking();
    lastLightRecheckMs = millis();
  }

  // Periodic automatic re-orientation when tracking is enabled.
  if (lightTrackingEnabled && !manualRotationActive &&
      millis() - lastLightRecheckMs >= LIGHT_RECHECK_INTERVAL_MS)
  {
    lastLightRecheckMs = millis();
    automaticLightTracking();
  }

  if (millis() - lastLiveUpdateMs >= LIVE_UPDATE_INTERVAL_MS)
  {
    lastLiveUpdateMs = millis();
    sendBlynkData();
  }

  if (millis() - lastWarningCheckMs >= WARNING_CHECK_INTERVAL_MS)
  {
    lastWarningCheckMs = millis();
    processWarnings();
  }

  if (millis() - lastScheduleCheckMs >= SCHEDULE_CHECK_INTERVAL_MS)
  {
    lastScheduleCheckMs = millis();

    if (scheduleConfigured)
    {
      handleScheduledWatering();
    }
    else if (autoWateringEnabled &&
             millis() - lastFallbackAutoCheckMs >= FALLBACK_AUTO_INTERVAL_MS)
    {
      lastFallbackAutoCheckMs = millis();
      WateringResult result = automaticWatering(false);

      if (result == WATERING_INCOMPLETE && wateringRetryCount < MAX_WATERING_RETRIES)
      {
        wateringRetryCount++;
        wateringRetryPending = true;
      }
    }
  }

  // In presentation mode the bounded 5-minute retry is handled
  // without Deep Sleep so the dashboard stays online.
  static uint32_t presentationRetryStartedMs = 0;
  if (wateringRetryPending)
  {
    if (presentationRetryStartedMs == 0)
    {
      presentationRetryStartedMs = millis();
    }

    if (millis() - presentationRetryStartedMs >= (uint32_t)(RETRY_SLEEP_SECONDS * 1000ULL))
    {
      presentationRetryStartedMs = 0;
      WateringResult retryResult = automaticWatering(true);
      clearWateringRetryState();
      (void)retryResult;
    }
  }
  else
  {
    presentationRetryStartedMs = 0;
  }

  delay(10);
}
