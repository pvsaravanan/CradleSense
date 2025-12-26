/*
 * CradleSense – NICU-Aligned IoT Smart Cradle
 * ESP32 | Non-blocking | Alarm Priority + Data Logging
 */

#include <Wire.h>
#include <Adafruit_MAX30105.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <HX711.h>
#include <Servo.h>

/* ---------------- PIN DEFINITIONS ---------------- */

#define LM35_PIN        34
#define SOUND_PIN       35
#define MOISTURE_PIN    32

#define DHT_PIN         13
#define HX711_DOUT      18
#define HX711_SCK       19

#define BUZZER_PIN      25
#define LED_PIN         26
#define SERVO_PIN       23
#define RELAY_PIN       27

/* ---------------- SYSTEM MODE ---------------- */

#define PRETERM_MODE true   // false = Term infant

/* ---------------- DISPLAY ---------------- */

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

/* ---------------- NICU THRESHOLDS ---------------- */

#if PRETERM_MODE
  const float TEMP_LOW  = 36.0;
  const float TEMP_HIGH = 37.2;
  const float MOTION_THRESHOLD = 12.5;
#else
  const float TEMP_LOW  = 36.5;
  const float TEMP_HIGH = 37.5;
  const float MOTION_THRESHOLD = 13.0;
#endif

const int SOUND_THRESHOLD = 600;

/* ---------------- TIMING ---------------- */

const unsigned long SENSOR_INTERVAL   = 1000;
const unsigned long DISPLAY_INTERVAL  = 1500;
const unsigned long SERVO_INTERVAL    = 700;

const unsigned long TEMP_ALARM_TIME   = 5000;
const unsigned long SOUND_ALARM_TIME  = 3000;
const unsigned long MOTION_ALARM_TIME = 2000;

/* ---------------- ALARM PRIORITY ---------------- */

enum AlarmPriority {
  NO_ALARM = 0,
  LOW_PRIORITY,
  MEDIUM_PRIORITY,
  HIGH_PRIORITY
};

AlarmPriority currentPriority = NO_ALARM;
AlarmPriority lastLoggedPriority = NO_ALARM;

/* ---------------- OBJECTS ---------------- */

DHT dht(DHT_PIN, DHT22);
Adafruit_MAX30105 max30102;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 loadCell;
Servo cradleServo;

/* ---------------- SENSOR STATE ---------------- */

float bodyTemp = 0;
float humidity = 0;
long irValue = 0;
float motionMag = 0;
int soundLevel = 0;
int moistureLevel = 0;

/* ---------------- TIMERS ---------------- */

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastServoMove = 0;

unsigned long tempStart = 0;
unsigned long soundStart = 0;
unsigned long motionStart = 0;

/* ---------------- ALARM FLAGS ---------------- */

bool tempAlarm = false;
bool soundAlarm = false;
bool motionAlarm = false;

/* =================================================
   DATA LOGGING STRUCTURE
   ================================================= */

#define LOG_SIZE 50

struct LogEntry {
  unsigned long timestamp;
  float temp;
  float motion;
  int sound;
  AlarmPriority priority;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;

/* ---------------- SETUP ---------------- */

void setup() {
  Serial.begin(115200);
  Wire.begin();

  dht.begin();
  max30102.begin();
  mpu.begin();
  loadCell.begin(HX711_DOUT, HX711_SCK);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  cradleServo.attach(SERVO_PIN);
  cradleServo.write(90);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("CradleSense NICU");
  display.display();

  Serial.println("Time(ms),Temp,Motion,Sound,Priority");
}

/* ---------------- LOOP ---------------- */

void loop() {
  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    evaluateAlarms(now);
    evaluatePriority();
    updateAlerts();
    logDataIfNeeded(now);
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    updateDisplay();
  }

  handleServo(now);
}

/* ---------------- SENSOR TASK ---------------- */

void readSensors() {
  bodyTemp = (analogRead(LM35_PIN) * 3.3 / 4095.0) * 100.0;
  humidity = dht.readHumidity();
  irValue = max30102.getIR();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  motionMag = sqrt(
    sq(a.acceleration.x) +
    sq(a.acceleration.y) +
    sq(a.acceleration.z)
  );

  soundLevel = analogRead(SOUND_PIN);
  moistureLevel = analogRead(MOISTURE_PIN);
}

/* ---------------- ALARM FILTERING ---------------- */

void evaluateAlarms(unsigned long now) {

  if (bodyTemp < TEMP_LOW || bodyTemp > TEMP_HIGH) {
    if (!tempStart) tempStart = now;
    if (now - tempStart >= TEMP_ALARM_TIME) tempAlarm = true;
  } else {
    tempStart = 0;
    tempAlarm = false;
  }

  if (soundLevel > SOUND_THRESHOLD) {
    if (!soundStart) soundStart = now;
    if (now - soundStart >= SOUND_ALARM_TIME) soundAlarm = true;
  } else {
    soundStart = 0;
    soundAlarm = false;
  }

  if (motionMag > MOTION_THRESHOLD) {
    if (!motionStart) motionStart = now;
    if (now - motionStart >= MOTION_ALARM_TIME) motionAlarm = true;
  } else {
    motionStart = 0;
    motionAlarm = false;
  }
}

/* ---------------- PRIORITY ---------------- */

void evaluatePriority() {
  if (tempAlarm) currentPriority = HIGH_PRIORITY;
  else if (motionAlarm) currentPriority = MEDIUM_PRIORITY;
  else if (soundAlarm) currentPriority = LOW_PRIORITY;
  else currentPriority = NO_ALARM;
}

/* ---------------- DATA LOGGING ---------------- */

void logDataIfNeeded(unsigned long now) {

  // Log only if priority changes OR every 10 seconds
  static unsigned long lastPeriodicLog = 0;

  if (currentPriority != lastLoggedPriority ||
      now - lastPeriodicLog >= 10000) {

    LogEntry &entry = logBuffer[logIndex];
    entry.timestamp = now;
    entry.temp = bodyTemp;
    entry.motion = motionMag;
    entry.sound = soundLevel;
    entry.priority = currentPriority;

    // Print log
    Serial.print(entry.timestamp); Serial.print(",");
    Serial.print(entry.temp); Serial.print(",");
    Serial.print(entry.motion); Serial.print(",");
    Serial.print(entry.sound); Serial.print(",");
    Serial.println(entry.priority);

    logIndex = (logIndex + 1) % LOG_SIZE;
    lastLoggedPriority = currentPriority;
    lastPeriodicLog = now;
  }
}

/* ---------------- ALERT OUTPUT ---------------- */

void updateAlerts() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();

  switch (currentPriority) {
    case HIGH_PRIORITY:
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      break;

    case MEDIUM_PRIORITY:
      if (now - lastBlink >= 700) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        digitalWrite(BUZZER_PIN, ledState);
      }
      break;

    case LOW_PRIORITY:
      if (now - lastBlink >= 1000) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
      }
      digitalWrite(BUZZER_PIN, LOW);
      break;

    default:
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
  }
}

/* ---------------- SERVO ---------------- */

void handleServo(unsigned long now) {
  if ((currentPriority == LOW_PRIORITY || currentPriority == MEDIUM_PRIORITY)
      && now - lastServoMove >= SERVO_INTERVAL) {

    lastServoMove = now;
    static bool dir = false;
    dir = !dir;
    cradleServo.write(dir ? 30 : 150);
  }

  if (currentPriority == HIGH_PRIORITY || currentPriority == NO_ALARM) {
    cradleServo.write(90);
  }
}

/* ---------------- DISPLAY ---------------- */

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("Temp: "); display.print(bodyTemp); display.println(" C");
  display.print("Motion: "); display.println(motionMag);
  display.print("Sound: "); display.println(soundLevel);

  display.print("Alarm: ");
  if (currentPriority == HIGH_PRIORITY) display.println("HIGH");
  else if (currentPriority == MEDIUM_PRIORITY) display.println("MED");
  else if (currentPriority == LOW_PRIORITY) display.println("LOW");
  else display.println("NONE");

#if PRETERM_MODE
  display.println("Mode: Preterm");
#else
  display.println("Mode: Term");
#endif

  display.display();
}
