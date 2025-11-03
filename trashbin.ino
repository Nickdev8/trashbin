#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <Servo.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Pin Mapping (Music Maker Shield defaults)
// ---------------------------------------------------------------------------
#define MP3_CS     7
#define MP3_DCS    6
#define MP3_DREQ   3
#define SD_CS      4

// Ultrasonic sensors (HC-SR04 style)
#define TRIG1_PIN  A2
#define ECHO1_PIN  A3
#define TRIG2_PIN  A5
#define ECHO2_PIN  A4

// Outputs
#define OUT_PIN        A1
#define OUT_PIN_MIRROR A0
#define SERVO_PIN      9

// ---------------------------------------------------------------------------
// Behaviour Tuning
// ---------------------------------------------------------------------------
const bool debugMode = false;              // Enable serial-trigger mode

// Distance thresholds (centimetres)
const float TRIGGER_DISTANCE_CM   = 38.0f; // Object considered "close"
const float RELEASE_MARGIN_CM     = 8.0f;  // Extra distance to re-arm trigger (hysteresis)

// Confirmation counts
const uint8_t TRIGGER_CONFIRMATIONS  = 3;  // Samples inside trigger band
const uint8_t RELEASE_CONFIRMATIONS  = 4;  // Samples fully outside release band

// Sensor timing
const unsigned long SENSOR_PING_GAP_MS = 60UL;
const unsigned long SENSOR_TIMEOUT_US  = 6000UL;
const float MAX_VALID_DISTANCE_CM      = 60.0f;

// Servo behaviour
const int SERVO_PRESS_DEG     = 5;
const int SERVO_REST_DEG      = 30;
const unsigned long SERVO_HOLD_MS     = 900UL;
const unsigned long SERVO_COOLDOWN_MS = 2500UL;   // Set to 0 for no cooldown

// OUT pin pulse
const unsigned long OUT_PULSE_WIDTH_MS   = 1000UL;
const unsigned long OUT_ARM_TIMEOUT_MS   = 350UL;

// Idle track playback
const char *const IDLE_FILE = "IDLE.MP3";
const unsigned long IDLE_RETRY_INTERVAL_MS = 10000UL;

// Trigger track
const char *const TRIGGER_FILE = "TRIGGER.MP3";

// Random playlist
const uint8_t MAX_RANDOM_TRACKS = 48;

// Misc
const unsigned long LOOP_DELAY_MS      = 10UL;
const unsigned long MIN_LOG_SPACING_MS = 250UL;

// ---------------------------------------------------------------------------
// Global State
// ---------------------------------------------------------------------------
Servo triggerServo;
bool servoActive             = false;
unsigned long servoStartMs   = 0;
unsigned long lastServoPress = 0;

bool outPinState        = LOW;
bool outPulseArmed      = false;
bool outPulseActive     = false;
unsigned long outPulseArmMs   = 0;
unsigned long outPulseStartMs = 0;

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(
  MP3_CS, MP3_DCS, MP3_DREQ, SD_CS
);

bool triggerTrackAvailable = false;
bool idleTrackAvailable    = false;
bool idlePlaying           = false;
bool idleSuppressed        = false;
unsigned long idleLastAttemptMs = 0;

unsigned long lastLogMs = 0;

char randomTracks[MAX_RANDOM_TRACKS][13];
uint8_t randomTrackCount = 0;
uint8_t randomTrackIndex = 0;

// Sensor history
static float lastDistance1 = -1.0f;
static float lastDistance2 = -1.0f;

// Trigger state machine
static uint8_t inRangeStreak     = 0;
static uint8_t releaseStreak     = 0;
static bool triggerLatched       = false;
static unsigned long lastSampleMs = 0;

// ---------------------------------------------------------------------------
// Utility Helpers
// ---------------------------------------------------------------------------
static inline bool isPrintableMp3(const char *name) {
  if (!name) return false;
  char upper[13];
  uint8_t i = 0;
  for (; i < 12 && name[i] != '\0'; ++i) {
    char c = name[i];
    if (c >= 'a' && c <= 'z') c -= 32;
    upper[i] = c;
  }
  upper[i] = '\0';
  if (i < 4) return false;
  if (upper[i - 4] != '.' || upper[i - 3] != 'M' || upper[i - 2] != 'P' || upper[i - 1] != '3') return false;
  if (strcmp(upper, TRIGGER_FILE) == 0) return false;
  if (strcmp(upper, IDLE_FILE) == 0) return false;
  return true;
}

static inline float measureDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float distance = duration * 0.0343f / 2.0f;
  if (distance <= 0.0f || distance > MAX_VALID_DISTANCE_CM) return -1.0f;
  return distance;
}

static inline void logThrottled(const __FlashStringHelper *msg) {
  unsigned long now = millis();
  if (now - lastLogMs >= MIN_LOG_SPACING_MS) {
    Serial.println(msg);
    lastLogMs = now;
  }
}

// ---------------------------------------------------------------------------
// OUT Pin / Pulse Handling
// ---------------------------------------------------------------------------
static void armOutPulseOnPlayback() {
  outPulseArmed = true;
  outPulseArmMs = millis();
  Serial.println(F("OUT pulse armed; waiting for playback start"));
}

static void serviceOutPulseArm() {
  if (!outPulseArmed) return;
  unsigned long now = millis();
  if (musicPlayer.playingMusic) {
    digitalWrite(OUT_PIN, HIGH);
    digitalWrite(OUT_PIN_MIRROR, HIGH);
    outPulseActive = true;
    outPulseStartMs = now;
    outPulseArmed = false;
    Serial.println(F("Playback detected → OUT pulse HIGH"));
  } else if (now - outPulseArmMs >= OUT_ARM_TIMEOUT_MS) {
    outPulseArmed = false;
    Serial.println(F("Playback did not start → OUT pulse cancelled"));
  }
}

static void serviceOutPulse() {
  if (!outPulseActive) return;
  if (millis() - outPulseStartMs >= OUT_PULSE_WIDTH_MS) {
    digitalWrite(OUT_PIN, LOW);
    digitalWrite(OUT_PIN_MIRROR, LOW);
    outPulseActive = false;
    Serial.println(F("OUT pulse complete → LOW"));
  }
}

// ---------------------------------------------------------------------------
// Servo Helpers
// ---------------------------------------------------------------------------
static bool pressServo() {
  unsigned long now = millis();
  if (SERVO_COOLDOWN_MS > 0 && (now - lastServoPress) < SERVO_COOLDOWN_MS) {
    unsigned long remain = SERVO_COOLDOWN_MS - (now - lastServoPress);
    Serial.print(F("Servo cooldown "));
    Serial.print(remain);
    Serial.println(F(" ms remaining"));
    return false;
  }

  lastServoPress = now;
  servoActive = true;
  servoStartMs = now;
  triggerServo.write(SERVO_PRESS_DEG);
  Serial.print(F("Servo press → "));
  Serial.print(SERVO_PRESS_DEG);
  Serial.println(F("°"));
  return true;
}

static void serviceServo() {
  if (!servoActive) return;
  if (millis() - servoStartMs >= SERVO_HOLD_MS) {
    servoActive = false;
    triggerServo.write(SERVO_REST_DEG);
    Serial.print(F("Servo rest → "));
    Serial.print(SERVO_REST_DEG);
    Serial.println(F("°"));
  }
}

// ---------------------------------------------------------------------------
// Idle Playback
// ---------------------------------------------------------------------------
static void suspendIdlePlayback() {
  idleSuppressed = true;
  idleLastAttemptMs = millis();

  if (idlePlaying) {
    Serial.println(F("Stopping idle track for trigger playback"));
    musicPlayer.stopPlaying();
    idlePlaying = false;
  }
}

static bool startIdlePlayback(bool logErrors = true) {
  if (!idleTrackAvailable) return false;
  if (idleSuppressed) return false;
  if (musicPlayer.playingMusic) return false;

  if (musicPlayer.startPlayingFile(IDLE_FILE)) {
    idlePlaying = true;
    idleLastAttemptMs = millis();
    Serial.println(F("Idle playback → IDLE.MP3"));
    return true;
  }

  idlePlaying = false;
  if (logErrors) Serial.println(F("Failed to start IDLE.MP3 (disabling idle mode)"));
  idleTrackAvailable = false;
  return false;
}

static void serviceIdlePlayback() {
  unsigned long now = millis();
  bool currentlyPlaying = musicPlayer.playingMusic;

  if (!currentlyPlaying && idlePlaying) {
    idlePlaying = false;
    Serial.println(F("Idle track finished"));
    idleLastAttemptMs = now;
  }

  if (!currentlyPlaying && idleSuppressed) {
    idleSuppressed = false;
    idleLastAttemptMs = now;
    Serial.println(F("Trigger playback finished → idle unlocked"));
  }

  if (!currentlyPlaying && !idlePlaying && idleTrackAvailable) {
    if (now - idleLastAttemptMs >= IDLE_RETRY_INTERVAL_MS) {
      startIdlePlayback();
    }
  }
}

// ---------------------------------------------------------------------------
// Random Playlist Handling
// ---------------------------------------------------------------------------
static void shufflePlaylist() {
  if (randomTrackCount <= 1) return;
  for (int i = randomTrackCount - 1; i > 0; --i) {
    int j = random(i + 1);
    if (j == i) continue;
    char tmp[13];
    memcpy(tmp, randomTracks[i], sizeof(tmp));
    memcpy(randomTracks[i], randomTracks[j], sizeof(tmp));
    memcpy(randomTracks[j], tmp, sizeof(tmp));
  }
  if (randomTrackCount > 1) {
    randomTrackIndex = random(randomTrackCount);
  } else {
    randomTrackIndex = 0;
  }
}

static void refreshRandomPlaylist() {
  randomTrackCount = 0;
  randomTrackIndex = 0;

  File dir = SD.open("/", FILE_READ);
  if (!dir) {
    Serial.println(F("Failed to open SD root for playlist build"));
    return;
  }

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      const char *entryName = entry.name();
      if (isPrintableMp3(entryName)) {
        if (randomTrackCount < MAX_RANDOM_TRACKS) {
          uint8_t j = 0;
          for (; j < 12 && entryName[j] != '\0'; ++j) {
            randomTracks[randomTrackCount][j] = entryName[j];
          }
          randomTracks[randomTrackCount][j] = '\0';
          randomTrackCount++;
        } else {
          Serial.println(F("Playlist limit reached; additional tracks ignored"));
        }
      }
    }
    entry.close();
  }
  dir.close();

  if (randomTrackCount == 0) {
    Serial.println(F("No random tracks found on SD"));
    return;
  }

  shufflePlaylist();
  Serial.print(F("Random playlist size: "));
  Serial.println(randomTrackCount);
}

static bool playNextRandomTrack() {
  if (randomTrackCount == 0) {
    refreshRandomPlaylist();
    if (randomTrackCount == 0) return false;
  }

  const char *track = randomTracks[randomTrackIndex];
  randomTrackIndex = (randomTrackIndex + 1) % randomTrackCount;

  Serial.print(F("Playing random track: "));
  Serial.println(track);

  if (!musicPlayer.startPlayingFile(track)) {
    Serial.println(F("Failed to play selected random track"));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Trigger Playback
// ---------------------------------------------------------------------------
static bool playTriggerOrRandom() {
  suspendIdlePlayback();

  if (triggerTrackAvailable) {
    if (musicPlayer.startPlayingFile(TRIGGER_FILE)) {
      Serial.println(F("Playing TRIGGER.MP3"));
      armOutPulseOnPlayback();
      return true;
    }
    Serial.println(F("Failed to play TRIGGER.MP3 → using random track"));
    triggerTrackAvailable = false;
  }

  if (playNextRandomTrack()) {
    armOutPulseOnPlayback();
    return true;
  }

  Serial.println(F("No audio available for trigger"));
  idleSuppressed = false;
  return false;
}

// ---------------------------------------------------------------------------
// Sensor Handling
// ---------------------------------------------------------------------------
static void handleSensors() {
  unsigned long now = millis();
  if (now - lastSampleMs < SENSOR_PING_GAP_MS) return;
  lastSampleMs = now;

  lastDistance1 = measureDistanceCm(TRIG1_PIN, ECHO1_PIN);
  delayMicroseconds(1500);
  lastDistance2 = measureDistanceCm(TRIG2_PIN, ECHO2_PIN);

  bool reading1InRange = (lastDistance1 > 0 && lastDistance1 <= TRIGGER_DISTANCE_CM);
  bool reading2InRange = (lastDistance2 > 0 && lastDistance2 <= TRIGGER_DISTANCE_CM);
  bool anyInRange = reading1InRange || reading2InRange;

  bool reading1Out = (lastDistance1 < 0) || (lastDistance1 >= TRIGGER_DISTANCE_CM + RELEASE_MARGIN_CM);
  bool reading2Out = (lastDistance2 < 0) || (lastDistance2 >= TRIGGER_DISTANCE_CM + RELEASE_MARGIN_CM);
  bool bothOut = reading1Out && reading2Out;

  if (anyInRange) {
    if (inRangeStreak < 255) inRangeStreak++;
    releaseStreak = 0;
  } else {
    inRangeStreak = 0;
    if (bothOut) {
      if (releaseStreak < 255) releaseStreak++;
    } else {
      releaseStreak = 0;
    }
  }

  if (!triggerLatched && inRangeStreak >= TRIGGER_CONFIRMATIONS) {
    triggerLatched = true;
    Serial.println(F("Trigger edge detected"));
    digitalWrite(LED_BUILTIN, HIGH);

    bool servoFired = pressServo();
    if (servoFired && (!musicPlayer.playingMusic || idlePlaying)) {
      if (!playTriggerOrRandom()) {
        Serial.println(F("Trigger fired but no track available"));
      }
    }
  }

  if (triggerLatched && releaseStreak >= RELEASE_CONFIRMATIONS) {
    triggerLatched = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(F("Trigger re-armed"));
  }
}

// ---------------------------------------------------------------------------
// Serial Debug Trigger
// ---------------------------------------------------------------------------
static void handleSerialTrigger() {
  if (!debugMode) return;
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() > 0) return;

  Serial.println(F("Debug trigger via serial"));
  bool servoFired = pressServo();
  if (servoFired && (!musicPlayer.playingMusic || idlePlaying)) {
    if (!playTriggerOrRandom()) {
      Serial.println(F("Debug trigger: no audio available"));
    }
  }
}

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------
void setup() {
  randomSeed(analogRead(A0) ^ analogRead(A1) ^ micros());

  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUT_PIN_MIRROR, OUTPUT);
  digitalWrite(OUT_PIN, outPinState);
  digitalWrite(OUT_PIN_MIRROR, outPinState);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  triggerServo.attach(SERVO_PIN);
  triggerServo.write(SERVO_REST_DEG);
  Serial.begin(9600);
  Serial.println(F("System booting..."));
  Serial.print(F("Servo rest angle → "));
  Serial.print(SERVO_REST_DEG);
  Serial.println(F("°"));

  if (!musicPlayer.begin()) {
    Serial.println(F("VS1053 not found; check wiring"));
    while (true) delay(20);
  }

  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD init failed; check card or wiring"));
    while (true) delay(20);
  }

  musicPlayer.setVolume(20, 20);
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);
  Serial.println(F("Music Maker ready"));

  triggerTrackAvailable = SD.exists(TRIGGER_FILE);
  Serial.println(triggerTrackAvailable ? F("TRIGGER.MP3 detected") : F("TRIGGER.MP3 missing → using random tracks"));

  idleTrackAvailable = SD.exists(IDLE_FILE);
  Serial.println(idleTrackAvailable ? F("IDLE.MP3 detected") : F("IDLE.MP3 missing"));

  refreshRandomPlaylist();

  if (idleTrackAvailable) {
    idleSuppressed = false;
    idleLastAttemptMs = millis() - IDLE_RETRY_INTERVAL_MS;
    startIdlePlayback();
  }
}

void loop() {
  handleSensors();
  serviceOutPulseArm();
  serviceOutPulse();
  serviceServo();
  handleSerialTrigger();
  serviceIdlePlayback();
  delay(LOOP_DELAY_MS);
}
