#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <Servo.h>  // NEW

// ------------ Output ------------
#define OUT_PIN A1              // NPN Collector

// ------------ Pins (Music Maker Shield defaults) ------------
#define MP3_CS     7
#define MP3_DCS    6
#define MP3_DREQ   3
#define SD_CS      4

// ------------ Ultrasonic pins ------------
#define TRIG1_PIN  A2
#define ECHO1_PIN  A3
#define TRIG2_PIN  A5   // was 2
#define ECHO2_PIN  A4   // was 5

// ------------ Pushbutton (to GND) ------------
#define BUTTON_PIN A0           // NPN Base

// ------------ Servo ------------
#define SERVO_PIN 9
#define SERVO_HOLD_MS 1000UL
#define SERVO_MIN     5
#define SERVO_MAX     30
#define SERVO_COOLDOWN_MS 3000UL


Servo triggerServo;
bool servoActive = false;
unsigned long servoStartMs = 0;
unsigned long lastServoMoveMs = 0;

// ------------ Behavior ------------
#define MUTE_INTERVAL_MS 200UL
#define TRIGGER_CM     46.0f
#define MAX_VALID_CM   46.0f
#define TIMEOUT_US     7000UL
#define PING_GAP_MS  70UL


// Button timing
#define DEBOUNCE_MS       30UL
#define LONG_PRESS_MS     600UL

// OUT_PIN pulse on music trigger
#define OUT_PULSE_MS          1000UL
#define PULSE_ARM_TIMEOUT_MS   300UL

// Sensor-trigger single file
const char *TRIGGER_FILE = "TRIGGER.MP3";
bool triggerFileAvailable = false;

// Sequence
const char* SEQ_FILES[] = { "SEQ1.MP3", "SEQ2.MP3", "SEQ3.MP3" };
const uint8_t SEQ_LEN = sizeof(SEQ_FILES)/sizeof(SEQ_FILES[0]);

// VS1053 player
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(
  MP3_CS, MP3_DCS, MP3_DREQ, SD_CS
);

unsigned long lastTriggerPrintMs = 0;

// ---- Helper: measure one HC-SR04 safely ----
float measureDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float cm = duration * 0.034f / 2.0f;
  if (cm <= 0.0f || cm > MAX_VALID_CM) return -1.0f;
  if ((int)cm == 794) return -1.0f;
  return cm;
}

// ---- Button state (edge + long-press) ----
enum BtnState { BTN_IDLE, BTN_DEBOUNCE, BTN_PRESSED, BTN_LONGFIRE };
BtnState btnState = BTN_IDLE;
unsigned long btnT0 = 0;
bool lastBtnLevel = HIGH;   // pull-up: HIGH = released

// ---- OUT_PIN control (flipped: idle LOW) ----
bool outPinState = LOW;
bool outPulseActive = false;
unsigned long outPulseStartMs = 0;

// ---- Arm pulse only after playback actually starts ----
bool pulseArmed = false;
unsigned long pulseArmStartMs = 0;

void armOutPulseOnPlaybackStart() {
  pulseArmed = true;
  pulseArmStartMs = millis();
  Serial.println(F("OUT pulse armed: waiting for playback start..."));
}

void servicePulseArm() {
  if (!pulseArmed) return;

  unsigned long now = millis();
  if (musicPlayer.playingMusic) {
    digitalWrite(OUT_PIN, HIGH);
    outPinState = HIGH;
    outPulseActive = true;
    outPulseStartMs = now;
    pulseArmed = false;
    Serial.println(F("Playback started → OUT_PIN pulse: ON for 1s"));
  } else if (now - pulseArmStartMs >= PULSE_ARM_TIMEOUT_MS) {
    pulseArmed = false;
    Serial.println(F("Playback did not start in time → skipping OUT pulse"));
  }
}

void serviceOutPulse() {
  if (outPulseActive && (millis() - outPulseStartMs >= OUT_PULSE_MS)) {
    digitalWrite(OUT_PIN, LOW);
    outPinState = LOW;
    outPulseActive = false;
    Serial.println(F("OUT_PIN pulse: back OFF"));
  }
}

// ---- Sequence state ----
bool seqActive = false;
uint8_t seqIndex = 0;

void startSequence() {
  seqActive = true;
  seqIndex = 0;
  if (!musicPlayer.startPlayingFile(SEQ_FILES[seqIndex])) {
    Serial.println(F("Sequence: failed to start first file."));
    seqActive = false;
  } else {
    Serial.print(F("Sequence: starting ")); Serial.println(SEQ_FILES[seqIndex]);
    armOutPulseOnPlaybackStart();
  }
}

void advanceSequenceIfNeeded() {
  if (!seqActive) return;
  if (musicPlayer.playingMusic) return;

  seqIndex++;
  if (seqIndex >= SEQ_LEN) {
    seqActive = false;
    Serial.println(F("Sequence: done."));
    return;
  }
  if (!musicPlayer.startPlayingFile(SEQ_FILES[seqIndex])) {
    Serial.print(F("Sequence: failed to play ")); Serial.println(SEQ_FILES[seqIndex]);
    seqActive = false;
  } else {
    Serial.print(F("Sequence: starting ")); Serial.println(SEQ_FILES[seqIndex]);
    // Optionally step WLED on each track change:
    // armOutPulseOnPlaybackStart();
  }
}

bool playRandomMp3() {
  File dir = SD.open("/");
  if (!dir) {
    Serial.println(F("Failed to open SD root for random MP3 lookup."));
    return false;
  }

  String chosen = "";
  uint16_t candidates = 0;

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      String name = String(entry.name());
      String upper = name;
      upper.toUpperCase();
      if (upper.endsWith(".MP3") && upper != "TRIGGER.MP3") {
        candidates++;
        if (random(candidates) == 0) {
          chosen = name;
        }
      }
    }
    entry.close();
  }
  dir.close();

  if (candidates == 0) {
    Serial.println(F("No alternate MP3 files found on SD."));
    return false;
  }

  Serial.print(F("Random MP3 selected: "));
  Serial.println(chosen);

  if (!musicPlayer.startPlayingFile(chosen.c_str())) {
    Serial.println(F("Failed to start selected random MP3."));
    return false;
  }

  Serial.println(F("Playing random MP3 fallback."));
  return true;
}

bool playTriggerOrRandom() {
  if (triggerFileAvailable) {
    if (musicPlayer.startPlayingFile(TRIGGER_FILE)) {
      Serial.println(F("Starting TRIGGER.MP3"));
      armOutPulseOnPlaybackStart();
      return true;
    }
    Serial.println(F("Failed to play TRIGGER.MP3, falling back to random MP3."));
    triggerFileAvailable = false;
  }

  if (playRandomMp3()) {
    armOutPulseOnPlaybackStart();
    return true;
  }

  return false;
}

// ---- Servo helpers ----
void triggerServoMove() {
  unsigned long now = millis();

  // Enforce cooldown
  if (now - lastServoMoveMs < SERVO_COOLDOWN_MS) {
    unsigned long remain = SERVO_COOLDOWN_MS - (now - lastServoMoveMs);
    Serial.print(F("Servo: cooldown active, "));
    Serial.print(remain);
    Serial.println(F(" ms remaining"));
    return; // block this trigger
  }

  // Allow the move and start a new cooldown window
  lastServoMoveMs = now;

  // Move to "pressed" position
  triggerServo.write(SERVO_MIN);
  servoActive = true;
  servoStartMs = now;
  Serial.println("Servo: move to " + String(SERVO_MIN) + "° (pressed)");
}

void serviceServo() {
  // After holding for SERVO_HOLD_MS, return to "rest" position
  if (servoActive && (millis() - servoStartMs >= SERVO_HOLD_MS)) {
    triggerServo.write(SERVO_MAX);
    servoActive = false;
    Serial.println("Servo: back to " + String(SERVO_MAX) + "° (rest)");
  }
}

void setup() {
  randomSeed(
    analogRead(A0) ^ analogRead(A1) ^ analogRead(A2) ^
    analogRead(A3) ^ analogRead(A4) ^ analogRead(A5)
  );

  // OUT pin + LED
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, outPinState);   // start OFF (LOW)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Serial
  Serial.begin(9600);
  Serial.println(F("Ready. Pulsing OUT only after audio playback actually starts."));

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Ultrasonic
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  // Servo
  // Note: On ATmega328P boards the Servo library uses Timer1.
  // Using D10 is fine here. It also keeps hardware SS as OUTPUT for SPI stability.
  triggerServo.attach(SERVO_PIN);
  triggerServo.write(50);  // Start at 50° on boot
  Serial.println(F("Servo initialized to 50°"));


  // Audio
  if (!musicPlayer.begin()) {
    Serial.println(F("VS1053 not found. Check wiring/shield."));
    while (1) { delay(10); }
  }
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD init failed. Check card/format/CS pin."));
    while (1) { delay(10); }
  }
  musicPlayer.setVolume(20, 20);
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);
  Serial.println(F("Music Maker ready."));

  triggerFileAvailable = SD.exists(TRIGGER_FILE);
  if (triggerFileAvailable) {
    Serial.println(F("TRIGGER.MP3 detected on SD."));
  } else {
    Serial.println(F("TRIGGER.MP3 not found. Random MP3 fallback enabled."));
  }
}

void handleButton() {
  bool level = digitalRead(BUTTON_PIN); // HIGH=released, LOW=pressed
  unsigned long now = millis();

  switch (btnState) {
    case BTN_IDLE:
      if (level == LOW) { btnState = BTN_DEBOUNCE; btnT0 = now; }
      break;

    case BTN_DEBOUNCE:
      if (now - btnT0 >= DEBOUNCE_MS) {
        btnState = (level == LOW) ? BTN_PRESSED : BTN_IDLE;
        btnT0 = now;
      }
      break;

    case BTN_PRESSED:
      if (level == HIGH) {
        outPinState = !outPinState;
        digitalWrite(OUT_PIN, outPinState);
        Serial.print(F("OUT_PIN toggled manually: "));
        Serial.println(outPinState ? F("ON") : F("OFF"));
        btnState = BTN_IDLE;
      } else if (now - btnT0 >= LONG_PRESS_MS) {
        btnState = BTN_LONGFIRE;
        Serial.println(F("Long press → sequence"));
        if (!seqActive) startSequence();
      }
      break;

    case BTN_LONGFIRE:
      if (level == HIGH) { btnState = BTN_IDLE; }
      break;
  }
  lastBtnLevel = level;
}

void handleSensors() {
  if (seqActive) return;

  static unsigned long lastPingMs = 0;
  static float d1 = -1.0f, d2 = -1.0f;

  unsigned long now = millis();
  if (now - lastPingMs >= PING_GAP_MS) {
    d1 = measureDistanceCm(TRIG1_PIN, ECHO1_PIN);
    delayMicroseconds(1500);
    d2 = measureDistanceCm(TRIG2_PIN, ECHO2_PIN);
    lastPingMs = now;
  }

  bool inRange = ( (d1 > 0 && d1 < TRIGGER_CM) || (d2 > 0 && d2 < TRIGGER_CM) );
  static bool wasInRange = false;

  if (inRange && !wasInRange && (now - lastTriggerPrintMs >= MUTE_INTERVAL_MS)) {
    Serial.println(F("TRIGGERED (edge)"));
    digitalWrite(LED_BUILTIN, HIGH);

    // Kick the servo immediately on edge
    triggerServoMove();

    if (!musicPlayer.playingMusic) {
      if (!playTriggerOrRandom()) {
        Serial.println(F("No audio track could be started."));
      }
    } else {
      Serial.println(F("Already playing, skipping OUT pulse to keep sync"));
    }
    lastTriggerPrintMs = now;
  }

  digitalWrite(LED_BUILTIN, inRange ? HIGH : LOW);
  wasInRange = inRange;
}

void handleSerialTrigger() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // read until newline

    // Trim whitespace (useful if Serial Monitor sends \r\n)
    input.trim();

    // If the input line is empty, trigger manually
    if (input.length() == 0) {
      Serial.println(F("Serial trigger received (empty line)"));

      // Kick the servo
      triggerServoMove();

      // Play TRIGGER.MP3 if not already playing
      if (!musicPlayer.playingMusic) {
        if (!playTriggerOrRandom()) {
          Serial.println(F("Serial trigger: no audio track available."));
        }
      } else {
        Serial.println(F("Already playing, skipping OUT pulse to keep sync"));
      }
    }
  }
}


void loop() {
  handleButton();               // short/long press handling
  handleSensors();              // ultrasonic triggers
  advanceSequenceIfNeeded();    // step through sequence when tracks finish
  servicePulseArm();            // fire OUT pulse when playback actually starts
  serviceOutPulse();            // end the 1s OUT pulse
  serviceServo();               // return servo to 0° after hold
  handleSerialTrigger();  // NEW: listen for empty-line triggers
  delay(10);
}
