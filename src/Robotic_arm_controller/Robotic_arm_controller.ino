#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
static const uint8_t OLED_ADDR = 0x3C;

// ESP32-S3 I2C pins
static const uint8_t I2C_SDA = 8;
static const uint8_t I2C_SCL = 9;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- Inputs ----------
static const uint8_t BTN_SELECT  = 4;  // change joint
static const uint8_t BTN_CONFIRM = 6;  // confirm
static const uint8_t KNOB_PIN    = 5;  // pot ADC

// ---------- Timing / Filtering ----------
static const uint32_t DEBOUNCE_MS = 25;   // slightly tighter for better feel
static const uint32_t UI_MIN_MS   = 200;  // don't redraw faster than this
static const int POT_DEADBAND_DEG = 2;    // ignore tiny noise in degrees

// ---------- Debounce struct ----------
struct DebounceBtn {
  uint8_t pin;
  bool raw;             // last raw read
  bool stable;          // debounced state
  uint32_t lastFlipMs;  // last time raw changed
};

DebounceBtn btnSel  {BTN_SELECT,  HIGH, HIGH, 0};
DebounceBtn btnConf {BTN_CONFIRM, HIGH, HIGH, 0};

// ---------- State ----------
uint8_t joint = 1;
int previewAngle = 90;
int confirmedAngle[3] = {0, 90, 90}; // [1..2]

// UI caching to reduce redraws
uint8_t lastUiJoint = 255;
int lastUiPreview = -999;
int lastUiConfirmed = -999;
uint32_t lastUiDrawMs = 0;

// ---------- Helpers ----------
int potToAngle(int potValue) {
  potValue = constrain(potValue, 0, 4095);
  return map(potValue, 0, 4095, 0, 180);
}

// Debounce update + press event detect (active LOW)
// Returns true exactly once per press.
bool updateButtonAndGetPress(DebounceBtn &b) {
  bool reading = digitalRead(b.pin);

  if (reading != b.raw) {
    b.raw = reading;
    b.lastFlipMs = millis();
  }

  // Wait until stable long enough
  if (millis() - b.lastFlipMs >= DEBOUNCE_MS) {
    if (b.stable != b.raw) {
      b.stable = b.raw;

      // active LOW press event
      if (b.stable == LOW) return true;
    }
  }

  return false;
}

void drawScreen(uint8_t jointSel, int preview, int confirmed) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("Joint: J");
  display.println(jointSel);

  display.print("Confirmed: ");
  display.print(confirmed);
  display.println(" deg");

  display.print("Preview:   ");
  display.print(preview);
  display.println(" deg");

  display.println();
  display.println("SEL: change joint");
  display.println("OK : confirm angle");

  display.display();
}

// Request UI update, but rate-limit it and only draw if values changed
void maybeUpdateUI(bool force = false) {
  uint32_t now = millis();

  // draw only if something changed OR forced, and not too frequently
  bool changed =
    (joint != lastUiJoint) ||
    (previewAngle != lastUiPreview) ||
    (confirmedAngle[joint] != lastUiConfirmed);

  if ((force || changed) && (now - lastUiDrawMs >= UI_MIN_MS)) {
    drawScreen(joint, previewAngle, confirmedAngle[joint]);
    lastUiJoint = joint;
    lastUiPreview = previewAngle;
    lastUiConfirmed = confirmedAngle[joint];
    lastUiDrawMs = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT_PULLUP);

  analogReadResolution(12);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("Initializing OLED...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed. Try 0x3D or check wiring.");
    while (true) { delay(1000); }
  }

  // Initial UI
  drawScreen(joint, previewAngle, confirmedAngle[joint]);
  lastUiJoint = joint;
  lastUiPreview = previewAngle;
  lastUiConfirmed = confirmedAngle[joint];
  lastUiDrawMs = millis();

  Serial.println("Ready.");
}

void loop() {
  // ---- 1) Read buttons early (fast path) ----
  bool selPressed  = updateButtonAndGetPress(btnSel);
  bool confPressed = updateButtonAndGetPress(btnConf);

  // ---- 2) Update state from button events ----
  if (selPressed) {
    joint = (joint == 1) ? 2 : 1;
    Serial.print("Selected J"); Serial.println(joint);
    maybeUpdateUI(true); // forced update (still rate-limited)
  }

  if (confPressed) {
    confirmedAngle[joint] = previewAngle;
    Serial.print("Confirmed J"); Serial.print(joint);
    Serial.print(" = "); Serial.println(confirmedAngle[joint]);
    maybeUpdateUI(true); // forced update (still rate-limited)
  }

  // ---- 3) Read pot (and filter noise) ----
  int newPreview = potToAngle(analogRead(KNOB_PIN));

  // Only accept if changed enough to matter
  if (abs(newPreview - previewAngle) >= POT_DEADBAND_DEG) {
    previewAngle = newPreview;
    // UI update requested, not forced (rate-limited)
    maybeUpdateUI(false);
  }

  // If nothing changes, loop remains fast and buttons stay responsive
}
