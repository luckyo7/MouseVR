#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

// ESP32-C3 SuperMini -> MouseVR driver board. SPI, DC, RST and BL pins come
// from the build flags in platformio.ini. Both panels hang off the same bus
// and differ only in chip select: J2 = CS1 = right, J3 = CS2 = left. TFT_CS is
// -1 so TFT_eSPI never drives CS; selectPanel() below does it instead.
//
// Test image: a checkerboard centred on the screen. The right panel shows it
// as-is. The left panel shows it flipped horizontally (the panel is mounted
// mirrored) and run through a spherical warp, so the two can be compared side
// by side to judge what lens correction is needed. After a still black and
// white first frame, loop() scrolls the board diagonally and cycles its two
// squares through complementary hues, so bus errors show up as tearing,
// shifted rows or off colours.

TFT_eSPI tft = TFT_eSPI();

namespace {

constexpr int16_t  CX     = TFT_WIDTH / 2;
constexpr int16_t  CY     = TFT_HEIGHT / 2;
constexpr float    CELL   = 5.0f;     // checker square size in pixels (32 across)
constexpr float    SPHERE = 1.0f;     // 0 = no warp, 1 = full hemisphere

// Pattern coordinates are fixed point, 1/FIX pixel, so the per-frame render is
// integer only. The board repeats every PERIOD, and BIAS (a whole number of
// periods) keeps every coordinate positive so plain division floors.
constexpr int32_t  FIX      = 16;
constexpr int32_t  CELL_FIX = (int32_t)(CELL * FIX);
constexpr int32_t  PERIOD   = 2 * CELL_FIX;
constexpr int32_t  BIAS     = PERIOD * 1024;

// Animation, per frame: scroll in 1/FIX pixels and hue step in degrees.
constexpr int32_t  STEP_X   = FIX;         // 1 px right
constexpr int32_t  STEP_Y   = FIX / 2;     // 0.5 px down
constexpr uint16_t HUE_STEP = 3;
constexpr int      ROWS     = 8;           // rows per pushImage, divides TFT_HEIGHT

// Bit flags so BOTH broadcasts the same bytes to both panels at once, which
// is how init and the first clear are done (RST is shared anyway).
enum Panel : uint8_t { NONE = 0, RIGHT = 1, LEFT = 2, BOTH = RIGHT | LEFT };

void selectPanel(Panel p) {
  // Release both first so a switch never briefly has two panels listening.
  digitalWrite(TFT_CS1, HIGH);
  digitalWrite(TFT_CS2, HIGH);
  if (p & RIGHT) digitalWrite(TFT_CS1, LOW);
  if (p & LEFT)  digitalWrite(TFT_CS2, LOW);
}

// --- BL_HIZ_TEST -------------------------------------------------------
// Diagnostic: leave TFT_BL high-impedance so R8 (10K, BL_BASE -> 3V3) is the
// only thing driving Q1's base. R9 (the 10K pull-down) is DNP, so with GPIO0
// released the backlight should come on by itself. If it stays dark, the
// GPIO0/LEDC drive path is exonerated and the fault is Q1 or downstream.
// Set to 0 to restore normal backlight control.
#define BL_HIZ_TEST 0
// -----------------------------------------------------------------------

// TFT_BL drives Q1's base through the 1K/10K/10K network, so PWM dims it.
void setBacklight(uint8_t level) {
#if BL_HIZ_TEST
  (void)level;                 // never drive the pin during the test
#else
  analogWrite(TFT_BL, level);
#endif
}

void fadeBacklight(uint8_t from, uint8_t to, uint16_t ms) {
  constexpr uint8_t steps = 32;
  for (uint8_t i = 0; i <= steps; i++) {
    setBacklight(from + ((int16_t)to - from) * i / steps);
    delay(ms / steps);
  }
}

// Spherical warp: treat the screen as looking straight down on a hemisphere
// that fills the round panel, with the pattern wrapped over its surface by arc
// length. A point at normalised radius r (1 = panel edge) sits at polar angle
// asin(r), so it samples the flat pattern at radius asin(r) / (pi/2). The
// centre is magnified by pi/2 and the edge compressed. SPHERE blends between
// that and the unwarped radius. Outside the circle nothing is changed.
void sphereWarp(float &u, float &v) {
  const float R = TFT_WIDTH / 2.0f;
  float r = sqrtf(u * u + v * v) / R;
  if (r <= 0.0f || r >= 1.0f) return;
  float rs    = asinf(r) / (float)M_PI_2;
  float scale = 1.0f + SPHERE * (rs / r - 1.0f);
  u *= scale;
  v *= scale;
}

// Left panel's pattern coordinates (mirrored + warped) for every pixel, in
// 1/FIX pixels from the screen centre. Built once (100 KB) so animating never
// repeats the asinf per pixel.
int16_t *warpU = nullptr;
int16_t *warpV = nullptr;

bool buildWarpMap() {
  const size_t n = (size_t)TFT_WIDTH * TFT_HEIGHT;
  warpU = (int16_t *)malloc(n * sizeof(int16_t));
  warpV = (int16_t *)malloc(n * sizeof(int16_t));
  if (!warpU || !warpV) return false;
  for (int y = 0; y < TFT_HEIGHT; y++) {
    for (int x = 0; x < TFT_WIDTH; x++) {
      float su = -(x + 0.5f - TFT_WIDTH / 2.0f);
      float sv = y + 0.5f - TFT_HEIGHT / 2.0f;
      sphereWarp(su, sv);
      warpU[y * TFT_WIDTH + x] = (int16_t)lroundf(su * FIX);
      warpV[y * TFT_WIDTH + x] = (int16_t)lroundf(sv * FIX);
    }
  }
  return true;
}

// Fully saturated hue, 0-359 degrees, as RGB565.
uint16_t hue565(uint16_t h) {
  h %= 360;
  uint8_t x = (h % 60) * 255 / 60;
  uint8_t r, g, b;
  switch (h / 60) {
    case 0:  r = 255;     g = x;       b = 0;       break;
    case 1:  r = 255 - x; g = 255;     b = 0;       break;
    case 2:  r = 0;       g = 255;     b = x;       break;
    case 3:  r = 0;       g = 255 - x; b = 255;     break;
    case 4:  r = x;       g = 0;       b = 255;     break;
    default: r = 255;     g = 0;       b = 255 - x; break;
  }
  return tft.color565(r, g, b);
}

// Render one panel ROWS lines at a time. The pattern is shifted by (ox, oy)
// in 1/FIX pixels; colour a fills the square touching the centre from the
// bottom right, b its neighbours. Pixel-centre coordinates put the centre on
// a corner where four squares meet.
void drawFrame(Panel p, int32_t ox, int32_t oy, uint16_t a, uint16_t b) {
  static uint16_t block[TFT_WIDTH * ROWS];
  const bool warped = (p == LEFT);
  selectPanel(p);
  for (int y0 = 0; y0 < TFT_HEIGHT; y0 += ROWS) {
    uint16_t *px = block;
    for (int y = y0; y < y0 + ROWS; y++) {
      for (int x = 0; x < TFT_WIDTH; x++) {
        int32_t su, sv;
        if (warped) {
          su = warpU[y * TFT_WIDTH + x];
          sv = warpV[y * TFT_WIDTH + x];
        } else {
          su = (2 * x + 1 - TFT_WIDTH) * FIX / 2;
          sv = (2 * y + 1 - TFT_HEIGHT) * FIX / 2;
        }
        int32_t cu = (su - ox + BIAS) / CELL_FIX;
        int32_t cv = (sv - oy + BIAS) / CELL_FIX;
        *px++ = ((cu + cv) & 1) ? b : a;
      }
    }
    tft.pushImage(0, y0, TFT_WIDTH, ROWS, block);
  }
  selectPanel(NONE);
}

// --- PIN_WALK_TEST -----------------------------------------------------
// Diagnostic: skip the display and walk a single high level across the
// signal pins, 4 s each, so every GPIO -> cable -> J1 -> R -> FPC path can be
// checked live with a multimeter at the panel side of R1-R6 (or FPC pins
// 7-11). The pin named on serial should read ~3.3 V and all others ~0 V.
// Set to 0 to restore normal operation.
#define PIN_WALK_TEST 0

#if PIN_WALK_TEST
struct WalkPin { uint8_t gpio; const char *name; };
const WalkPin WALK_PINS[] = {
  {TFT_DC,   "DC   J1-7  R1  FPC-7"},
  {TFT_CS1,  "CS1  J1-9  R2  J2 FPC-8"},
  {TFT_CS2,  "CS2  J1-11 R3  J3 FPC-8"},
  {TFT_SCLK, "CLK  J1-13 R4  FPC-9"},
  {TFT_MOSI, "DIN  J1-15 R5  FPC-10"},
  {TFT_RST,  "RST  J1-17 R6  FPC-11"},
};

[[noreturn]] void pinWalk() {
  for (const WalkPin &p : WALK_PINS) {
    pinMode(p.gpio, OUTPUT);
    digitalWrite(p.gpio, LOW);
  }
  for (;;) {
    for (const WalkPin &p : WALK_PINS) {
      Serial.printf("[walk] GPIO%-2u HIGH  %s\n", p.gpio, p.name);
      digitalWrite(p.gpio, HIGH);
      delay(4000);
      digitalWrite(p.gpio, LOW);
    }
    Serial.println("[walk] all LOW");
    delay(4000);
  }
}
#endif
// -----------------------------------------------------------------------

}  // namespace

void setup() {
#if PIN_WALK_TEST
  Serial.begin(115200);
  delay(500);
  pinWalk();
#endif

  // Backlight off and both panels selected before the bus is touched: init
  // and the first clear are broadcast so both come up identically, and the
  // power-on garbage in display RAM is never lit.
#if BL_HIZ_TEST
  pinMode(TFT_BL, INPUT);      // hi-Z: R8 alone should switch Q1 on
#else
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
#endif
  pinMode(TFT_CS1, OUTPUT);
  pinMode(TFT_CS2, OUTPUT);
  selectPanel(BOTH);

  Serial.begin(115200);
  delay(500);
  Serial.println("[boot] ESP32-C3 SuperMini, dual panel");

  tft.init();
  tft.setRotation(0);
  selectPanel(NONE);
  Serial.println("[boot] init broadcast to CS1 and CS2");

  uint32_t t0 = millis();
  if (!buildWarpMap()) {
    Serial.println("[boot] warp map allocation failed");
    for (;;) delay(1000);
  }
  uint32_t t1 = millis();
  tft.setSwapBytes(true);
  drawFrame(RIGHT, 0, 0, TFT_WHITE, TFT_BLACK);
  drawFrame(LEFT, 0, 0, TFT_WHITE, TFT_BLACK);
  uint32_t t2 = millis();
  Serial.printf("[boot] warp map %lu ms, first frame (both panels) %lu ms\n",
                (unsigned long)(t1 - t0), (unsigned long)(t2 - t1));

  fadeBacklight(0, 255, 300);
  delay(1000);                 // hold the still frame before animating
  Serial.println("[boot] ready, animating");
}

void loop() {
  static uint32_t frame = 0;
  static uint32_t statFrame = 0;
  static uint32_t statTime = millis();

  int32_t  ox = (int32_t)(frame * STEP_X % PERIOD);
  int32_t  oy = (int32_t)(frame * STEP_Y % PERIOD);
  uint16_t h  = frame * HUE_STEP % 360;
  uint16_t a  = hue565(h);
  uint16_t b  = hue565(h + 180);
  drawFrame(RIGHT, ox, oy, a, b);
  drawFrame(LEFT, ox, oy, a, b);
  frame++;

  uint32_t now = millis();
  if (now - statTime >= 2000) {
    Serial.printf("[anim] %.1f fps (both panels per frame)\n",
                  (frame - statFrame) * 1000.0f / (now - statTime));
    statFrame = frame;
    statTime  = now;
  }
}
