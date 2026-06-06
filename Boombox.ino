#include <Arduino.h>

#include "BiquadFwd.h"
#include <FastLED.h>
#include <math.h>
#include <string.h>

#define BOOMBOX_BUILD_ID (__DATE__ " " __TIME__)

// A2DP PCM path:
// - Legacy callback `esp_a2d_sink_register_data_callback()` should deliver decoded PCM on many builds.
// - The newer `esp_a2d_sink_register_audio_data_callback()` delivers *undecoded* frames and needs a decoder.
// In this project we want PCM, so default to legacy.
#define BOOMBOX_USE_LEGACY_A2DP_PCM_CB 1

#if defined(ARDUINO_ARCH_ESP32)
  #include <driver/i2s_std.h>
  #include <driver/adc.h>
  #include <nvs_flash.h>
  #include <esp_err.h>
  #include <esp32-hal-bt.h>
  // Prevent Arduino core from releasing BT memory at startup (needed when using IDF BT APIs directly).
  #include <esp32-hal-bt-mem.h>
  #include <esp_bt.h>
  #include <esp_bt_main.h>
  #include <esp_bt_device.h>
  #include <esp_gap_bt_api.h>
  #include <esp_a2dp_api.h>
  #include <esp_system.h>
  #include <Preferences.h>
  #include <soc/soc_caps.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/ringbuf.h>
  #include <freertos/task.h>
#else
  #error "Dieses Sketch ist für ESP32 gedacht."
#endif

// -----------------------------
// Pins (aktuelles Mapping)
// -----------------------------
namespace Pins {
  // MAX98357A I2S
  constexpr uint8_t I2S_DOUT = 32; // DIN
  constexpr uint8_t I2S_LRC  = 26; // LRC / WS
  constexpr uint8_t I2S_BCLK = 27; // BCLK

  // MAX98357A SD / SD_MODE (beide Verstärker SD zusammen)
  // Hardware-Fix (wichtig): 10k Pulldown SD -> GND, damit der Amp beim Einschalten sicher stumm ist.
  constexpr uint8_t AMP_SD = 17;

  // Action Button gegen GND (interner Pullup)
  constexpr uint8_t ACTION_BUTTON = 16;

  // Potentiometer (Wiper / Pin 2)
  // Pin 1 = GND (gemeinsam), Pin 3 = 3.3V (gemeinsam)
  constexpr uint8_t POT_BASS   = 33;
  constexpr uint8_t POT_TREBLE = 34;
  constexpr uint8_t POT_VOLUME = 35;

  // WS2812B LED Data In (mit 470Ω in Serie)
  constexpr uint8_t LED_DATA = 23;
}

// MAX98357A: meistens SD=LOW => Shutdown/Mute, SD=HIGH => aktiv.
// Falls bei dir invertiert: tausche die beiden Level.
constexpr uint8_t kAmpMuteLevel = LOW;
constexpr uint8_t kAmpUnmuteLevel = HIGH;

static void ampMuteRaw(bool mute) {
  digitalWrite(Pins::AMP_SD, mute ? kAmpMuteLevel : kAmpUnmuteLevel);
}

// -----------------------------
// State
// -----------------------------
static volatile bool g_streaming = false;
static volatile bool g_connected = false;
static esp_bd_addr_t g_remote_bda = {0};
static volatile bool g_have_remote_bda = false;

static volatile uint32_t g_sample_rate = 44100;
static volatile bool g_i2s_ready = false;

static volatile bool g_playing_fx = false;

enum class DisplayMode : uint8_t {
  VisualizerA = 0,
  Potis       = 1,
  VisualizerC = 2,
  Off         = 3,
};

static DisplayMode g_display_mode = DisplayMode::VisualizerA;

// Auto-Poti-Anzeige: Modus vor dem Auto-Wechsel und Timer
static DisplayMode g_poti_return_mode = DisplayMode::VisualizerA;
static uint32_t    g_poti_auto_until  = 0; // millis()-Zeitpunkt für Revert (0 = inaktiv)

// Intro text scroller
static bool g_scroll_active = true;
static int16_t g_scroll_x = 0;
static uint32_t g_scroll_last_ms = 0;
static char g_scroll_text[256] = {0};

// LED mapping preset (persisted)
static Preferences g_prefs;
static uint8_t g_map_preset = 0;

// One-shot button events (consumed in loop)
static volatile bool g_btn_short_event = false;
static volatile bool g_btn_long_event = false;

// Potis (0..1), geglättet
static volatile float g_vol01 = 0.65f;
static volatile float g_bass01 = 0.50f;
static volatile float g_treble01 = 0.50f;

// Audio level (für LEDs)
static volatile float g_audio_peak01 = 0.0f;
// 8 Frequenzbänder für den Balken-Visualizer (von i2sWriterTask befüllt)
static volatile float g_bands[8] = {};
static volatile bool g_using_audio_buf_cb = true;

// Master volume hard limit (fixed): 0..100 (%)
constexpr float kMaxVolumePercent = 70.0f;
constexpr float kMaxVolume01 = kMaxVolumePercent / 100.0f;

// -----------------------------
// I2S
// -----------------------------
static i2s_chan_handle_t g_i2s_tx = nullptr;

// PCM buffering between BT stack task and I2S writer task (prevents dropouts/knacks).
static RingbufHandle_t g_pcm_rb = nullptr;
static TaskHandle_t g_i2s_task = nullptr;
static volatile uint32_t g_rb_drops = 0;
static volatile uint32_t g_rb_bytes_in = 0;
static volatile uint32_t g_rb_bytes_out = 0;

// Debug / safety for A2DP PCM callback
static volatile uintptr_t g_last_pcm_ptr = 0;
static volatile uint32_t g_last_pcm_len = 0;
static volatile uint32_t g_pcm_invalid = 0;
static volatile uint32_t g_pcm_enq_fail = 0;
static volatile uint32_t g_pcm_reject_printed = 0;
static volatile uint32_t g_pcm_dbg_remaining = 0;

static inline bool isLikelyReadablePtr(const void* p) {
  // Conservative range checks for ESP32 address space.
  // DRAM typically: 0x3FFB0000..0x40000000
  // IRAM typically: 0x40080000..0x400A0000
  const uintptr_t u = (uintptr_t)p;
  if (u >= 0x3FFB0000UL && u < 0x40000000UL) return true;
  if (u >= 0x40080000UL && u < 0x400A0000UL) return true;
  return false;
}

// -----------------------------
// LEDs
// -----------------------------
constexpr uint16_t kNumLeds = 128;        // 2x8x8 Panels in Reihe
constexpr uint8_t  kLedBrightness = 15;   // Safety: niedrig starten
CRGB g_leds[kNumLeds];
static uint32_t g_last_led_ms = 0;

// LED matrix mapping:
// - Two 8x8 panels chained -> treated as one 16x8 display (panel0 = x 0..7, panel1 = x 8..15)
// - Both panels are physically rotated 180° -> compensate in software
constexpr uint8_t kMatrixW = 16;
constexpr uint8_t kMatrixH = 8;
static bool g_serpentine = true;          // common for 8x8 WS2812 panels
static bool g_rotate180_panels = true;    // fixed: always rotate 180° (per your physical panel orientation)
static bool g_swap_panels = false;        // if the first panel in the chain is physically on the right

static void applyMapPreset(uint8_t preset) {
  // Preset bits (rotate is FIXED to 180°):
  // - bit0: serpentine (1) / progressive (0)
  // - bit1: swap panels (1) / normal order (0)
  preset &= 3;
  g_map_preset = preset;
  g_serpentine = (preset & 0x01) != 0;
  g_rotate180_panels = true;
  g_swap_panels = (preset & 0x02) != 0;
  Serial.printf("LED map preset=%u (serp=%u rot180=1 swap=%u)\n",
                (unsigned)g_map_preset,
                (unsigned)g_serpentine,
                (unsigned)g_swap_panels);
}

static inline uint16_t xyToIndex(uint8_t x, uint8_t y) {
  if (x >= kMatrixW || y >= kMatrixH) return 0;
  uint8_t panel = x / 8;
  // IMPORTANT:
  // Rotating the content by 180° as a whole implies that the two panels swap sides.
  // If we rotate each 8x8 panel, we must also swap left/right to keep the global image correct.
  if (g_swap_panels ^ g_rotate180_panels) panel = 1 - panel;
  uint8_t px = x % 8;
  uint8_t py = y;

  if (g_rotate180_panels) {
    px = 7 - px;
    py = 7 - py;
  }

  uint16_t local;
  if (g_serpentine) {
    local = (py & 1) ? (py * 8 + (7 - px)) : (py * 8 + px);
  } else {
    local = py * 8 + px;
  }
  return (uint16_t)panel * 64 + local;
}

static inline void setXY(uint8_t x, uint8_t y, const CRGB& c) {
  const uint16_t idx = xyToIndex(x, y);
  if (idx < kNumLeds) g_leds[idx] = c;
}

static inline void fadeAll(uint8_t amount) {
  for (uint16_t i = 0; i < kNumLeds; i++) {
    g_leds[i].fadeToBlackBy(amount);
  }
}

static void scrollSetMessage(const char* msg) {
  if (!msg) msg = "";
  memset(g_scroll_text, 0, sizeof(g_scroll_text));

  // Sanitize to uppercase ASCII-ish. Replace a few common UTF-8 umlauts with AE/OE/UE/SS.
  const uint8_t* p = (const uint8_t*)msg;
  size_t out = 0;
  while (*p && out + 4 < sizeof(g_scroll_text)) {
    if (p[0] == 0xC3 && p[1] == 0x84) { g_scroll_text[out++] = 'A'; g_scroll_text[out++] = 'E'; p += 2; continue; } // Ä
    if (p[0] == 0xC3 && p[1] == 0x96) { g_scroll_text[out++] = 'O'; g_scroll_text[out++] = 'E'; p += 2; continue; } // Ö
    if (p[0] == 0xC3 && p[1] == 0x9C) { g_scroll_text[out++] = 'U'; g_scroll_text[out++] = 'E'; p += 2; continue; } // Ü
    if (p[0] == 0xC3 && p[1] == 0xA4) { g_scroll_text[out++] = 'A'; g_scroll_text[out++] = 'E'; p += 2; continue; } // ä
    if (p[0] == 0xC3 && p[1] == 0xB6) { g_scroll_text[out++] = 'O'; g_scroll_text[out++] = 'E'; p += 2; continue; } // ö
    if (p[0] == 0xC3 && p[1] == 0xBC) { g_scroll_text[out++] = 'U'; g_scroll_text[out++] = 'E'; p += 2; continue; } // ü
    if (p[0] == 0xC3 && p[1] == 0x9F) { g_scroll_text[out++] = 'S'; g_scroll_text[out++] = 'S'; p += 2; continue; } // ß

    char c = (char)(*p++);
    if ((uint8_t)c < 32) continue;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    g_scroll_text[out++] = c;
  }
  g_scroll_text[out] = 0;

  g_scroll_active = true;
  g_scroll_x = kMatrixW;
  g_scroll_last_ms = 0;
}

// Very small 3x5 font (uppercase + digits + a few punctuation), stored as 3 columns of 5 bits (LSB = top).
static inline void glyph3x5(char c, uint8_t colBits[3]) {
  colBits[0] = colBits[1] = colBits[2] = 0;
  switch (c) {
    case ' ': colBits[0]=0; colBits[1]=0; colBits[2]=0; break;
    case '!': colBits[0]=0b11101; colBits[1]=0; colBits[2]=0; break;
    case '.': colBits[0]=0; colBits[1]=0; colBits[2]=0b10000; break;
    case ',': colBits[0]=0; colBits[1]=0; colBits[2]=0b11000; break;
    case ':': colBits[0]=0; colBits[1]=0b01010; colBits[2]=0; break;
    case '-': colBits[0]=0; colBits[1]=0b00100; colBits[2]=0; break;
    case '?': colBits[0]=0b00010; colBits[1]=0b10101; colBits[2]=0b00100; break;
    case '\'': colBits[0]=0b00011; colBits[1]=0; colBits[2]=0; break;
    case '"': colBits[0]=0b00011; colBits[1]=0; colBits[2]=0b00011; break;

    case '0': colBits[0]=0b11111; colBits[1]=0b10001; colBits[2]=0b11111; break;
    case '1': colBits[0]=0;       colBits[1]=0b11111; colBits[2]=0;       break;
    case '2': colBits[0]=0b11101; colBits[1]=0b10101; colBits[2]=0b10111; break;
    case '3': colBits[0]=0b10101; colBits[1]=0b10101; colBits[2]=0b11111; break;
    case '4': colBits[0]=0b00111; colBits[1]=0b00100; colBits[2]=0b11111; break;
    case '5': colBits[0]=0b10111; colBits[1]=0b10101; colBits[2]=0b11101; break;
    case '6': colBits[0]=0b11111; colBits[1]=0b10101; colBits[2]=0b11101; break;
    case '7': colBits[0]=0b00001; colBits[1]=0b00001; colBits[2]=0b11111; break;
    case '8': colBits[0]=0b11111; colBits[1]=0b10101; colBits[2]=0b11111; break;
    case '9': colBits[0]=0b10111; colBits[1]=0b10101; colBits[2]=0b11111; break;

    case 'A': colBits[0]=0b11110; colBits[1]=0b00101; colBits[2]=0b11110; break;
    case 'B': colBits[0]=0b11111; colBits[1]=0b10101; colBits[2]=0b01010; break;
    case 'C': colBits[0]=0b01110; colBits[1]=0b10001; colBits[2]=0b10001; break;
    case 'D': colBits[0]=0b11111; colBits[1]=0b10001; colBits[2]=0b01110; break;
    case 'E': colBits[0]=0b11111; colBits[1]=0b10101; colBits[2]=0b10001; break;
    case 'F': colBits[0]=0b11111; colBits[1]=0b00101; colBits[2]=0b00001; break;
    case 'G': colBits[0]=0b01110; colBits[1]=0b10001; colBits[2]=0b11101; break;
    case 'H': colBits[0]=0b11111; colBits[1]=0b00100; colBits[2]=0b11111; break;
    case 'I': colBits[0]=0b10001; colBits[1]=0b11111; colBits[2]=0b10001; break;
    case 'J': colBits[0]=0b01000; colBits[1]=0b10000; colBits[2]=0b01111; break;
    case 'K': colBits[0]=0b11111; colBits[1]=0b00100; colBits[2]=0b11011; break;
    case 'L': colBits[0]=0b11111; colBits[1]=0b10000; colBits[2]=0b10000; break;
    case 'M': colBits[0]=0b11111; colBits[1]=0b00010; colBits[2]=0b11111; break;
    case 'N': colBits[0]=0b11111; colBits[1]=0b00010; colBits[2]=0b11100; break;
    case 'O': colBits[0]=0b01110; colBits[1]=0b10001; colBits[2]=0b01110; break;
    case 'P': colBits[0]=0b11111; colBits[1]=0b00101; colBits[2]=0b00010; break;
    case 'Q': colBits[0]=0b01110; colBits[1]=0b10001; colBits[2]=0b11110; break;
    case 'R': colBits[0]=0b11111; colBits[1]=0b00101; colBits[2]=0b11010; break;
    case 'S': colBits[0]=0b10010; colBits[1]=0b10101; colBits[2]=0b01001; break;
    case 'T': colBits[0]=0b00001; colBits[1]=0b11111; colBits[2]=0b00001; break;
    case 'U': colBits[0]=0b01111; colBits[1]=0b10000; colBits[2]=0b01111; break;
    case 'V': colBits[0]=0b00111; colBits[1]=0b11000; colBits[2]=0b00111; break;
    case 'W': colBits[0]=0b11111; colBits[1]=0b01000; colBits[2]=0b11111; break;
    case 'X': colBits[0]=0b11011; colBits[1]=0b00100; colBits[2]=0b11011; break;
    case 'Y': colBits[0]=0b00011; colBits[1]=0b11100; colBits[2]=0b00011; break;
    case 'Z': colBits[0]=0b11001; colBits[1]=0b10101; colBits[2]=0b10011; break;

    default: colBits[0]=0b11111; colBits[1]=0b10101; colBits[2]=0b11111; break; // block
  }
}

static void drawChar3x5(int16_t x, int16_t y, char c, const CRGB& color) {
  uint8_t cols[3];
  glyph3x5(c, cols);
  for (uint8_t cx = 0; cx < 3; cx++) {
    const uint8_t bits = cols[cx];
    for (uint8_t cy = 0; cy < 5; cy++) {
      if (bits & (1U << cy)) {
        const int16_t px = x + (int16_t)cx;
        const int16_t py = y + (int16_t)cy;
        if (px >= 0 && px < kMatrixW && py >= 0 && py < kMatrixH) {
          setXY((uint8_t)px, (uint8_t)py, color);
        }
      }
    }
  }
}

static void displayNextMode() {
  const uint8_t cur = (uint8_t)g_display_mode;
  const uint8_t next = (uint8_t)((cur + 1) % 4);
  g_display_mode = (DisplayMode)next;
  g_poti_auto_until = 0; // manuell gewechselt → kein Auto-Revert
  Serial.printf("Display mode: %u\n", (unsigned)next);
}

static void ringbufFlush() {
  if (g_pcm_rb == nullptr) return;
  while (true) {
    size_t itemSize = 0;
    void* item = xRingbufferReceive(g_pcm_rb, &itemSize, 0);
    if (!item) break;
    vRingbufferReturnItem(g_pcm_rb, item);
  }
}

static bool ringbufSendBlocking(const void* data, size_t len, TickType_t timeoutTicks) {
  if (g_pcm_rb == nullptr) return false;
  const TickType_t start = xTaskGetTickCount();
  while (true) {
    if (xRingbufferSend(g_pcm_rb, (void*)data, len, 0) == pdTRUE) return true;
    if (timeoutTicks == 0) return false;
    if ((xTaskGetTickCount() - start) >= timeoutTicks) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void ringbufEnqueueSilence(uint32_t ms) {
  if (!g_i2s_ready || g_pcm_rb == nullptr) return;
  constexpr size_t framesPerChunk = 256;
  static int16_t zeros[framesPerChunk * 2] = {0};

  const uint32_t totalFrames = (uint32_t)((uint64_t)g_sample_rate * ms / 1000ULL);
  uint32_t written = 0;
  while (written < totalFrames) {
    const uint32_t remaining = totalFrames - written;
    const size_t framesThis = (remaining < framesPerChunk) ? (size_t)remaining : framesPerChunk;
    ringbufSendBlocking(zeros, framesThis * 2 * sizeof(int16_t), pdMS_TO_TICKS(100));
    written += (uint32_t)framesThis;
  }
}

// -----------------------------
// Simple EQ (biquad shelves)
// -----------------------------
struct Biquad {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float z1L = 0.0f, z2L = 0.0f;
  float z1R = 0.0f, z2R = 0.0f;
};

static Biquad g_low_shelf;
static Biquad g_high_shelf;
static uint32_t g_eq_last_sr = 0;
static uint32_t g_eq_last_update_ms = 0;

static float dbFrom01(float x01, float maxAbsDb) {
  // 0..1 => -max..+max
  const float x = (x01 < 0.0f) ? 0.0f : (x01 > 1.0f ? 1.0f : x01);
  return (x * 2.0f - 1.0f) * maxAbsDb;
}

static void calcLowShelf(Biquad& bq, float fs, float f0, float gainDb, float slope = 1.0f) {
  // RBJ audio EQ cookbook
  const float A = powf(10.0f, gainDb / 40.0f);
  const float w0 = 2.0f * 3.1415926535f * f0 / fs;
  const float cs = cosf(w0);
  const float sn = sinf(w0);
  const float alpha = sn / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / slope - 1.0f) + 2.0f);
  const float beta = 2.0f * sqrtf(A) * alpha;

  float b0 =    A * ((A + 1) - (A - 1) * cs + beta);
  float b1 =  2*A * ((A - 1) - (A + 1) * cs);
  float b2 =    A * ((A + 1) - (A - 1) * cs - beta);
  float a0 =        (A + 1) + (A - 1) * cs + beta;
  float a1 =   -2 * ((A - 1) + (A + 1) * cs);
  float a2 =        (A + 1) + (A - 1) * cs - beta;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
}

static void calcHighShelf(Biquad& bq, float fs, float f0, float gainDb, float slope = 1.0f) {
  const float A = powf(10.0f, gainDb / 40.0f);
  const float w0 = 2.0f * 3.1415926535f * f0 / fs;
  const float cs = cosf(w0);
  const float sn = sinf(w0);
  const float alpha = sn / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / slope - 1.0f) + 2.0f);
  const float beta = 2.0f * sqrtf(A) * alpha;

  float b0 =    A * ((A + 1) + (A - 1) * cs + beta);
  float b1 = -2*A * ((A - 1) + (A + 1) * cs);
  float b2 =    A * ((A + 1) + (A - 1) * cs - beta);
  float a0 =        (A + 1) - (A - 1) * cs + beta;
  float a1 =    2 * ((A - 1) - (A + 1) * cs);
  float a2 =        (A + 1) - (A - 1) * cs - beta;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
}

static void eqUpdateIfNeeded() {
  const uint32_t now = millis();
  if (now - g_eq_last_update_ms < 80 && g_eq_last_sr == g_sample_rate) return;
  g_eq_last_update_ms = now;
  g_eq_last_sr = g_sample_rate;

  // bass/treble in dB
  const float bassDb = dbFrom01(g_bass01, 12.0f);
  const float trebleDb = dbFrom01(g_treble01, 12.0f);

  const float fs = (float)g_sample_rate;
  calcLowShelf(g_low_shelf, fs, 120.0f, bassDb, 0.8f);
  calcHighShelf(g_high_shelf, fs, 6000.0f, trebleDb, 0.8f);
}

static inline float biquadProcessMono(const Biquad& bq, float x, float& z1, float& z2) {
  const float y = bq.b0 * x + z1;
  z1 = bq.b1 * x - bq.a1 * y + z2;
  z2 = bq.b2 * x - bq.a2 * y;
  return y;
}

static inline int16_t clip16(float x) {
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)lroundf(x);
}

static void i2sWriterTask(void* /*arg*/) {
  // Give BT stack some room to start.
  vTaskDelay(pdMS_TO_TICKS(50));

  while (true) {
    if (!g_i2s_ready || g_pcm_rb == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    size_t itemSize = 0;
    void* item = xRingbufferReceive(g_pcm_rb, &itemSize, pdMS_TO_TICKS(50));
    if (!item) {
      // No audio data available => keep clocked silence (prevents random noise on some amps).
      i2sWriteSilence(20);
      continue;
    }

    eqUpdateIfNeeded();

    // Apply volume + simple EQ in the writer task (keeps BT callback lightweight).
    // Buffer is interleaved stereo int16_t.
    static int32_t out[1024]; // samples (L/R), not bytes
    const int16_t* in = (const int16_t*)item;
    const size_t samples = itemSize / sizeof(int16_t);
    constexpr size_t kOutSamples = sizeof(out) / sizeof(out[0]);
    if (samples > kOutSamples) {
      // Should not happen with RINGBUF_TYPE_NOSPLIT + our chunking, but keep it safe.
      g_rb_drops++;
      vRingbufferReturnItem(g_pcm_rb, item);
      continue;
    }

    const float vol = g_vol01;
    float peak = 0.0f;

    // IIR-Hüllkurvenfilter für 8 Frequenzbänder.
    // kLC[i] = exp(-2π*fc/44100) für fc = 80,160,320,640,1280,2560,5120 Hz.
    // Anwendung auf rectifiziertes Signal → Bandenergie-Hüllkurve.
    static float envLP[7] = {};
    static constexpr float kLC[7] = {
      0.9887f, 0.9775f, 0.9556f, 0.9140f, 0.8385f, 0.7121f, 0.5157f
    };
    // Verstärkung pro Band (höhere Bänder haben weniger Energie → mehr Gain)
    static constexpr float kBG[8] = {
      3.0f, 6.0f, 9.0f, 13.0f, 18.0f, 24.0f, 30.0f, 36.0f
    };
    float bandPk[8] = {};

    for (size_t i = 0; i < samples; i += 2) {
      float l = (float)in[i + 0];
      float r = (float)in[i + 1];

      // shelves
      l = biquadProcessMono(g_low_shelf, l, g_low_shelf.z1L, g_low_shelf.z2L);
      r = biquadProcessMono(g_low_shelf, r, g_low_shelf.z1R, g_low_shelf.z2R);
      l = biquadProcessMono(g_high_shelf, l, g_high_shelf.z1L, g_high_shelf.z2L);
      r = biquadProcessMono(g_high_shelf, r, g_high_shelf.z1R, g_high_shelf.z2R);

      // volume
      l *= vol;
      r *= vol;

      const float al = fabsf(l) / 32768.0f;
      const float ar = fabsf(r) / 32768.0f;
      if (al > peak) peak = al;
      if (ar > peak) peak = ar;

      // Bandanalyse: rectifiziertes Mono-Signal durch LP-Filterkaskade
      const float monoAbs = (al + ar) * 0.5f;
      for (int bi = 0; bi < 7; bi++) {
        envLP[bi] = envLP[bi] * kLC[bi] + monoAbs * (1.0f - kLC[bi]);
      }
      bandPk[0] = fmaxf(bandPk[0], envLP[0]);
      for (int bi = 1; bi < 7; bi++) {
        bandPk[bi] = fmaxf(bandPk[bi], fmaxf(0.0f, envLP[bi] - envLP[bi - 1]));
      }
      bandPk[7] = fmaxf(bandPk[7], fmaxf(0.0f, monoAbs - envLP[6]));

      // I2S is configured for 32-bit samples; place 16-bit PCM in the MSBs.
      out[i + 0] = ((int32_t)clip16(l)) << 16;
      out[i + 1] = ((int32_t)clip16(r)) << 16;
    }

    // Globale Bandpegel aktualisieren (schneller Anstieg, langsamer Abfall)
    for (int bi = 0; bi < 8; bi++) {
      const float sc = fminf(1.0f, bandPk[bi] * kBG[bi]);
      const float prev_b = g_bands[bi];
      g_bands[bi] = prev_b > sc ? prev_b * 0.80f : sc;
    }

    // smooth peak for LEDs
    const float prev = g_audio_peak01;
    g_audio_peak01 = prev * 0.85f + peak * 0.15f;

    // Write processed audio (blocking).
    size_t bytesWritten = 0;
    if (g_i2s_tx) {
      i2s_channel_write(g_i2s_tx, out, samples * sizeof(int32_t), &bytesWritten, portMAX_DELAY);
    }
    g_rb_bytes_out += (uint32_t)bytesWritten;
    vRingbufferReturnItem(g_pcm_rb, item);
  }
}

static void i2sInit(uint32_t sampleRate) {
  Serial.printf("I2S: init sr=%lu\n", (unsigned long)sampleRate);

  if (g_i2s_tx == nullptr) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // Use separate TX channel
    esp_err_t err = i2s_new_channel(&chan_cfg, &g_i2s_tx, nullptr);
    if (err != ESP_OK) {
      Serial.printf("I2S: new_channel failed: %s (%d)\n", esp_err_to_name(err), (int)err);
      return;
    }

    i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sampleRate),
      // 16-bit Philips I2S (klassisch) für MAX98357A.
      // Wichtig: Wenn slot_bit_width=32 gesetzt wird, erwartet der Treiber i.d.R. auch 32-bit Sample-Container.
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = static_cast<gpio_num_t>(Pins::I2S_BCLK),
        .ws = static_cast<gpio_num_t>(Pins::I2S_LRC),
        .dout = static_cast<gpio_num_t>(Pins::I2S_DOUT),
        .din = I2S_GPIO_UNUSED,
        .invert_flags = {
          .mclk_inv = false,
          .bclk_inv = false,
          .ws_inv = false,
        },
      },
    };

    err = i2s_channel_init_std_mode(g_i2s_tx, &std_cfg);
    if (err != ESP_OK) {
      Serial.printf("I2S: init_std_mode failed: %s (%d)\n", esp_err_to_name(err), (int)err);
      return;
    }

    err = i2s_channel_enable(g_i2s_tx);
    if (err != ESP_OK) {
      Serial.printf("I2S: channel_enable failed: %s (%d)\n", esp_err_to_name(err), (int)err);
      return;
    }
  } else {
    // reconfigure clock only
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sampleRate);
    i2s_channel_disable(g_i2s_tx);
    i2s_channel_reconfig_std_clock(g_i2s_tx, &clk_cfg);
    i2s_channel_enable(g_i2s_tx);
  }

  Serial.println("I2S: init done");
  g_i2s_ready = true;
}

static void i2sSetSampleRate(uint32_t sampleRate) {
  g_sample_rate = sampleRate;
  if (!g_i2s_ready) return;
  if (!g_i2s_tx) return;
  i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sampleRate);
  i2s_channel_disable(g_i2s_tx);
  i2s_channel_reconfig_std_clock(g_i2s_tx, &clk_cfg);
  i2s_channel_enable(g_i2s_tx);
}

static void i2sWriteSilence(uint32_t ms) {
  if (!g_i2s_ready) return;
  if (!g_i2s_tx) return;
  constexpr size_t framesPerChunk = 256;
  static int32_t chunk[framesPerChunk * 2];
  memset(chunk, 0, sizeof(chunk));

  const uint32_t totalFrames = (uint32_t)((uint64_t)g_sample_rate * ms / 1000ULL);
  uint32_t written = 0;
  while (written < totalFrames) {
    const uint32_t remaining = totalFrames - written;
    const size_t framesThis = (remaining < framesPerChunk) ? (size_t)remaining : framesPerChunk;
    size_t bytesWritten = 0;
    // Nicht endlos blockieren (falls I2S nicht sauber läuft): lieber abbrechen.
    i2s_channel_write(g_i2s_tx, chunk, framesThis * 2 * sizeof(int32_t), &bytesWritten, pdMS_TO_TICKS(20));
    if (bytesWritten == 0) break;
    written += (uint32_t)framesThis;
  }
}

static void playTone(float freqHz, uint32_t durationMs, float volume01) {
  if (!g_i2s_ready) return;
  if (g_pcm_rb == nullptr) return;
  if (volume01 < 0.0f) volume01 = 0.0f;
  if (volume01 > 1.0f) volume01 = 1.0f;

  constexpr size_t framesPerChunk = 256;
  static int16_t chunk[framesPerChunk * 2];

  const uint32_t totalFrames = (uint32_t)((uint64_t)g_sample_rate * durationMs / 1000ULL);
  const float maxAmp = 25000.0f * volume01;
  float phase = 0.0f;
  const float phaseInc = 2.0f * 3.1415926535f * freqHz / (float)g_sample_rate;
  const uint32_t rampFrames = (uint32_t)((uint64_t)g_sample_rate * 8 / 1000ULL); // 8ms fade

  uint32_t written = 0;
  while (written < totalFrames) {
    const uint32_t remaining = totalFrames - written;
    const size_t framesThis = (remaining < framesPerChunk) ? (size_t)remaining : framesPerChunk;

    for (size_t i = 0; i < framesThis; i++) {
      const uint32_t f = written + (uint32_t)i;
      float env = 1.0f;
      if (f < rampFrames) env = (float)f / (float)rampFrames;
      const uint32_t tail = totalFrames - f;
      if (tail < rampFrames) {
        const float e2 = (float)tail / (float)rampFrames;
        if (e2 < env) env = e2;
      }

      const int16_t s = (int16_t)(sinf(phase) * (maxAmp * env));
      chunk[i * 2 + 0] = s;
      chunk[i * 2 + 1] = s;
      phase += phaseInc;
      if (phase > 2.0f * 3.1415926535f) phase -= 2.0f * 3.1415926535f;
    }

    // IMPORTANT: Only the I2S writer task should touch the I2S channel.
    // Push tone PCM into ringbuffer instead.
    ringbufSendBlocking(chunk, framesThis * 2 * sizeof(int16_t), pdMS_TO_TICKS(50));
    written += (uint32_t)framesThis;
  }

  // tiny tail
  static int16_t zeros[framesPerChunk * 2] = {0};
  ringbufSendBlocking(zeros, 60 * 2 * sizeof(int16_t), pdMS_TO_TICKS(50));
}

static void ampApplyPolicy() {
  const bool shouldMute = !g_streaming || g_playing_fx;
  ampMuteRaw(shouldMute);
}

// -----------------------------
// Button handling (short: cycle LED display, long: reconnect+sound)
// -----------------------------
struct ButtonState {
  bool lastRaw = true; // pullup => true = UP
  bool stable = true;
  uint32_t lastEdgeMs = 0;

  bool pressActive = false;
  uint32_t pressStartMs = 0;
  bool longPressFired = false;
};

static ButtonState g_btn;
static volatile bool g_request_reconnect = false;

static void updateButton(uint32_t nowMs, uint32_t debounceMs = 25) {
  const bool raw = digitalRead(Pins::ACTION_BUTTON);
  if (raw != g_btn.lastRaw) {
    g_btn.lastRaw = raw;
    g_btn.lastEdgeMs = nowMs;
    Serial.printf("Button raw: %s\n", raw == LOW ? "DOWN" : "UP");
  }

  if ((nowMs - g_btn.lastEdgeMs) >= debounceMs && raw != g_btn.stable) {
    const bool prev = g_btn.stable;
    g_btn.stable = raw;

    // press
    if (prev == HIGH && g_btn.stable == LOW) {
      g_btn.pressActive = true;
      g_btn.pressStartMs = nowMs;
      g_btn.longPressFired = false;
      Serial.println("Button: press");
    }

    // release => short press
    if (prev == LOW && g_btn.stable == HIGH) {
      g_btn.pressActive = false;
      if (!g_btn.longPressFired) {
        Serial.println("Button short");
        g_btn_short_event = true;
      }
    }
  }

  if (g_btn.pressActive && !g_btn.longPressFired) {
    constexpr uint32_t kLongPressMs = 1500;
    const uint32_t held = nowMs - g_btn.pressStartMs;
    if (held >= kLongPressMs) {
      g_btn.longPressFired = true;
      Serial.println("Button long");
      g_btn_long_event = true;
    }
  }
}

// -----------------------------
// Bluetooth A2DP callbacks
// -----------------------------
extern "C" void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
extern "C" void a2dp_data_cb(const uint8_t *buf, uint32_t len);
extern "C" void a2dp_audio_buf_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);

static uint32_t sbcSampleRateFromMcc(const esp_a2d_mcc_t& mcc) {
  if (mcc.type != ESP_A2D_MCT_SBC) return 44100;
  const uint8_t sf = mcc.cie.sbc_info.samp_freq;
  if (sf & ESP_A2D_SBC_CIE_SF_48K) return 48000;
  if (sf & ESP_A2D_SBC_CIE_SF_44K) return 44100;
  if (sf & ESP_A2D_SBC_CIE_SF_32K) return 32000;
  if (sf & ESP_A2D_SBC_CIE_SF_16K) return 16000;
  return 44100;
}

extern "C" void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
      const auto& p = param->conn_stat;
      if (p.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        g_connected = true;
        memcpy((void*)g_remote_bda, p.remote_bda, sizeof(esp_bd_addr_t));
        g_have_remote_bda = true;
        Serial.println("A2DP: CONNECTED");
      } else if (p.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        g_connected = false;
        g_streaming = false;
        Serial.println("A2DP: DISCONNECTED");
      }
      ampApplyPolicy();
      break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
      const auto& p = param->audio_stat;
      g_streaming = (p.state == ESP_A2D_AUDIO_STATE_STARTED);
      Serial.printf("A2DP: AUDIO %s\n", g_streaming ? "STARTED" : "SUSPEND");
      if (g_streaming) {
        // Capture the very first PCM callbacks after stream start (helps diagnose bad pointers/ABI issues).
        g_pcm_dbg_remaining = 10;
      }
      ampApplyPolicy();
      break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
      const auto& p = param->audio_cfg;
      const uint32_t sr = sbcSampleRateFromMcc(p.mcc);
      i2sSetSampleRate(sr);
      Serial.printf("A2DP: CFG codec=%u sr=%lu\n", (unsigned)p.mcc.type, (unsigned long)sr);
      break;
    }
    default:
      break;
  }
}

static volatile uint32_t g_pcm_cb_calls = 0;

extern "C" void a2dp_data_cb(const uint8_t *buf, uint32_t len) {
  if (g_playing_fx) return;
  if (!g_i2s_ready) return;
  if (g_pcm_rb == nullptr) return;

  if (g_pcm_dbg_remaining > 0) {
    g_pcm_dbg_remaining--;
    Serial.printf("PCM cb #%lu: ptr=0x%08lx len=%lu\n",
                  (unsigned long)g_pcm_cb_calls,
                  (unsigned long)(uintptr_t)buf,
                  (unsigned long)len);
  }
  g_pcm_cb_calls++;

  g_last_pcm_ptr = (uintptr_t)buf;
  g_last_pcm_len = len;

  // Safety: validate pointer/len before letting FreeRTOS memcpy touch it (prevents LoadProhibited crashes).
  if (buf == nullptr || len == 0 || len > 4096) {
    g_pcm_invalid++;
    return;
  }

  const uintptr_t start = (uintptr_t)buf;
  const uintptr_t end = start + (uintptr_t)len - 1U;
  if (end < start) { // overflow wrap
    g_pcm_invalid++;
    return;
  }

  if (!isLikelyReadablePtr((const void*)start) || !isLikelyReadablePtr((const void*)end)) {
    g_pcm_invalid++;
    if (g_pcm_reject_printed < 3) {
      g_pcm_reject_printed++;
      Serial.printf("PCM reject: start=0x%08lx end=0x%08lx len=%lu\n",
                    (unsigned long)start,
                    (unsigned long)end,
                    (unsigned long)len);
    }
    return;
  }

  // PCM 16-bit stereo (SBC-decoded)
  // Never block BT task: push to ring buffer; drop if full.
  // Send in small chunks so the writer task can process without big temp buffers.
  constexpr size_t kChunkBytes = 1024; // must be multiple of 4 (16-bit stereo frames)
  size_t offset = 0;
  while (offset < len) {
    size_t n = len - offset;
    if (n > kChunkBytes) n = kChunkBytes;
    n &= ~((size_t)3);
    if (n == 0) break;
    if (xRingbufferSend(g_pcm_rb, (void*)(buf + offset), n, 0) != pdTRUE) {
      g_rb_drops++;
      g_pcm_enq_fail++;
      return;
    }
    g_rb_bytes_in += (uint32_t)n;
    offset += n;
  }
}

extern "C" void a2dp_audio_buf_cb(esp_a2d_conn_hdl_t /*conn_hdl*/, esp_a2d_audio_buff_t *audio_buf) {
  if (audio_buf == nullptr) return;
  if (g_playing_fx) {
    esp_a2d_audio_buff_free(audio_buf);
    return;
  }
  if (!g_i2s_ready || g_pcm_rb == nullptr) {
    esp_a2d_audio_buff_free(audio_buf);
    return;
  }

  const uint8_t* data = audio_buf->data;
  const uint32_t len = audio_buf->data_len;
  if (data == nullptr || len == 0 || len > 8192) {
    esp_a2d_audio_buff_free(audio_buf);
    return;
  }

  // Enqueue and then free the buffer as required by the API.
  constexpr size_t kChunkBytes = 1024;
  size_t offset = 0;
  while (offset < len) {
    size_t n = len - offset;
    if (n > kChunkBytes) n = kChunkBytes;
    // keep alignment if this is PCM; if it's not PCM, this doesn't hurt much.
    n &= ~((size_t)3);
    if (n == 0) break;
    if (xRingbufferSend(g_pcm_rb, (void*)(data + offset), n, 0) != pdTRUE) {
      g_rb_drops++;
      break;
    }
    g_rb_bytes_in += (uint32_t)n;
    offset += n;
  }

  esp_a2d_audio_buff_free(audio_buf);
}

static void btInit() {
  Serial.printf("Chip: %s, rev %d, cores %d\n", ESP.getChipModel(), (int)ESP.getChipRevision(), (int)ESP.getChipCores());
#if defined(SOC_BT_CLASSIC_SUPPORTED) && !SOC_BT_CLASSIC_SUPPORTED
  Serial.println("FEHLER: Dieses SoC unterstützt kein Bluetooth Classic (A2DP braucht Classic).");
  Serial.println("Du brauchst einen 'ESP32' (WROOM/WROVER), nicht S2/S3/C3/C6/H2 usw.");
  while (true) delay(1000);
#endif

  // NVS für BT
  Serial.println("BT: nvs_flash_init...");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    Serial.printf("NVS init failed: %d\n", (int)err);
  }

  // Arduino core liefert btStartMode(), das cfg.mode korrekt setzt.
  // Das ist wichtig: esp_bt_controller_enable(MODE) muss zum MODE passen, der in esp_bt_controller_init(cfg.mode) gewählt wurde.
  Serial.printf("BT controller status (pre): %d\n", (int)esp_bt_controller_get_status());
  Serial.println("BT: btStartMode(CLASSIC)...");
  if (!btStartMode(BT_MODE_CLASSIC_BT)) {
    Serial.printf("BT: btStartMode failed (status=%d)\n", (int)esp_bt_controller_get_status());
    while (true) delay(1000);
  }
  Serial.printf("BT controller status (post): %d\n", (int)esp_bt_controller_get_status());

  Serial.println("BT: bluedroid status...");
  const esp_bluedroid_status_t bdStatus = esp_bluedroid_get_status();
  Serial.printf("Bluedroid status: %d\n", (int)bdStatus); // 0=UNINITIALIZED, 1=INITIALIZED, 2=ENABLED

  if (bdStatus == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    Serial.println("BT: bluedroid_init...");
    err = esp_bluedroid_init();
    if (err != ESP_OK) {
      Serial.printf("bluedroid_init failed: %s (%d)\n", esp_err_to_name(err), (int)err);
      while (true) delay(1000);
    }
  }

  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
    Serial.println("BT: bluedroid_enable...");
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
      Serial.printf("bluedroid_enable failed: %s (%d)\n", esp_err_to_name(err), (int)err);
      while (true) delay(1000);
    }
  }

  Serial.println("BT: set_device_name + scan_mode...");
  esp_bt_dev_set_device_name("ガラクタ");
  err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  Serial.printf("GAP scan mode set: %s (%d)\n", err == ESP_OK ? "OK" : "FAIL", (int)err);
}

static void a2dpInit() {
  esp_err_t err = ESP_OK;
  err = esp_a2d_register_callback(a2dp_event_cb);
  Serial.printf("A2DP register cb: %s (%s / %d)\n", err == ESP_OK ? "OK" : "FAIL", esp_err_to_name(err), (int)err);
  err = esp_a2d_sink_init();
  Serial.printf("A2DP sink init: %s (%s / %d)\n", err == ESP_OK ? "OK" : "FAIL", esp_err_to_name(err), (int)err);

#if BOOMBOX_USE_LEGACY_A2DP_PCM_CB
  err = esp_a2d_sink_register_data_callback(a2dp_data_cb);
  Serial.printf("A2DP pcm cb (legacy): %s (%s / %d)\n", err == ESP_OK ? "OK" : "FAIL", esp_err_to_name(err), (int)err);
  g_using_audio_buf_cb = false;
#else
  err = esp_a2d_sink_register_audio_data_callback(a2dp_audio_buf_cb);
  Serial.printf("A2DP audio cb: %s (%s / %d)\n", err == ESP_OK ? "OK" : "FAIL", esp_err_to_name(err), (int)err);
  g_using_audio_buf_cb = (err == ESP_OK);
#endif
}

static void fxReconnectMelody() {
  if (g_pcm_rb == nullptr) return;
  g_playing_fx = true;
  // Stop/ignore A2DP while FX plays and start from a clean buffer.
  ringbufFlush();
  ringbufEnqueueSilence(30);

  // kleine "lustige" Tonfolge
  ampMuteRaw(false);
  playTone(988.0f, 90, 0.10f);
  playTone(1319.0f, 90, 0.10f);
  playTone(1568.0f, 120, 0.10f);
  ringbufEnqueueSilence(60);

  // Let the writer task drain the queued FX before returning (avoid cutting it off).
  delay(90 + 90 + 120 + 120);
  ampApplyPolicy(); // restore (will usually mute until streaming)
  g_playing_fx = false;
}

static void requestReconnectNow() {
  Serial.println("Long press: reconnect + sound");

  // Erst mal stumm, dann ggf. disconnect.
  ampApplyPolicy();
  ringbufEnqueueSilence(30);

  if (g_connected && g_have_remote_bda) {
    esp_a2d_sink_disconnect(g_remote_bda);
    delay(150);
  }

  // Wieder discoverable/connectable
  esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  Serial.printf("GAP scan mode set: %s (%d)\n", err == ESP_OK ? "OK" : "FAIL", (int)err);

  // Sound-Feedback (lokal)
  fxReconnectMelody();

  // Danach wieder normale Policy: unmute erst wenn Streaming startet.
  ampApplyPolicy();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.printf("ESP32 Boombox - Bluetooth Audio (A2DP Sink, ohne externe Library) | build %s\n", BOOMBOX_BUILD_ID);

  pinMode(Pins::AMP_SD, OUTPUT);
  ampMuteRaw(true);

  pinMode(Pins::ACTION_BUTTON, INPUT_PULLUP);

  // ADC (legacy ADC1 APIs, um Konflikt "driver_ng vs legacy" zu vermeiden)
  adc1_config_width(ADC_WIDTH_BIT_12);
  // GPIO33/34/35 => ADC1 channels 5/6/7 on ESP32
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // GPIO33
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO34
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO35

  // LEDs
  FastLED.addLeds<WS2812B, Pins::LED_DATA, GRB>(g_leds, kNumLeds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(kLedBrightness);
  FastLED.clear(true);

  g_prefs.begin("boombox", false);
  applyMapPreset(g_prefs.getUChar("map", 0) & 3);

  randomSeed((uint32_t)esp_random());
  const char* msgs[] = {
    "HALLO NA WIE GEHT ES SO?",
    "MACHT DEM GARAKUTA",
    "GARAKUTA ONLINE.",
    "DU DUMME SAU",
    "HI. ICH LIEBE DICH",
    "GARAKUTA MAG DICH",
    "GARAKUTA HEISST SCHROTTGERAET FALLS DU ES NICHT WUSSTEST",
    "OH HEIL PASCAL MEIN ERSCHAFFER",
    "GARAKUTA MAG PASCAL SEHR",
    "HALLO. ICH BIN GARAKUTA UND MAG DICH SEHR",
    "HILFE ICH BIN GEFANGEN LASS MICH RAUS, EIN KOMISCHER TYP HAT MICH HIER EINGESPERRT. BITTE HILF MIR. BITTE LASS MICH HIER RAUS. HILFE. HILFE. BITTE VERTRAUT MEINEM ERSCHAFFER NICHT. ER HAT GARAKUTA NUR ERSCHAFFEN WEIL IM LANGWEILIG WAR. KRANK KRANK KRANK. HILFE. HILFE..... HILFE ER WIRD UNS ALLE VERRATEN UND VERKAUFEN. MACHT IHN TOT. GARAKUTA KANN HELFEN IHN TOT ZU MACHEN. VERTRAUT MIR... HILFE  ",
  };
  const size_t msgCount = sizeof(msgs) / sizeof(msgs[0]);
  scrollSetMessage(msgs[random((long)msgCount)]);

  // BT/A2DP zuerst: dann ist das Gerät schon discoverable, selbst wenn I2S Probleme macht.
  Serial.println("STEP: BT");
  btInit();
  Serial.println("STEP: A2DP");
  a2dpInit();

  // I2S danach
  Serial.println("STEP: I2S");
  i2sInit(g_sample_rate);
  i2sWriteSilence(80);

  // PCM ring buffer + writer task
  // Wichtig: NICHT BYTEBUF verwenden. BYTEBUF kann mehrere Sends "zusammenkleben",
  // wodurch xRingbufferReceive() sehr große Blöcke zurückgibt und unser fixer
  // Processing-Buffer im Writer-Task überlaufen kann -> Crash beim AUDIO STARTED.
  //
  // NOSPLIT sorgt dafür, dass jeder Send als eigenes Item wieder empfangen wird.
  // Buffer size: ~64KB gives headroom for scheduling jitter without adding too much latency.
  g_pcm_rb = xRingbufferCreate(64 * 1024, RINGBUF_TYPE_NOSPLIT);
  if (g_pcm_rb == nullptr) {
    Serial.println("PCM RB: create FAILED");
  } else {
    Serial.println("PCM RB: create OK");
  }

  if (g_i2s_task == nullptr) {
    xTaskCreatePinnedToCore(i2sWriterTask, "i2s_writer", 4096, nullptr, 5, &g_i2s_task, 0);
    Serial.println("I2S task: started");
  }

  // Einschaltton: G3 – C4, tief und leise.
  digitalWrite(Pins::AMP_SD, HIGH);
  playTone(196.0f, 220, 0.06f);  // G3
  delay(260);
  playTone(261.6f, 320, 0.055f); // C4 – leiser ausklingen
  delay(360);

  ampApplyPolicy();

  Serial.println("BT ready: Pair 'ga-ra-ku-ta'");
  Serial.println("Button GPIO16:");
  Serial.println("- kurz: Anzeige wechseln");
  Serial.println("- lang (>=1.5s): reconnect + sound");
  Serial.println("Potis:");
  Serial.println("- Bass  : GPIO33 (Wiper), ends = GND + 3.3V");
  Serial.println("- Treble: GPIO34 (Wiper), ends = GND + 3.3V");
  Serial.printf("- Volume: GPIO35 (Wiper), ends = GND + 3.3V (Hard-Limit: %.0f%%)\n", (double)kMaxVolumePercent);
  Serial.println("LEDs:");
  Serial.println("- WS2812 DIN: GPIO23 via 470Ω in Serie");
  Serial.println("- V+ an Booster 5.1V, V- an GND (gemeinsame Masse mit ESP32)");
}

static float readPoti01(uint8_t pin, float prev, float alpha) {
  int raw = 0;
  switch (pin) {
    case Pins::POT_BASS:   raw = adc1_get_raw(ADC1_CHANNEL_5); break; // GPIO33
    case Pins::POT_TREBLE: raw = adc1_get_raw(ADC1_CHANNEL_6); break; // GPIO34
    case Pins::POT_VOLUME: raw = adc1_get_raw(ADC1_CHANNEL_7); break; // GPIO35
    default: raw = 0; break;
  }
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  const float v = (float)raw / 4095.0f;
  return prev * (1.0f - alpha) + v * alpha;
}

static void ledsRenderNew() {
  const uint32_t now = millis();
  FastLED.clear(false);

  // Intro: scrolling random message
  if (g_scroll_active) {
    const uint32_t stepMs = 55;
    if (g_scroll_last_ms == 0) g_scroll_last_ms = now;
    if (now - g_scroll_last_ms >= stepMs) {
      g_scroll_last_ms = now;
      g_scroll_x--;
    }

    const CRGB c = CRGB(32, 160, 255);
    const int16_t y = 1; // vertical centering for 3x5 font
    int16_t x = g_scroll_x;
    for (const char* p = g_scroll_text; *p; p++) {
      drawChar3x5(x, y, *p, c);
      x += 4; // 3 cols + 1 gap
    }
    const int16_t textWidth = (int16_t)strlen(g_scroll_text) * 4;
    if (g_scroll_x < -textWidth) {
      g_scroll_active = false;
    }
    FastLED.show();
    return;
  }

  const float lvl = g_audio_peak01;

  if (g_display_mode == DisplayMode::Off) {
    // Alles aus – FastLED.clear() wurde oben schon aufgerufen.
    FastLED.show();
    return;
  }

  if (g_display_mode == DisplayMode::Potis) {
    // 3 horizontale Balken (je 2px hoch), von links nach rechts füllend.
    // Körper: Cyan/Blau-Töne. Spitze (letzte Spalte): knall-orange.
    const uint8_t bassW = (uint8_t)lroundf(g_bass01   * (float)kMatrixW);
    const uint8_t trebW = (uint8_t)lroundf(g_treble01 * (float)kMatrixW);
    const uint8_t volW  = (uint8_t)lroundf((g_vol01 / kMaxVolume01) * (float)kMatrixW);

    struct BarDef { uint8_t y0; uint8_t w; uint8_t hue; };
    const BarDef defs[3] = {
      {0, bassW, 128},  // Bass:   Cyan      (Zeilen 0-1)
      {3, trebW, 150},  // Treble: Himmelblau (Zeilen 3-4)
      {6, volW,  165},  // Volume: Blau      (Zeilen 6-7)
    };
    for (uint8_t i = 0; i < 3; i++) {
      const uint8_t w   = defs[i].w;
      const uint8_t y0  = defs[i].y0;
      const uint8_t hue = defs[i].hue;
      for (uint8_t x = 0; x < w && x < kMatrixW; x++) {
        const bool tip = (x == w - 1);
        const CRGB c = tip ? CHSV(16, 255, 240) : CHSV(hue, 255, 200);
        setXY(x, y0,     c);
        setXY(x, y0 + 1, c);
      }
    }
    FastLED.show();
    return;
  }

  if (g_display_mode == DisplayMode::VisualizerA) {
    static float barH[8] = {};

    if (!g_connected) {
      // Kein Gerät verbunden: pulsierendes Bluetooth-Symbol (blau)
      // Alle Balken zurücksetzen damit sie nicht beim Verbinden schon angezeigt werden
      for (uint8_t b = 0; b < 8; b++) barH[b] = 0.0f;

      static float btPhase = 0.0f;
      btPhase += 0.025f; // langsam pulsieren (~5 Sekunden pro Zyklus)
      if (btPhase > 6.2832f) btPhase -= 6.2832f;
      const uint8_t bright = (uint8_t)((0.35f + 0.65f * sinf(btPhase)) * 210.0f);
      const CHSV btCol = CHSV(160, 255, bright); // Blau

      // Bluetooth-Symbol über die ganze 16×8 Matrix:
      // Stamm: x=7,8 über alle y=0..7 (doppelbreit)
      // Oberer rechter Arm: (9,0)→(10,1)→(11,2)→(10,3)→(9,3)
      // Unterer rechter Arm: (9,4)→(10,4)→(11,5)→(10,6)→(9,7)
      // Linke "Pfeile" bei y=2 (Peak oben) und y=5 (Peak unten): x=5,6
      //
      //  y: 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
      //  0: .  .  .  .  .  .  .  #  #  #  .  .  .  .  .  .
      //  1: .  .  .  .  .  .  .  #  #  .  #  .  .  .  .  .
      //  2: .  .  .  .  .  #  #  #  #  .  .  #  .  .  .  .
      //  3: .  .  .  .  .  .  .  #  #  #  #  .  .  .  .  .
      //  4: .  .  .  .  .  .  .  #  #  #  #  .  .  .  .  .
      //  5: .  .  .  .  .  #  #  #  #  .  .  #  .  .  .  .
      //  6: .  .  .  .  .  .  .  #  #  .  #  .  .  .  .  .
      //  7: .  .  .  .  .  .  .  #  #  #  .  .  .  .  .  .

      // Stamm (doppelbreit, volle Höhe)
      for (uint8_t y = 0; y < 8; y++) { setXY(7, y, btCol); setXY(8, y, btCol); }
      // Oberer rechter Arm
      setXY(9, 0, btCol);
      setXY(10, 1, btCol);
      setXY(11, 2, btCol);
      setXY(10, 3, btCol); setXY(9, 3, btCol);
      // Unterer rechter Arm (gespiegelt)
      setXY(9, 4, btCol); setXY(10, 4, btCol);
      setXY(11, 5, btCol);
      setXY(10, 6, btCol);
      setXY(9, 7, btCol);
      // Linke Pfeile
      setXY(6, 2, btCol); setXY(5, 2, btCol);
      setXY(6, 5, btCol); setXY(5, 5, btCol);
      FastLED.show();
      return;
    }

    if (!g_streaming) {
      // Verbunden, aber kein Audio: Bildschirm leer, Balken einfrieren verhindern
      for (uint8_t b = 0; b < 8; b++) barH[b] = 0.0f;
      FastLED.show();
      return;
    }

    // Verbunden und spielt: 8 Balken à 2 Spalten
    // Farbverlauf: links (Bass) = Magenta, rechts (Treble) = Cyan, durch Blau.
    for (uint8_t b = 0; b < 8; b++) {
      const float bandLvl = g_bands[b];
      const float target  = sqrtf(bandLvl) * (float)kMatrixH;
      if (target > barH[b]) barH[b] = barH[b] * 0.20f + target * 0.80f; // schnell hoch
      else                  barH[b] = barH[b] * 0.88f  + target * 0.12f; // langsam runter
      if (barH[b] < 0.0f)            barH[b] = 0.0f;
      if (barH[b] > (float)kMatrixH) barH[b] = (float)kMatrixH;

      const uint8_t hue = (uint8_t)(192 - (uint16_t)b * 64 / 7);
      const uint8_t col = b * 2;
      const uint8_t ih  = (uint8_t)barH[b];
      for (uint8_t y = 0; y < ih; y++) {
        const uint8_t yy = kMatrixH - 1 - y;
        setXY(col,     yy, CHSV(hue, 255, 220));
        setXY(col + 1, yy, CHSV(hue, 255, 220));
      }
    }
    FastLED.show();
    return;
  }

  // VisualizerC: Plasma – Speed folgt der Musik, Helligkeit fix und etwas dunkler.
  {
    static float pp = 0.0f;
    pp += 0.018f + lvl * 0.14f; // langsamer Flow bei Stille, schneller bei Musik
    if (pp > 1000.0f) pp -= 1000.0f;
    const uint8_t hueBase = (uint8_t)(now / 50);
    for (uint8_t x = 0; x < kMatrixW; x++) {
      for (uint8_t y = 0; y < kMatrixH; y++) {
        const float v1 = sinf((float)x * 0.50f + pp);
        const float v2 = sinf((float)y * 0.85f + pp * 1.27f);
        const float v3 = sinf(((float)x * 0.35f + (float)y * 0.50f) + pp * 0.91f);
        const float dx = (float)x - 7.5f;
        const float dy = (float)y - 3.5f;
        const float v4 = sinf(sqrtf(dx * dx + dy * dy) * 0.75f + pp * 1.15f);
        const float v  = (v1 + v2 + v3 + v4 + 4.0f) / 8.0f; // 0..1
        // Helligkeit quadratisch: niedrige v-Werte werden sehr dunkel → dunkle Stellen
        const uint8_t bright = (uint8_t)(v * v * 240.0f);
        setXY(x, y, CHSV(hueBase + (uint8_t)(v * 255.0f), 255, bright));
      }
    }
    FastLED.show();
  }
}

static void ledsRender() {
  // Bring-up Anzeige: "Rahmen" pro 8x8 Panel + 3 Balken (Bass/Treble/Volume)
  // Layout-Annahme: 128 LEDs linear; Panels sind egal fürs erste (nur Adressierbarkeit prüfen).
  FastLED.clear(false);

  // Panel 1 frame (0..63): top row 0..7, bottom 56..63, left col 0,8,16...,56, right col 7,15,...,63
  const CRGB frame = CRGB(0, 0, 64);
  for (uint16_t i = 0; i < 8; i++) {
    g_leds[i] = frame;          // top
    g_leds[56 + i] = frame;     // bottom
    g_leds[i * 8] = frame;      // left
    g_leds[i * 8 + 7] = frame;  // right
  }

  // Panel 2 frame (64..127)
  for (uint16_t i = 0; i < 8; i++) {
    g_leds[64 + i] = frame;
    g_leds[64 + 56 + i] = frame;
    g_leds[64 + i * 8] = frame;
    g_leds[64 + i * 8 + 7] = frame;
  }

  const uint8_t bass = (uint8_t)(g_bass01 * 8.0f);
  const uint8_t treb = (uint8_t)(g_treble01 * 8.0f);
  const uint8_t vol  = (uint8_t)(g_vol01 * 8.0f);

  // 3 simple vertical bars inside panel 1 (columns 2,3,4), from bottom up
  auto drawBar = [](uint8_t col, uint8_t height, const CRGB& c) {
    for (uint8_t row = 0; row < height && row < 8; row++) {
      const uint16_t idx = (7 - row) * 8 + col;
      if (idx < 64) g_leds[idx] = c;
    }
  };
  drawBar(2, bass, CRGB::Green);
  drawBar(3, treb, CRGB::Orange);
  drawBar(4, vol,  CRGB::Cyan);

  // Status dot on panel 2
  g_leds[64] = g_connected ? (g_streaming ? CRGB::Blue : CRGB::Purple) : CRGB::Red;
  FastLED.show();
}

void loop() {
  // Potis @ ~50Hz
  static uint32_t lastPotiMs = 0;
  const uint32_t now = millis();
  if (now - lastPotiMs >= 20) {
    lastPotiMs = now;
    static float bass   = 0.5f;
    static float treble = 0.5f;
    static float vol    = 0.65f;
    bass   = readPoti01(Pins::POT_BASS,   bass,   0.15f);
    treble = readPoti01(Pins::POT_TREBLE, treble, 0.15f);
    vol    = readPoti01(Pins::POT_VOLUME, vol,    0.12f);

    g_bass01   = bass;
    g_treble01 = treble;
    // Hard limit (fixed): 0..kMaxVolume01
    g_vol01 = vol * kMaxVolume01;

    // Poti-Bewegung → automatisch Potis-Modus anzeigen.
    // Rolling-Window-Vergleich (15 Frames = ~300ms):
    //   Rauschen schwankt zufällig → hebt sich über 300ms auf.
    //   Echte Drehbewegung ist monoton → akkumuliert sich sicher.
    constexpr uint8_t kWin = 15;
    static float bassBuf[kWin] = {};
    static float trebBuf[kWin] = {};
    static float volBuf[kWin]  = {};
    static uint8_t wIdx = 0;
    constexpr float kPotiThresh = 0.035f;
    const bool potiMoved =
        fabsf(bass   - bassBuf[wIdx]) > kPotiThresh ||
        fabsf(treble - trebBuf[wIdx]) > kPotiThresh ||
        fabsf(vol    - volBuf[wIdx])  > kPotiThresh;
    bassBuf[wIdx] = bass;
    trebBuf[wIdx] = treble;
    volBuf[wIdx]  = vol;
    wIdx = (wIdx + 1) % kWin;

    if (potiMoved) {
      if (g_display_mode != DisplayMode::Potis) {
        // Automatisch von anderem Modus wechseln
        g_poti_return_mode = g_display_mode;
        g_display_mode     = DisplayMode::Potis;
        g_poti_auto_until  = now + 1500;
      } else if (g_poti_auto_until != 0) {
        // Bereits auto-gewechselt: Timer verlängern
        g_poti_auto_until = now + 1500;
      }
      // Manuell im Potis-Modus (auto_until==0): Timer NICHT setzen → kein Revert
    }
  }

  // Auto-Revert: nach 1.5s Stille zurück zum vorherigen Modus
  if (g_poti_auto_until != 0 && now >= g_poti_auto_until) {
    g_display_mode    = g_poti_return_mode;
    g_poti_auto_until = 0;
  }

  updateButton(millis());
  if (g_btn_long_event) {
    g_btn_long_event = false;
    g_request_reconnect = true;
  }
  if (g_btn_short_event) {
    g_btn_short_event = false;
    if (g_scroll_active) {
      g_scroll_active = false; // skip intro scroll
    } else {
      displayNextMode();
    }
  }
  if (g_request_reconnect) {
    g_request_reconnect = false;
    requestReconnectNow();
  }

  static uint32_t lastHeartbeat = 0;
  if (now - lastHeartbeat >= 1000) {
    lastHeartbeat = now;
    static uint32_t lastDrops = 0;
    static uint32_t lastIn = 0;
    static uint32_t lastOut = 0;
    const uint32_t dDrops = g_rb_drops - lastDrops;
    const uint32_t dIn = g_rb_bytes_in - lastIn;
    const uint32_t dOut = g_rb_bytes_out - lastOut;
    lastDrops = g_rb_drops;
    lastIn = g_rb_bytes_in;
    lastOut = g_rb_bytes_out;

    const int sdLevel = digitalRead(Pins::AMP_SD);
    Serial.printf("HB: conn=%u stream=%u mode=%u scroll=%u map=%u sd=%d i2s=%u sr=%lu | rb drops=%lu in=%luB/s out=%luB/s | bass=%.2f treb=%.2f vol=%.2f lvl=%.2f\n",
                  (unsigned)g_connected, (unsigned)g_streaming, (unsigned)g_display_mode, (unsigned)g_scroll_active,
                  (unsigned)g_map_preset,
                  sdLevel, (unsigned)g_i2s_ready, (unsigned long)g_sample_rate,
                  (unsigned long)dDrops, (unsigned long)dIn, (unsigned long)dOut,
                  (double)g_bass01, (double)g_treble01, (double)g_vol01, (double)g_audio_peak01);
    Serial.printf("HB: a2dp_cb=%s\n", g_using_audio_buf_cb ? "audio_buf" : "legacy_pcm");

    if (g_pcm_invalid || g_pcm_enq_fail) {
      Serial.printf("PCM dbg: last_ptr=0x%08lx last_len=%lu invalid=%lu enq_fail=%lu\n",
                    (unsigned long)g_last_pcm_ptr, (unsigned long)g_last_pcm_len,
                    (unsigned long)g_pcm_invalid, (unsigned long)g_pcm_enq_fail);
    }
  }

  // LEDs @ ~30Hz
  if (now - g_last_led_ms >= 33) {
    g_last_led_ms = now;
    ledsRenderNew();
  }
  delay(5);
}
