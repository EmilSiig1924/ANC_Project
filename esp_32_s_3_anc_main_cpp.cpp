// src/main.cpp
// ESP32-S3 ANC prototype firmware
// Board: 4d_systems_esp32s3_gen4_r8n16
// Audio: 2x INMP441 on I2S RX shared SD line + PCM5102A on I2S TX
// Framework: Arduino + ESP-IDF I2S legacy driver through PlatformIO

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// =========================
// Pin layout
// =========================
static constexpr i2s_port_t I2S_MIC_PORT = I2S_NUM_1;
static constexpr int MIC_BCLK_PIN = 14;
static constexpr int MIC_WS_PIN   = 15;
static constexpr int MIC_SD_PIN   = 16;

static constexpr i2s_port_t I2S_DAC_PORT = I2S_NUM_0;
static constexpr int DAC_BCLK_PIN = 4;
static constexpr int DAC_WS_PIN   = 5;
static constexpr int DAC_DIN_PIN  = 6;

// =========================
// Audio constants
// =========================
static constexpr int SAMPLE_RATE = 48000;
static constexpr int BLOCK_FRAMES = 128;       // Stereo frames per processing block
static constexpr int16_t I16_MAX_SAFE = 30000;

// INMP441 usually outputs 24-bit data in a 32-bit I2S slot.
// We receive stereo 32-bit slots, then convert to float.
static int32_t rx_raw[BLOCK_FRAMES * 2];
static int16_t tx_raw[BLOCK_FRAMES * 2];       // Stereo output to DAC: duplicate mono to L/R

// Slot mapping from your repo docs:
// Frame format RIGHT_LEFT, MIC=1 -> Left/reference, MIC=2 -> Right/error.
// In a common interleaved buffer, index 0 is usually Left and index 1 Right with Arduino/IDF standard mode.
// If your check_lr_mapping.py proves it reversed, swap these two indices.
static constexpr int REF_SLOT_INDEX = 0;       // Left slot / MIC=1 / reference mic
static constexpr int ERR_SLOT_INDEX = 1;       // Right slot / MIC=2 / error mic

// =========================
// ANC state
// =========================
enum class AncMode {
  STOPPED,
  DIRECT,
  FXLMS
};

static volatile AncMode g_mode = AncMode::STOPPED;

static float g_gain = 0.12f;                   // Safe startup output gain
static float g_mu = 0.00015f;                  // Adaptive step size. Start small.
static float g_leak = 0.00002f;                // Weight leakage
static float g_limit = 0.20f;                  // Output limiter, normalized [-1,1]
static int g_delay_samples = 0;                // Direct/FxLMS reference delay
static int g_sign = -1;                        // Direct inversion sign

static constexpr int DELAY_BUF_LEN = 2048;
static float ref_delay_buf[DELAY_BUF_LEN];
static int ref_delay_wr = 0;

static constexpr int FILTER_LEN = 96;
static float w[FILTER_LEN];                    // Adaptive FIR weights
static float x_hist[FILTER_LEN];               // Reference history

// Simple band conditioning, first-order HP + LP. Not a perfect ANC filter, but safe and cheap.
static float hp_hz = 70.0f;
static float lp_hz = 2200.0f;
static float hp_y_prev = 0.0f;
static float hp_x_prev = 0.0f;
static float lp_y_prev = 0.0f;

static uint32_t rx_read_errs = 0;
static uint32_t tx_write_errs = 0;
static uint32_t tx_short_writes = 0;
static uint32_t blocks_processed = 0;

// =========================
// Utility
// =========================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int16_t f32_to_i16(float x) {
  x = clampf(x, -g_limit, g_limit);
  return static_cast<int16_t>(x * I16_MAX_SAFE);
}

static float i2s32_to_float(int32_t s) {
  // INMP441 24-bit sample is commonly left-aligned in 32-bit slot.
  // Shift down to signed 24-bit-ish then normalize conservatively.
  int32_t v = s >> 8;
  return clampf(static_cast<float>(v) / 8388608.0f, -1.0f, 1.0f);
}

static float band_condition(float x) {
  // One-pole high-pass
  const float dt = 1.0f / SAMPLE_RATE;
  const float rc_hp = 1.0f / (2.0f * PI * hp_hz);
  const float a_hp = rc_hp / (rc_hp + dt);
  float hp = a_hp * (hp_y_prev + x - hp_x_prev);
  hp_y_prev = hp;
  hp_x_prev = x;

  // One-pole low-pass
  const float rc_lp = 1.0f / (2.0f * PI * lp_hz);
  const float a_lp = dt / (rc_lp + dt);
  lp_y_prev = lp_y_prev + a_lp * (hp - lp_y_prev);
  return lp_y_prev;
}

static float push_delay_and_get(float x, int delay_samples) {
  ref_delay_buf[ref_delay_wr] = x;
  int rd = ref_delay_wr - delay_samples;
  while (rd < 0) rd += DELAY_BUF_LEN;
  float y = ref_delay_buf[rd % DELAY_BUF_LEN];
  ref_delay_wr = (ref_delay_wr + 1) % DELAY_BUF_LEN;
  return y;
}

static void reset_anc_state() {
  memset(ref_delay_buf, 0, sizeof(ref_delay_buf));
  memset(w, 0, sizeof(w));
  memset(x_hist, 0, sizeof(x_hist));
  ref_delay_wr = 0;
  hp_y_prev = hp_x_prev = lp_y_prev = 0.0f;
  blocks_processed = 0;
}

// =========================
// I2S setup
// =========================
static bool setup_mic_i2s() {
  i2s_config_t cfg = {};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = BLOCK_FRAMES;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = MIC_BCLK_PIN;
  pins.ws_io_num = MIC_WS_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_SD_PIN;

  esp_err_t err = i2s_driver_install(I2S_MIC_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) return false;
  err = i2s_set_pin(I2S_MIC_PORT, &pins);
  if (err != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_MIC_PORT);
  return true;
}

static bool setup_dac_i2s() {
  i2s_config_t cfg = {};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = BLOCK_FRAMES;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = DAC_BCLK_PIN;
  pins.ws_io_num = DAC_WS_PIN;
  pins.data_out_num = DAC_DIN_PIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t err = i2s_driver_install(I2S_DAC_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) return false;
  err = i2s_set_pin(I2S_DAC_PORT, &pins);
  if (err != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_DAC_PORT);
  return true;
}

// =========================
// ANC processing
// =========================
static float process_direct(float ref) {
  float delayed = push_delay_and_get(ref, g_delay_samples);
  return g_sign * g_gain * delayed;
}

static float process_fxlms(float ref, float err) {
  // NOTE: This is intentionally a safe simplified adaptive FIR.
  // True FxLMS should filter x_hist by a measured secondary-path estimate S-hat.
  // This version is still useful for early hardware tests, but you should later add S-hat calibration.

  float x = push_delay_and_get(ref, g_delay_samples);

  for (int i = FILTER_LEN - 1; i > 0; --i) x_hist[i] = x_hist[i - 1];
  x_hist[0] = x;

  float y = 0.0f;
  for (int i = 0; i < FILTER_LEN; ++i) y += w[i] * x_hist[i];
  y *= g_gain;
  y = clampf(y, -g_limit, g_limit);

  // NLMS-like normalization for stability
  float energy = 1e-5f;
  for (int i = 0; i < FILTER_LEN; ++i) energy += x_hist[i] * x_hist[i];
  float step = g_mu / energy;

  // Sign may need flipping depending on acoustic path.
  for (int i = 0; i < FILTER_LEN; ++i) {
    w[i] = (1.0f - g_leak) * w[i] - step * err * x_hist[i];
    w[i] = clampf(w[i], -2.0f, 2.0f);
  }

  return y;
}

static void audio_task(void *param) {
  (void)param;

  while (true) {
    size_t bytes_read = 0;
    esp_err_t r = i2s_read(I2S_MIC_PORT, rx_raw, sizeof(rx_raw), &bytes_read, portMAX_DELAY);
    if (r != ESP_OK || bytes_read == 0) {
      rx_read_errs++;
      continue;
    }

    int frames = bytes_read / (sizeof(int32_t) * 2);
    if (frames > BLOCK_FRAMES) frames = BLOCK_FRAMES;

    AncMode mode = g_mode;

    for (int n = 0; n < frames; ++n) {
      int32_t raw_ref = rx_raw[n * 2 + REF_SLOT_INDEX];
      int32_t raw_err = rx_raw[n * 2 + ERR_SLOT_INDEX];

      float ref = band_condition(i2s32_to_float(raw_ref));
      float err = i2s32_to_float(raw_err);

      float out = 0.0f;
      if (mode == AncMode::DIRECT) {
        out = process_direct(ref);
      } else if (mode == AncMode::FXLMS) {
        out = process_fxlms(ref, err);
      }

      int16_t s = f32_to_i16(out);
      tx_raw[n * 2 + 0] = s;
      tx_raw[n * 2 + 1] = s;
    }

    size_t bytes_to_write = frames * 2 * sizeof(int16_t);
    size_t bytes_written = 0;
    esp_err_t werr = i2s_write(I2S_DAC_PORT, tx_raw, bytes_to_write, &bytes_written, 0);
    if (werr != ESP_OK) tx_write_errs++;
    if (bytes_written != bytes_to_write) tx_short_writes++;

    blocks_processed++;
  }
}

// =========================
// Serial commands
// =========================
static void print_status() {
  Serial.printf("STATUS SR=%d MIC_BCLK=%d MIC_WS=%d MIC_SD=%d DAC_BCLK=%d DAC_WS=%d DAC_DIN=%d\n",
                SAMPLE_RATE, MIC_BCLK_PIN, MIC_WS_PIN, MIC_SD_PIN, DAC_BCLK_PIN, DAC_WS_PIN, DAC_DIN_PIN);
  Serial.printf("STATUS MODE=%s GAIN=%.4f MU=%.7f LIMIT=%.3f DELAY=%d SIGN=%d HP=%.1f LP=%.1f\n",
                g_mode == AncMode::STOPPED ? "STOPPED" : (g_mode == AncMode::DIRECT ? "DIRECT" : "FXLMS"),
                g_gain, g_mu, g_limit, g_delay_samples, g_sign, hp_hz, lp_hz);
  Serial.printf("STATUS BLOCKS=%lu RX_ERRS=%lu TX_ERRS=%lu TX_SHORT=%lu\n",
                (unsigned long)blocks_processed, (unsigned long)rx_read_errs,
                (unsigned long)tx_write_errs, (unsigned long)tx_short_writes);
}

static float read_arg_float(const String &cmd, const char *key, float def) {
  String pat = String(key) + "=";
  int p = cmd.indexOf(pat);
  if (p < 0) return def;
  int start = p + pat.length();
  int end = cmd.indexOf(' ', start);
  if (end < 0) end = cmd.length();
  return cmd.substring(start, end).toFloat();
}

static int read_arg_int(const String &cmd, const char *key, int def) {
  return static_cast<int>(read_arg_float(cmd, key, def));
}

static void handle_command(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd == "STATUS" || cmd == "ANC STATUS") {
    print_status();
    return;
  }

  if (cmd == "ANC STOP") {
    g_mode = AncMode::STOPPED;
    reset_anc_state();
    Serial.println("OK ANC STOPPED");
    return;
  }

  if (cmd.startsWith("ANC DIRECT START")) {
    g_gain = read_arg_float(cmd, "GAIN", g_gain);
    g_limit = read_arg_float(cmd, "LIMIT", g_limit);
    g_delay_samples = read_arg_int(cmd, "DELAY", g_delay_samples);
    g_sign = read_arg_int(cmd, "SIGN", g_sign);
    hp_hz = read_arg_float(cmd, "HP", hp_hz);
    lp_hz = read_arg_float(cmd, "LP", lp_hz);
    g_delay_samples = constrain(g_delay_samples, 0, DELAY_BUF_LEN - 1);
    reset_anc_state();
    g_mode = AncMode::DIRECT;
    Serial.println("OK ANC DIRECT STARTED");
    print_status();
    return;
  }

  if (cmd.startsWith("ANC START")) {
    g_gain = read_arg_float(cmd, "GAIN", g_gain);
    g_mu = read_arg_float(cmd, "MU", g_mu);
    g_leak = read_arg_float(cmd, "LEAK", g_leak);
    g_limit = read_arg_float(cmd, "LIMIT", g_limit);
    g_delay_samples = read_arg_int(cmd, "DELAY", g_delay_samples);
    hp_hz = read_arg_float(cmd, "HP", hp_hz);
    lp_hz = read_arg_float(cmd, "LP", lp_hz);
    g_delay_samples = constrain(g_delay_samples, 0, DELAY_BUF_LEN - 1);
    reset_anc_state();
    g_mode = AncMode::FXLMS;
    Serial.println("OK ANC FXLMS STARTED");
    print_status();
    return;
  }

  if (cmd.startsWith("ANC TONE")) {
    float hz = read_arg_float(cmd, "HZ", 1000.0f);
    float amp = read_arg_float(cmd, "AMP", 0.05f);
    int ms = read_arg_int(cmd, "MS", 1000);
    amp = clampf(amp, 0.0f, 0.8f);
    Serial.printf("OK TONE HZ=%.1f AMP=%.3f MS=%d\n", hz, amp, ms);

    int total = (SAMPLE_RATE * ms) / 1000;
    int done = 0;
    float phase = 0.0f;
    float inc = 2.0f * PI * hz / SAMPLE_RATE;
    while (done < total) {
      int frames = min(BLOCK_FRAMES, total - done);
      for (int i = 0; i < frames; ++i) {
        int16_t s = static_cast<int16_t>(sin(phase) * amp * I16_MAX_SAFE);
        phase += inc;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;
        tx_raw[i * 2 + 0] = s;
        tx_raw[i * 2 + 1] = s;
      }
      size_t bw = 0;
      i2s_write(I2S_DAC_PORT, tx_raw, frames * 2 * sizeof(int16_t), &bw, portMAX_DELAY);
      done += frames;
    }
    Serial.println("OK TONE DONE");
    return;
  }

  Serial.print("ERR UNKNOWN COMMAND: ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ANC ESP32-S3 READY");

  bool mic_ok = setup_mic_i2s();
  bool dac_ok = setup_dac_i2s();

  Serial.printf("INIT MIC=%s DAC=%s\n", mic_ok ? "OK" : "FAIL", dac_ok ? "OK" : "FAIL");
  print_status();

  xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        handle_command(line);
        line = "";
      }
    } else {
      line += c;
      if (line.length() > 160) line = "";
    }
  }
}
