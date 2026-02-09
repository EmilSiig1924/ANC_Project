#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

#include <cctype>
#include <cmath>
#include <cstring>

namespace {

// -------------------------
// Platform / I2S constants
// -------------------------

constexpr uint32_t kSampleRateHz = 48000;
constexpr uint32_t kSerialBaud = 115200;
constexpr bool kEnableDebugLogs = true;

// Mic RX I2S (2x INMP441 shared data line, L/R slot selected in hardware)
constexpr i2s_port_t kMicI2sPort = I2S_NUM_1;
constexpr int kMicBclkPin = 14;
constexpr int kMicWsPin = 15;
constexpr int kMicDataPin = 16;

// DAC TX I2S (PCM5102A)
constexpr i2s_port_t kDacI2sPort = I2S_NUM_0;
constexpr int kDacBclkPin = 4;
constexpr int kDacWsPin = 5;
constexpr int kDacDataPin = 6;

// Existing capture protocol settings
constexpr uint16_t kOutputBitsPerSample = 16;
constexpr uint16_t kOutputChannelsMono = 1;
constexpr uint16_t kOutputChannelsStereo = 2;
constexpr uint32_t kDefaultDurationMs = 20000;
constexpr uint32_t kMinDurationMs = 100;
constexpr uint32_t kMaxDurationMs = 60000;
constexpr uint8_t kMicBothId = 0;

// Frame layout for I2S_CHANNEL_FMT_RIGHT_LEFT is [Right, Left]
enum class MicSlot : uint8_t {
  Left = 1,
  Right = 0,
};

// Mapping for REC ... MIC=1|2 and ANC role binding.
// Based on your wiring notes:
// - Mic A strap to LEFT slot = reference mic
// - Mic B strap to RIGHT slot = error mic
constexpr MicSlot kMic1Slot = MicSlot::Left;
constexpr MicSlot kMic2Slot = MicSlot::Right;
constexpr MicSlot kReferenceSlot = MicSlot::Left;
constexpr MicSlot kErrorSlot = MicSlot::Right;

// -------------------------
// ANC defaults (software-only)
// -------------------------

constexpr int kControlFilterLen = 128;
constexpr int kSecondaryFilterLen = 64;
constexpr int kAncFramesPerBlock = 64;
constexpr float kAncEpsilon = 1e-6f;

constexpr uint32_t kAncDefaultCalMs = 3000;
constexpr uint32_t kAncDefaultAdaptMs = 7000;  // adapt then freeze
constexpr float kAncDefaultMu = 0.0020f;
constexpr float kAncDefaultMuCal = 0.12f;
constexpr float kAncDefaultOutputGain = 1.0f;
constexpr float kAncDefaultLimiterDbfs = -6.0f;
constexpr float kAncDefaultCalAmp = 0.20f;
constexpr float kAncDefaultAdaptSign = -1.0f;
constexpr uint32_t kAncDefaultLogMs = 1000;
constexpr uint32_t kStatusMinIntervalMs = 1000;
constexpr uint32_t kSerialCmdIdleFlushMs = 200;

// On-device diagnostic self-test timings
constexpr uint32_t kSelfTestMicWindowMs = 1200;
constexpr uint32_t kSelfTestToneMs = 1200;
constexpr uint32_t kSelfTestCalMs = 1200;
constexpr uint32_t kSelfTestAdaptMs = 2000;
constexpr uint32_t kSelfTestPhaseTimeoutMs = 8000;
constexpr float kSelfTestToneHz = 440.0f;
constexpr float kSelfTestToneAmp = 0.20f;
constexpr float kToneDefaultHz = 1000.0f;
constexpr float kToneDefaultAmp = 0.85f;
constexpr uint32_t kToneDefaultMs = 3000;
constexpr uint32_t kLoopTestDefaultBaseMs = 1000;
constexpr uint32_t kLoopTestDefaultToneMs = 2500;
constexpr float kLoopTestDefaultThresholdDb = 8.0f;
constexpr uint32_t kLoopTestDefaultWaitMs = 1500;

// -------------------------
// Capture protocol structs
// -------------------------

struct RecCommand {
  uint32_t duration_ms = kDefaultDurationMs;
  uint8_t mic_id = 1;
};

#pragma pack(push, 1)
struct CaptureHeader {
  char magic[4];            // "ANC1"
  uint16_t version;         // protocol version
  uint16_t header_bytes;    // sizeof(CaptureHeader)
  uint32_t sample_rate_hz;
  uint32_t duration_ms;
  uint32_t mic_id;
  uint16_t bits_per_sample;
  uint16_t channels;
  uint32_t sample_count;
  uint32_t payload_bytes;
};
#pragma pack(pop)

static_assert(sizeof(CaptureHeader) == 32, "CaptureHeader layout changed unexpectedly");

// -------------------------
// ANC runtime structs
// -------------------------

enum class AncPhase : uint8_t {
  Idle = 0,
  Calibrating,
  Adapting,
  Frozen,
};

struct AncConfig {
  uint32_t cal_ms = kAncDefaultCalMs;
  uint32_t adapt_ms = kAncDefaultAdaptMs;
  float mu = kAncDefaultMu;
  float mu_cal = kAncDefaultMuCal;
  float output_gain = kAncDefaultOutputGain;
  float limiter_dbfs = kAncDefaultLimiterDbfs;
  float cal_excitation_amp = kAncDefaultCalAmp;
  float adapt_sign = kAncDefaultAdaptSign;
  uint32_t log_period_ms = kAncDefaultLogMs;
};

struct AncStats {
  uint64_t total_samples = 0;
  uint64_t phase_samples = 0;
  uint32_t saturated_samples = 0;
  uint32_t blocks = 0;
  uint32_t rx_read_errors = 0;
  uint32_t rx_empty_reads = 0;
  uint32_t tx_write_errors = 0;
  uint32_t tx_short_writes = 0;
  float ema_ref_pow = 0.0f;
  float ema_err_pow = 0.0f;
  float ema_out_pow = 0.0f;
  uint32_t last_log_ms = 0;
};

struct AncRuntime {
  bool active = false;
  AncPhase phase = AncPhase::Idle;
  AncConfig cfg{};
  AncStats stats{};
  uint32_t cal_target_samples = 0;
  uint32_t adapt_target_samples = 0;  // 0 => no auto-freeze
  float limiter_linear = 0.5f;
  uint32_t noise_state = 0x1f123bb5u;

  // FxLMS control path state
  float w[kControlFilterLen]{};
  float x_ring[kControlFilterLen]{};
  int x_head = 0;

  float xf_ring[kControlFilterLen]{};
  int xf_head = 0;
  float xf_energy = kAncEpsilon;

  // Secondary path estimate (calibration)
  float s_hat[kSecondaryFilterLen]{};
  float cal_u_ring[kSecondaryFilterLen]{};
  int cal_u_head = 0;
};

// -------------------------
// Global state
// -------------------------

bool g_capture_rx_installed = false;
bool g_anc_rx_installed = false;
bool g_anc_tx_installed = false;
String g_serial_line;
uint32_t g_serial_last_rx_ms = 0;
AncRuntime g_anc;

// -------------------------
// Utility helpers
// -------------------------

void log_debug(const String& message) {
  if (!kEnableDebugLogs) {
    return;
  }
  Serial.println("DBG " + message);
}

const char* anc_phase_name(AncPhase phase) {
  switch (phase) {
    case AncPhase::Idle:
      return "IDLE";
    case AncPhase::Calibrating:
      return "CALIBRATING";
    case AncPhase::Adapting:
      return "ADAPTING";
    case AncPhase::Frozen:
      return "FROZEN";
  }
  return "UNKNOWN";
}

const char* reset_reason_name(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:
      return "UNKNOWN";
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXT";
    case ESP_RST_SW:
      return "SW";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      return "OTHER";
  }
}

float dbfs_to_linear(float dbfs) {
  return powf(10.0f, dbfs / 20.0f);
}

float power_to_dbfs(float power) {
  if (power <= 1e-20f) {
    return -200.0f;
  }
  return 10.0f * log10f(power);
}

const char* yes_no(bool v) {
  return v ? "YES" : "NO";
}

inline int slot_index_in_frame(MicSlot slot) {
  return (slot == MicSlot::Right) ? 0 : 1;
}

inline bool is_dual_mic_capture(uint8_t mic_id) {
  return mic_id == kMicBothId;
}

inline int16_t decode_i2s_32_to_i16(int32_t sample_32) {
  // INMP441 provides 24-bit data left-aligned in a 32-bit frame.
  int32_t v = sample_32 >> 14;
  if (v > 32767) {
    v = 32767;
  } else if (v < -32768) {
    v = -32768;
  }
  return static_cast<int16_t>(v);
}

inline float pcm16_to_float(int16_t v) {
  return static_cast<float>(v) / 32768.0f;
}

inline int16_t float_to_pcm16(float x) {
  float y = x;
  if (y > 0.99997f) {
    y = 0.99997f;
  } else if (y < -1.0f) {
    y = -1.0f;
  }
  return static_cast<int16_t>(lrintf(y * 32767.0f));
}

template <size_t N>
inline void ring_push(float (&ring)[N], int& head, float value) {
  head = (head - 1 + static_cast<int>(N)) % static_cast<int>(N);
  ring[head] = value;
}

template <size_t N>
inline float ring_get(const float (&ring)[N], int head, int tap) {
  return ring[(head + tap) % static_cast<int>(N)];
}

template <size_t N>
float ring_energy(const float (&ring)[N], int head) {
  float acc = 0.0f;
  for (int i = 0; i < static_cast<int>(N); ++i) {
    const float v = ring_get(ring, head, i);
    acc += v * v;
  }
  return acc;
}

float xorshift_noise(uint32_t* state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  const int32_t centered = static_cast<int32_t>(x & 0xFFFFu) - 32768;
  return static_cast<float>(centered) / 32768.0f;
}

float apply_soft_limiter(float sample, float threshold_linear, bool* saturated) {
  const float a = fabsf(sample);
  if (a <= threshold_linear) {
    return sample;
  }
  if (saturated != nullptr) {
    *saturated = true;
  }
  const float safe_span = fmaxf(1.0f - threshold_linear, 1e-6f);
  const float excess = (a - threshold_linear) / safe_span;
  const float curved = threshold_linear + (1.0f - threshold_linear) * (1.0f - expf(-2.5f * excess));
  return copysignf(fminf(curved, 1.0f), sample);
}

bool extract_option_value(const String& upper_line, const char* key, String* out_value) {
  const String pattern = String(key) + "=";
  const int idx = upper_line.indexOf(pattern);
  if (idx < 0) {
    return false;
  }
  int start = idx + pattern.length();
  int end = start;
  while (end < upper_line.length() &&
         std::isspace(static_cast<unsigned char>(upper_line[end])) == 0) {
    ++end;
  }
  *out_value = upper_line.substring(start, end);
  return true;
}

bool parse_u32_option(const String& upper_line, const char* key, uint32_t* out) {
  String s;
  if (!extract_option_value(upper_line, key, &s)) {
    return false;
  }
  const long v = s.toInt();
  if (v < 0) {
    return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

bool parse_float_option(const String& upper_line, const char* key, float* out) {
  String s;
  if (!extract_option_value(upper_line, key, &s)) {
    return false;
  }
  *out = s.toFloat();
  return true;
}

// -------------------------
// I2S management
// -------------------------

void uninstall_capture_rx_if_installed() {
  if (!g_capture_rx_installed) {
    return;
  }
  i2s_driver_uninstall(kMicI2sPort);
  g_capture_rx_installed = false;
}

void uninstall_anc_stream_if_installed() {
  if (g_anc_rx_installed) {
    i2s_driver_uninstall(kMicI2sPort);
    g_anc_rx_installed = false;
  }
  if (g_anc_tx_installed) {
    i2s_driver_uninstall(kDacI2sPort);
    g_anc_tx_installed = false;
  }
}

bool configure_capture_rx_i2s(String* err) {
  uninstall_anc_stream_if_installed();
  uninstall_capture_rx_if_installed();

  i2s_config_t cfg{};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = kSampleRateHz;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  esp_err_t esp = i2s_driver_install(kMicI2sPort, &cfg, 0, nullptr);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "i2s_driver_install RX failed";
    }
    return false;
  }

  i2s_pin_config_t pin_cfg{};
  pin_cfg.bck_io_num = kMicBclkPin;
  pin_cfg.ws_io_num = kMicWsPin;
  pin_cfg.data_out_num = I2S_PIN_NO_CHANGE;
  pin_cfg.data_in_num = kMicDataPin;

  esp = i2s_set_pin(kMicI2sPort, &pin_cfg);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "i2s_set_pin RX failed";
    }
    i2s_driver_uninstall(kMicI2sPort);
    return false;
  }

  esp = i2s_set_clk(kMicI2sPort, kSampleRateHz, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "i2s_set_clk RX failed";
    }
    i2s_driver_uninstall(kMicI2sPort);
    return false;
  }

  i2s_zero_dma_buffer(kMicI2sPort);
  g_capture_rx_installed = true;
  return true;
}

bool configure_anc_stream(String* err) {
  uninstall_capture_rx_if_installed();
  uninstall_anc_stream_if_installed();

  // RX (mics)
  i2s_config_t rx_cfg{};
  rx_cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  rx_cfg.sample_rate = kSampleRateHz;
  rx_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  rx_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  rx_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  rx_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  rx_cfg.dma_buf_count = 8;
  rx_cfg.dma_buf_len = 256;
  rx_cfg.use_apll = false;
  rx_cfg.tx_desc_auto_clear = false;
  rx_cfg.fixed_mclk = 0;

  esp_err_t esp = i2s_driver_install(kMicI2sPort, &rx_cfg, 0, nullptr);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC RX i2s_driver_install failed";
    }
    return false;
  }
  g_anc_rx_installed = true;

  i2s_pin_config_t rx_pins{};
  rx_pins.bck_io_num = kMicBclkPin;
  rx_pins.ws_io_num = kMicWsPin;
  rx_pins.data_out_num = I2S_PIN_NO_CHANGE;
  rx_pins.data_in_num = kMicDataPin;

  esp = i2s_set_pin(kMicI2sPort, &rx_pins);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC RX i2s_set_pin failed";
    }
    uninstall_anc_stream_if_installed();
    return false;
  }

  esp = i2s_set_clk(kMicI2sPort, kSampleRateHz, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC RX i2s_set_clk failed";
    }
    uninstall_anc_stream_if_installed();
    return false;
  }

  // TX (DAC)
  i2s_config_t tx_cfg{};
  tx_cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  tx_cfg.sample_rate = kSampleRateHz;
  tx_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  tx_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  tx_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  tx_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  tx_cfg.dma_buf_count = 8;
  tx_cfg.dma_buf_len = 256;
  tx_cfg.use_apll = false;
  tx_cfg.tx_desc_auto_clear = true;
  tx_cfg.fixed_mclk = 0;

  esp = i2s_driver_install(kDacI2sPort, &tx_cfg, 0, nullptr);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC TX i2s_driver_install failed";
    }
    uninstall_anc_stream_if_installed();
    return false;
  }
  g_anc_tx_installed = true;

  i2s_pin_config_t tx_pins{};
  tx_pins.bck_io_num = kDacBclkPin;
  tx_pins.ws_io_num = kDacWsPin;
  tx_pins.data_out_num = kDacDataPin;
  tx_pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp = i2s_set_pin(kDacI2sPort, &tx_pins);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC TX i2s_set_pin failed";
    }
    uninstall_anc_stream_if_installed();
    return false;
  }

  esp = i2s_set_clk(kDacI2sPort, kSampleRateHz, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (esp != ESP_OK) {
    if (err != nullptr) {
      *err = "ANC TX i2s_set_clk failed";
    }
    uninstall_anc_stream_if_installed();
    return false;
  }

  i2s_zero_dma_buffer(kMicI2sPort);
  i2s_zero_dma_buffer(kDacI2sPort);
  return true;
}

void warm_up_mic_i2s() {
  constexpr size_t kDiscardWords = 256;
  int32_t discard[kDiscardWords];
  size_t bytes_read = 0;
  for (int i = 0; i < 5; ++i) {
    (void)i2s_read(
        kMicI2sPort,
        discard,
        sizeof(discard),
        &bytes_read,
        pdMS_TO_TICKS(100));
  }
}

// -------------------------
// Existing REC command path
// -------------------------

bool parse_rec_command(const String& line, RecCommand* out, String* err) {
  String s = line;
  s.trim();
  if (s.isEmpty()) {
    return false;
  }

  String upper = s;
  upper.toUpperCase();
  if (!upper.startsWith("REC")) {
    if (err != nullptr) {
      *err = "Unknown command. Expected: REC <ms> MIC=<0|1|2>";
    }
    return false;
  }

  RecCommand cmd;
  int i = 3;
  while (i < s.length() && std::isspace(static_cast<unsigned char>(s[i])) != 0) {
    ++i;
  }

  if (i < s.length() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) {
    const int start = i;
    while (i < s.length() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) {
      ++i;
    }
    cmd.duration_ms = static_cast<uint32_t>(s.substring(start, i).toInt());
  }

  while (i < s.length()) {
    while (i < s.length() && std::isspace(static_cast<unsigned char>(s[i])) != 0) {
      ++i;
    }
    if (i >= s.length()) {
      break;
    }

    const int start = i;
    while (i < s.length() && std::isspace(static_cast<unsigned char>(s[i])) == 0) {
      ++i;
    }
    String token = s.substring(start, i);
    String token_upper = token;
    token_upper.toUpperCase();

    if (token_upper.startsWith("MIC=")) {
      const int mic = token.substring(4).toInt();
      cmd.mic_id = static_cast<uint8_t>(mic);
      continue;
    }

    if (err != nullptr) {
      *err = "Unknown option token: " + token;
    }
    return false;
  }

  if (cmd.duration_ms < kMinDurationMs || cmd.duration_ms > kMaxDurationMs) {
    if (err != nullptr) {
      *err = "Duration out of range. Use 100..60000 ms.";
    }
    return false;
  }

  if (!(cmd.mic_id == 0 || cmd.mic_id == 1 || cmd.mic_id == 2)) {
    if (err != nullptr) {
      *err = "Mic must be 0, 1, or 2.";
    }
    return false;
  }

  *out = cmd;
  return true;
}

MicSlot mic_slot_for_id(uint8_t mic_id) {
  if (mic_id == 1) {
    return kMic1Slot;
  }
  return kMic2Slot;
}

bool capture_to_psram(
    const RecCommand& cmd,
    int16_t** out_pcm,
    uint32_t* out_sample_count,
    uint16_t* out_channels,
    String* err) {
  if (!psramFound()) {
    if (err != nullptr) {
      *err = "PSRAM not detected. Check board config.";
    }
    return false;
  }

  if (!configure_capture_rx_i2s(err)) {
    return false;
  }

  const uint64_t sample_count_64 =
      (static_cast<uint64_t>(cmd.duration_ms) * static_cast<uint64_t>(kSampleRateHz)) / 1000ULL;
  if (sample_count_64 == 0 || sample_count_64 > 0xFFFFFFFFULL) {
    if (err != nullptr) {
      *err = "Invalid sample count.";
    }
    uninstall_capture_rx_if_installed();
    return false;
  }

  const uint32_t sample_count = static_cast<uint32_t>(sample_count_64);
  const uint16_t channels = is_dual_mic_capture(cmd.mic_id) ? kOutputChannelsStereo : kOutputChannelsMono;
  const size_t payload_bytes = static_cast<size_t>(sample_count) * channels * sizeof(int16_t);
  int16_t* pcm = static_cast<int16_t*>(
      heap_caps_malloc(payload_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (pcm == nullptr) {
    if (err != nullptr) {
      *err = "PSRAM allocation failed.";
    }
    uninstall_capture_rx_if_installed();
    return false;
  }

  warm_up_mic_i2s();

  constexpr size_t kReadFrames = 128;
  constexpr size_t kWordsPerFrame = 2;
  int32_t rx32[kReadFrames * kWordsPerFrame];
  uint32_t written = 0;

  const bool dual_capture = is_dual_mic_capture(cmd.mic_id);
  const int selected_idx = slot_index_in_frame(mic_slot_for_id(cmd.mic_id));
  const int mic1_idx = slot_index_in_frame(kMic1Slot);
  const int mic2_idx = slot_index_in_frame(kMic2Slot);

  while (written < sample_count) {
    const uint32_t remaining_samples = sample_count - written;
    const size_t wanted_frames = (remaining_samples < kReadFrames) ? remaining_samples : kReadFrames;

    size_t bytes_read = 0;
    const esp_err_t esp = i2s_read(
        kMicI2sPort,
        rx32,
        wanted_frames * kWordsPerFrame * sizeof(int32_t),
        &bytes_read,
        pdMS_TO_TICKS(1000));
    if (esp != ESP_OK) {
      if (err != nullptr) {
        *err = "i2s_read failed.";
      }
      heap_caps_free(pcm);
      uninstall_capture_rx_if_installed();
      return false;
    }

    const size_t got_words = bytes_read / sizeof(int32_t);
    const size_t got_frames = got_words / kWordsPerFrame;
    if (got_frames == 0) {
      if (err != nullptr) {
        *err = "i2s_read timeout.";
      }
      heap_caps_free(pcm);
      uninstall_capture_rx_if_installed();
      return false;
    }

    for (size_t i = 0; i < got_frames; ++i) {
      if (dual_capture) {
        const int16_t v1 = decode_i2s_32_to_i16(rx32[(i * kWordsPerFrame) + mic1_idx]);
        const int16_t v2 = decode_i2s_32_to_i16(rx32[(i * kWordsPerFrame) + mic2_idx]);
        const size_t dst = static_cast<size_t>(written + i) * 2U;
        pcm[dst] = v1;
        pcm[dst + 1] = v2;
      } else {
        pcm[written + i] = decode_i2s_32_to_i16(rx32[(i * kWordsPerFrame) + selected_idx]);
      }
    }
    written += static_cast<uint32_t>(got_frames);
  }

  uninstall_capture_rx_if_installed();
  *out_pcm = pcm;
  *out_sample_count = sample_count;
  *out_channels = channels;
  return true;
}

void send_capture_binary(const RecCommand& cmd, const int16_t* pcm, uint32_t sample_count, uint16_t channels) {
  CaptureHeader header{};
  header.magic[0] = 'A';
  header.magic[1] = 'N';
  header.magic[2] = 'C';
  header.magic[3] = '1';
  header.version = 1;
  header.header_bytes = sizeof(CaptureHeader);
  header.sample_rate_hz = kSampleRateHz;
  header.duration_ms = cmd.duration_ms;
  header.mic_id = cmd.mic_id;
  header.bits_per_sample = kOutputBitsPerSample;
  header.channels = channels;
  header.sample_count = sample_count;
  header.payload_bytes = sample_count * channels * sizeof(int16_t);

  Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));

  const uint8_t* payload = reinterpret_cast<const uint8_t*>(pcm);
  size_t remaining = header.payload_bytes;
  while (remaining > 0) {
    const size_t chunk = (remaining < 4096U) ? remaining : 4096U;
    Serial.write(payload, chunk);
    payload += chunk;
    remaining -= chunk;
  }
  Serial.flush();
}

// -------------------------
// ANC processing
// -------------------------

void anc_reset_state(bool clear_secondary) {
  memset(g_anc.w, 0, sizeof(g_anc.w));
  memset(g_anc.x_ring, 0, sizeof(g_anc.x_ring));
  memset(g_anc.xf_ring, 0, sizeof(g_anc.xf_ring));
  memset(g_anc.cal_u_ring, 0, sizeof(g_anc.cal_u_ring));
  if (clear_secondary) {
    memset(g_anc.s_hat, 0, sizeof(g_anc.s_hat));
  }
  g_anc.x_head = 0;
  g_anc.xf_head = 0;
  g_anc.cal_u_head = 0;
  g_anc.xf_energy = kAncEpsilon;
  g_anc.stats = AncStats{};
  g_anc.stats.last_log_ms = millis();
  g_anc.noise_state = 0x1f123bb5u;
}

float anc_dot_control(const float* coeffs, const float (&ring)[kControlFilterLen], int head) {
  float y = 0.0f;
  for (int k = 0; k < kControlFilterLen; ++k) {
    y += coeffs[k] * ring_get(ring, head, k);
  }
  return y;
}

float anc_dot_secondary_from_x(const float* coeffs, const float (&x_ring)[kControlFilterLen], int x_head) {
  float y = 0.0f;
  for (int k = 0; k < kSecondaryFilterLen; ++k) {
    y += coeffs[k] * ring_get(x_ring, x_head, k);
  }
  return y;
}

float anc_dot_secondary_from_u(const float* coeffs, const float (&u_ring)[kSecondaryFilterLen], int u_head) {
  float y = 0.0f;
  for (int k = 0; k < kSecondaryFilterLen; ++k) {
    y += coeffs[k] * ring_get(u_ring, u_head, k);
  }
  return y;
}

float anc_secondary_norm() {
  float acc = 0.0f;
  for (int i = 0; i < kSecondaryFilterLen; ++i) {
    acc += g_anc.s_hat[i] * g_anc.s_hat[i];
  }
  return sqrtf(acc);
}

float anc_control_norm() {
  float acc = 0.0f;
  for (int i = 0; i < kControlFilterLen; ++i) {
    acc += g_anc.w[i] * g_anc.w[i];
  }
  return sqrtf(acc);
}

bool validate_anc_config(const AncConfig& cfg, String* err) {
  if (cfg.cal_ms > 120000U) {
    if (err != nullptr) {
      *err = "CAL_MS too large (max 120000).";
    }
    return false;
  }
  if (cfg.adapt_ms > 600000U) {
    if (err != nullptr) {
      *err = "ADAPT_MS too large (max 600000).";
    }
    return false;
  }
  if (!(cfg.mu > 0.0f && cfg.mu <= 0.1f)) {
    if (err != nullptr) {
      *err = "MU must be in (0, 0.1].";
    }
    return false;
  }
  if (!(cfg.mu_cal > 0.0f && cfg.mu_cal <= 1.0f)) {
    if (err != nullptr) {
      *err = "MUCAL must be in (0, 1].";
    }
    return false;
  }
  if (!(cfg.output_gain >= 0.0f && cfg.output_gain <= 2.0f)) {
    if (err != nullptr) {
      *err = "GAIN must be in [0, 2].";
    }
    return false;
  }
  if (!(cfg.cal_excitation_amp > 0.0f && cfg.cal_excitation_amp <= 0.8f)) {
    if (err != nullptr) {
      *err = "CAL_AMP must be in (0, 0.8].";
    }
    return false;
  }
  if (!(cfg.limiter_dbfs <= -1.0f && cfg.limiter_dbfs >= -40.0f)) {
    if (err != nullptr) {
      *err = "LIMIT_DBFS must be between -40 and -1.";
    }
    return false;
  }
  if (cfg.log_period_ms > 0U &&
      (cfg.log_period_ms < 100U || cfg.log_period_ms > 60000U)) {
    if (err != nullptr) {
      *err = "LOG_MS must be 0 (off) or in [100, 60000].";
    }
    return false;
  }
  if (!(cfg.adapt_sign == 1.0f || cfg.adapt_sign == -1.0f)) {
    if (err != nullptr) {
      *err = "SIGN must be +1 or -1.";
    }
    return false;
  }
  return true;
}

void anc_stop() {
  if (!g_anc.active) {
    return;
  }
  uninstall_anc_stream_if_installed();
  g_anc.active = false;
  g_anc.phase = AncPhase::Idle;
  log_debug("ANC stopped");
}

void anc_set_phase(AncPhase phase) {
  g_anc.phase = phase;
  g_anc.stats.phase_samples = 0;
  if (phase == AncPhase::Calibrating) {
    Serial.println("OK ANC_PHASE=CALIBRATING");
  } else if (phase == AncPhase::Adapting) {
    Serial.println("OK ANC_PHASE=ADAPTING");
  } else if (phase == AncPhase::Frozen) {
    Serial.println("OK ANC_PHASE=FROZEN");
  } else {
    Serial.println("OK ANC_PHASE=IDLE");
  }
}

bool anc_start(const AncConfig& cfg, String* err) {
  if (!validate_anc_config(cfg, err)) {
    return false;
  }

  if (!configure_anc_stream(err)) {
    return false;
  }

  warm_up_mic_i2s();
  anc_reset_state(cfg.cal_ms > 0);
  g_anc.cfg = cfg;
  g_anc.limiter_linear = dbfs_to_linear(cfg.limiter_dbfs);
  g_anc.cal_target_samples = static_cast<uint32_t>((static_cast<uint64_t>(cfg.cal_ms) * kSampleRateHz) / 1000ULL);
  g_anc.adapt_target_samples = static_cast<uint32_t>((static_cast<uint64_t>(cfg.adapt_ms) * kSampleRateHz) / 1000ULL);
  g_anc.active = true;

  if (g_anc.cal_target_samples > 0U) {
    anc_set_phase(AncPhase::Calibrating);
  } else {
    // If no calibration requested, keep any existing s_hat and start adapting.
    anc_set_phase(AncPhase::Adapting);
  }

  Serial.println(String("OK ANC_STARTED CAL_MS=") + String(cfg.cal_ms) +
                 " ADAPT_MS=" + String(cfg.adapt_ms) +
                 " MU=" + String(cfg.mu, 6) +
                 " MUCAL=" + String(cfg.mu_cal, 6) +
                 " GAIN=" + String(cfg.output_gain, 4) +
                 " LIMIT_DBFS=" + String(cfg.limiter_dbfs, 2) +
                 " CAL_AMP=" + String(cfg.cal_excitation_amp, 3) +
                 " SIGN=" + String(cfg.adapt_sign, 1) +
                 " LOG_MS=" + String(cfg.log_period_ms));
  return true;
}

void anc_maybe_log() {
  if (!g_anc.active) {
    return;
  }
  if (g_anc.cfg.log_period_ms == 0U) {
    return;
  }
  const uint32_t now = millis();
  if (now - g_anc.stats.last_log_ms < g_anc.cfg.log_period_ms) {
    return;
  }
  g_anc.stats.last_log_ms = now;

  const float ref_db = power_to_dbfs(g_anc.stats.ema_ref_pow);
  const float err_db = power_to_dbfs(g_anc.stats.ema_err_pow);
  const float out_db = power_to_dbfs(g_anc.stats.ema_out_pow);
  const float w_norm = anc_control_norm();
  const float s_norm = anc_secondary_norm();

  Serial.println(
      String("LOG ANC PHASE=") + anc_phase_name(g_anc.phase) +
      " REF_RMS_DBFS=" + String(ref_db, 2) +
      " ERR_RMS_DBFS=" + String(err_db, 2) +
      " OUT_RMS_DBFS=" + String(out_db, 2) +
      " SAT=" + String(g_anc.stats.saturated_samples) +
      " RXERR=" + String(g_anc.stats.rx_read_errors) +
      " RXEMPTY=" + String(g_anc.stats.rx_empty_reads) +
      " TXERR=" + String(g_anc.stats.tx_write_errors) +
      " TXSHORT=" + String(g_anc.stats.tx_short_writes) +
      " WNORM=" + String(w_norm, 4) +
      " SNORM=" + String(s_norm, 4) +
      " TOTAL_SAMPLES=" + String(static_cast<uint32_t>(g_anc.stats.total_samples & 0xFFFFFFFFULL)));
}

void anc_transition_if_needed() {
  if (!g_anc.active) {
    return;
  }

  if (g_anc.phase == AncPhase::Calibrating &&
      g_anc.cal_target_samples > 0 &&
      g_anc.stats.phase_samples >= g_anc.cal_target_samples) {
    // Zero control filter state before adaptation starts.
    memset(g_anc.w, 0, sizeof(g_anc.w));
    memset(g_anc.x_ring, 0, sizeof(g_anc.x_ring));
    memset(g_anc.xf_ring, 0, sizeof(g_anc.xf_ring));
    g_anc.x_head = 0;
    g_anc.xf_head = 0;
    g_anc.xf_energy = kAncEpsilon;
    anc_set_phase(AncPhase::Adapting);
    return;
  }

  if (g_anc.phase == AncPhase::Adapting &&
      g_anc.adapt_target_samples > 0 &&
      g_anc.stats.phase_samples >= g_anc.adapt_target_samples) {
    anc_set_phase(AncPhase::Frozen);
  }
}

void anc_process_block() {
  if (!g_anc.active) {
    return;
  }
  if (!g_anc_rx_installed || !g_anc_tx_installed) {
    return;
  }

  static int32_t rx_buf[kAncFramesPerBlock * 2];
  static int16_t tx_buf[kAncFramesPerBlock * 2];

  size_t bytes_read = 0;
  const esp_err_t read_err = i2s_read(
      kMicI2sPort,
      rx_buf,
      sizeof(rx_buf),
      &bytes_read,
      pdMS_TO_TICKS(5));
  if (read_err != ESP_OK) {
    g_anc.stats.rx_read_errors++;
    return;
  }
  if (bytes_read == 0) {
    g_anc.stats.rx_empty_reads++;
    return;
  }

  const size_t words = bytes_read / sizeof(int32_t);
  const size_t frames = words / 2;
  const int ref_idx = slot_index_in_frame(kReferenceSlot);
  const int err_idx = slot_index_in_frame(kErrorSlot);

  constexpr float ema_alpha = 0.002f;

  for (size_t i = 0; i < frames; ++i) {
    const int32_t right_w = rx_buf[i * 2 + 0];
    const int32_t left_w = rx_buf[i * 2 + 1];
    const int16_t slots[2] = {
        decode_i2s_32_to_i16(right_w),
        decode_i2s_32_to_i16(left_w),
    };

    const float ref = pcm16_to_float(slots[ref_idx]);
    const float err = pcm16_to_float(slots[err_idx]);
    float out = 0.0f;

    if (g_anc.phase == AncPhase::Calibrating) {
      // Identify secondary path: d ~= s_hat * u
      const float u = xorshift_noise(&g_anc.noise_state) * g_anc.cfg.cal_excitation_amp;
      out = u;

      ring_push(g_anc.cal_u_ring, g_anc.cal_u_head, u);
      const float pred = anc_dot_secondary_from_u(g_anc.s_hat, g_anc.cal_u_ring, g_anc.cal_u_head);
      const float id_err = err - pred;
      const float norm = kAncEpsilon + ring_energy(g_anc.cal_u_ring, g_anc.cal_u_head);
      const float step = g_anc.cfg.mu_cal * id_err / norm;
      for (int k = 0; k < kSecondaryFilterLen; ++k) {
        g_anc.s_hat[k] += step * ring_get(g_anc.cal_u_ring, g_anc.cal_u_head, k);
      }
    } else {
      // FxLMS control path
      ring_push(g_anc.x_ring, g_anc.x_head, ref);

      const float y = anc_dot_control(g_anc.w, g_anc.x_ring, g_anc.x_head);
      out = y * g_anc.cfg.output_gain;

      bool saturated = false;
      out = apply_soft_limiter(out, g_anc.limiter_linear, &saturated);
      if (saturated) {
        g_anc.stats.saturated_samples++;
      }

      if (g_anc.phase == AncPhase::Adapting) {
        const float xf_now = anc_dot_secondary_from_x(g_anc.s_hat, g_anc.x_ring, g_anc.x_head);

        const int new_head = (g_anc.xf_head - 1 + kControlFilterLen) % kControlFilterLen;
        const float old_xf = g_anc.xf_ring[new_head];
        g_anc.xf_ring[new_head] = xf_now;
        g_anc.xf_head = new_head;
        g_anc.xf_energy += (xf_now * xf_now) - (old_xf * old_xf);
        if (g_anc.xf_energy < kAncEpsilon) {
          g_anc.xf_energy = kAncEpsilon;
        }

        const float step = (g_anc.cfg.mu * g_anc.cfg.adapt_sign * err) / g_anc.xf_energy;
        for (int k = 0; k < kControlFilterLen; ++k) {
          g_anc.w[k] += step * ring_get(g_anc.xf_ring, g_anc.xf_head, k);
        }
      }
    }

    const int16_t out_i16 = float_to_pcm16(out);
    tx_buf[i * 2 + 0] = out_i16;
    tx_buf[i * 2 + 1] = out_i16;

    g_anc.stats.total_samples++;
    g_anc.stats.phase_samples++;
    g_anc.stats.ema_ref_pow = (1.0f - ema_alpha) * g_anc.stats.ema_ref_pow + ema_alpha * (ref * ref);
    g_anc.stats.ema_err_pow = (1.0f - ema_alpha) * g_anc.stats.ema_err_pow + ema_alpha * (err * err);
    g_anc.stats.ema_out_pow = (1.0f - ema_alpha) * g_anc.stats.ema_out_pow + ema_alpha * (out * out);
  }

  size_t bytes_written = 0;
  const size_t need_bytes = frames * 2 * sizeof(int16_t);
  const esp_err_t write_err = i2s_write(
      kDacI2sPort,
      tx_buf,
      need_bytes,
      &bytes_written,
      pdMS_TO_TICKS(5));
  if (write_err != ESP_OK) {
    g_anc.stats.tx_write_errors++;
  } else if (bytes_written != need_bytes) {
    g_anc.stats.tx_short_writes++;
  }

  g_anc.stats.blocks++;
  anc_transition_if_needed();
  anc_maybe_log();
}

// -------------------------
// Self-test
// -------------------------

struct SelfTestMicMetrics {
  float ref_rms_dbfs = -200.0f;
  float err_rms_dbfs = -200.0f;
  int32_t ref_peak = 0;
  int32_t err_peak = 0;
  uint32_t frames = 0;
  bool ref_active = false;
  bool err_active = false;
};

bool measure_mics_window_ms(uint32_t window_ms, SelfTestMicMetrics* out, String* err) {
  if (window_ms < 100U || window_ms > 60000U) {
    if (err != nullptr) {
      *err = "Window ms out of range [100, 60000].";
    }
    return false;
  }
  if (!g_anc_rx_installed) {
    if (err != nullptr) {
      *err = "ANC RX stream not installed.";
    }
    return false;
  }

  constexpr size_t kReadFrames = 128;
  int32_t rx_buf[kReadFrames * 2];
  const uint32_t target_frames =
      static_cast<uint32_t>((static_cast<uint64_t>(window_ms) * kSampleRateHz) / 1000ULL);
  const int ref_idx = slot_index_in_frame(kReferenceSlot);
  const int err_idx = slot_index_in_frame(kErrorSlot);

  uint32_t frames = 0;
  double sum_ref = 0.0;
  double sum_err = 0.0;
  int32_t ref_peak = 0;
  int32_t err_peak = 0;

  while (frames < target_frames) {
    const uint32_t remaining = target_frames - frames;
    const size_t wanted_frames = (remaining < kReadFrames) ? remaining : kReadFrames;

    size_t bytes_read = 0;
    const esp_err_t esp = i2s_read(
        kMicI2sPort,
        rx_buf,
        wanted_frames * 2U * sizeof(int32_t),
        &bytes_read,
        pdMS_TO_TICKS(250));
    if (esp != ESP_OK) {
      if (err != nullptr) {
        *err = "i2s_read failed during mic self-test.";
      }
      return false;
    }

    const size_t got_words = bytes_read / sizeof(int32_t);
    const size_t got_frames = got_words / 2U;
    if (got_frames == 0) {
      continue;
    }

    for (size_t i = 0; i < got_frames; ++i) {
      const int32_t right_w = rx_buf[i * 2 + 0];
      const int32_t left_w = rx_buf[i * 2 + 1];
      const int16_t slots[2] = {
          decode_i2s_32_to_i16(right_w),
          decode_i2s_32_to_i16(left_w),
      };

      const int16_t ref_i16 = slots[ref_idx];
      const int16_t err_i16 = slots[err_idx];
      const float ref = pcm16_to_float(ref_i16);
      const float errf = pcm16_to_float(err_i16);

      sum_ref += static_cast<double>(ref) * static_cast<double>(ref);
      sum_err += static_cast<double>(errf) * static_cast<double>(errf);

      const int32_t ref_abs = (ref_i16 >= 0) ? ref_i16 : -static_cast<int32_t>(ref_i16);
      const int32_t err_abs = (err_i16 >= 0) ? err_i16 : -static_cast<int32_t>(err_i16);
      if (ref_abs > ref_peak) {
        ref_peak = ref_abs;
      }
      if (err_abs > err_peak) {
        err_peak = err_abs;
      }
    }

    frames += static_cast<uint32_t>(got_frames);
  }

  if (frames == 0U) {
    if (err != nullptr) {
      *err = "No frames captured during mic self-test.";
    }
    return false;
  }

  out->frames = frames;
  out->ref_peak = ref_peak;
  out->err_peak = err_peak;
  out->ref_rms_dbfs = power_to_dbfs(static_cast<float>(sum_ref / static_cast<double>(frames)));
  out->err_rms_dbfs = power_to_dbfs(static_cast<float>(sum_err / static_cast<double>(frames)));
  out->ref_active = (ref_peak > 64) || (out->ref_rms_dbfs > -100.0f);
  out->err_active = (err_peak > 64) || (out->err_rms_dbfs > -100.0f);
  return true;
}

bool selftest_measure_mics(SelfTestMicMetrics* out, String* err) {
  return measure_mics_window_ms(kSelfTestMicWindowMs, out, err);
}

bool play_dac_tone(
    float tone_hz,
    float tone_amp,
    uint32_t tone_ms,
    String* err,
    uint32_t* out_bytes,
    float* out_rms_dbfs) {
  if (!(tone_hz >= 20.0f && tone_hz <= 20000.0f)) {
    if (err != nullptr) {
      *err = "TONE HZ must be in [20, 20000].";
    }
    return false;
  }
  if (!(tone_amp > 0.0f && tone_amp <= 0.95f)) {
    if (err != nullptr) {
      *err = "TONE AMP must be in (0, 0.95].";
    }
    return false;
  }
  if (tone_ms < 100U || tone_ms > 60000U) {
    if (err != nullptr) {
      *err = "TONE MS must be in [100, 60000].";
    }
    return false;
  }

  if (!g_anc_tx_installed) {
    if (err != nullptr) {
      *err = "ANC TX stream not installed.";
    }
    return false;
  }

  constexpr size_t kFramesPerChunk = 128;
  int16_t tx_buf[kFramesPerChunk * 2];

  const uint32_t target_frames =
      static_cast<uint32_t>((static_cast<uint64_t>(tone_ms) * kSampleRateHz) / 1000ULL);
  const float phase_inc = 2.0f * static_cast<float>(M_PI) * tone_hz / static_cast<float>(kSampleRateHz);
  float phase = 0.0f;

  uint32_t sent_frames = 0;
  uint32_t total_bytes = 0;
  double sum_sq = 0.0;

  while (sent_frames < target_frames) {
    const uint32_t remaining = target_frames - sent_frames;
    const size_t frames = (remaining < kFramesPerChunk) ? remaining : kFramesPerChunk;

    for (size_t i = 0; i < frames; ++i) {
      const float s = sinf(phase) * tone_amp;
      phase += phase_inc;
      if (phase > 2.0f * static_cast<float>(M_PI)) {
        phase -= 2.0f * static_cast<float>(M_PI);
      }
      const int16_t v = float_to_pcm16(s);
      tx_buf[i * 2 + 0] = v;
      tx_buf[i * 2 + 1] = v;
      sum_sq += static_cast<double>(s) * static_cast<double>(s);
    }

    size_t bytes_written = 0;
    const size_t need_bytes = frames * 2U * sizeof(int16_t);
    const esp_err_t esp = i2s_write(
        kDacI2sPort,
        tx_buf,
        need_bytes,
        &bytes_written,
        pdMS_TO_TICKS(250));
    if (esp != ESP_OK) {
      if (err != nullptr) {
        *err = "i2s_write failed during DAC self-test.";
      }
      return false;
    }
    if (bytes_written != need_bytes) {
      if (err != nullptr) {
        *err = "Short i2s_write during DAC self-test.";
      }
      return false;
    }

    total_bytes += static_cast<uint32_t>(bytes_written);
    sent_frames += static_cast<uint32_t>(frames);
  }

  if (out_bytes != nullptr) {
    *out_bytes = total_bytes;
  }
  if (out_rms_dbfs != nullptr) {
    const float mean_power = (sent_frames > 0U) ?
                                 static_cast<float>(sum_sq / static_cast<double>(sent_frames)) :
                                 0.0f;
    *out_rms_dbfs = power_to_dbfs(mean_power);
  }
  return true;
}

bool play_tone_and_measure_mics(
    float tone_hz,
    float tone_amp,
    uint32_t tone_ms,
    SelfTestMicMetrics* out,
    uint32_t* out_bytes,
    String* err) {
  if (!(tone_hz >= 20.0f && tone_hz <= 20000.0f)) {
    if (err != nullptr) {
      *err = "TONE HZ must be in [20, 20000].";
    }
    return false;
  }
  if (!(tone_amp > 0.0f && tone_amp <= 0.95f)) {
    if (err != nullptr) {
      *err = "TONE AMP must be in (0, 0.95].";
    }
    return false;
  }
  if (tone_ms < 100U || tone_ms > 60000U) {
    if (err != nullptr) {
      *err = "TONE MS must be in [100, 60000].";
    }
    return false;
  }
  if (!g_anc_rx_installed || !g_anc_tx_installed) {
    if (err != nullptr) {
      *err = "ANC RX/TX streams not installed.";
    }
    return false;
  }
  if (out == nullptr) {
    if (err != nullptr) {
      *err = "Output metrics pointer is null.";
    }
    return false;
  }

  constexpr size_t kFramesPerChunk = 128;
  int32_t rx_buf[kFramesPerChunk * 2];
  int16_t tx_buf[kFramesPerChunk * 2];

  const uint32_t target_frames =
      static_cast<uint32_t>((static_cast<uint64_t>(tone_ms) * kSampleRateHz) / 1000ULL);
  const float phase_inc = 2.0f * static_cast<float>(M_PI) * tone_hz / static_cast<float>(kSampleRateHz);
  float phase = 0.0f;

  const int ref_idx = slot_index_in_frame(kReferenceSlot);
  const int err_idx = slot_index_in_frame(kErrorSlot);

  uint32_t measured_frames = 0;
  uint32_t total_bytes = 0;
  double sum_ref = 0.0;
  double sum_err = 0.0;
  int32_t ref_peak = 0;
  int32_t err_peak = 0;

  while (measured_frames < target_frames) {
    const uint32_t remaining = target_frames - measured_frames;
    const size_t wanted_frames = (remaining < kFramesPerChunk) ? remaining : kFramesPerChunk;

    size_t bytes_read = 0;
    const esp_err_t read_err = i2s_read(
        kMicI2sPort,
        rx_buf,
        wanted_frames * 2U * sizeof(int32_t),
        &bytes_read,
        pdMS_TO_TICKS(250));
    if (read_err != ESP_OK) {
      if (err != nullptr) {
        *err = "i2s_read failed during loop test.";
      }
      return false;
    }

    const size_t got_words = bytes_read / sizeof(int32_t);
    const size_t got_frames = got_words / 2U;
    if (got_frames == 0) {
      continue;
    }

    for (size_t i = 0; i < got_frames; ++i) {
      const int32_t right_w = rx_buf[i * 2 + 0];
      const int32_t left_w = rx_buf[i * 2 + 1];
      const int16_t slots[2] = {
          decode_i2s_32_to_i16(right_w),
          decode_i2s_32_to_i16(left_w),
      };

      const int16_t ref_i16 = slots[ref_idx];
      const int16_t err_i16 = slots[err_idx];
      const float ref = pcm16_to_float(ref_i16);
      const float errf = pcm16_to_float(err_i16);

      sum_ref += static_cast<double>(ref) * static_cast<double>(ref);
      sum_err += static_cast<double>(errf) * static_cast<double>(errf);

      const int32_t ref_abs = (ref_i16 >= 0) ? ref_i16 : -static_cast<int32_t>(ref_i16);
      const int32_t err_abs = (err_i16 >= 0) ? err_i16 : -static_cast<int32_t>(err_i16);
      if (ref_abs > ref_peak) {
        ref_peak = ref_abs;
      }
      if (err_abs > err_peak) {
        err_peak = err_abs;
      }

      const float s = sinf(phase) * tone_amp;
      phase += phase_inc;
      if (phase > 2.0f * static_cast<float>(M_PI)) {
        phase -= 2.0f * static_cast<float>(M_PI);
      }
      const int16_t v = float_to_pcm16(s);
      tx_buf[i * 2 + 0] = v;
      tx_buf[i * 2 + 1] = v;
    }

    const size_t need_bytes = got_frames * 2U * sizeof(int16_t);
    size_t bytes_written = 0;
    const esp_err_t write_err = i2s_write(
        kDacI2sPort,
        tx_buf,
        need_bytes,
        &bytes_written,
        pdMS_TO_TICKS(250));
    if (write_err != ESP_OK) {
      if (err != nullptr) {
        *err = "i2s_write failed during loop test.";
      }
      return false;
    }
    if (bytes_written != need_bytes) {
      if (err != nullptr) {
        *err = "Short i2s_write during loop test.";
      }
      return false;
    }

    measured_frames += static_cast<uint32_t>(got_frames);
    total_bytes += static_cast<uint32_t>(bytes_written);
  }

  if (measured_frames == 0U) {
    if (err != nullptr) {
      *err = "No frames captured during loop test tone window.";
    }
    return false;
  }

  out->frames = measured_frames;
  out->ref_peak = ref_peak;
  out->err_peak = err_peak;
  out->ref_rms_dbfs = power_to_dbfs(static_cast<float>(sum_ref / static_cast<double>(measured_frames)));
  out->err_rms_dbfs = power_to_dbfs(static_cast<float>(sum_err / static_cast<double>(measured_frames)));
  out->ref_active = (ref_peak > 64) || (out->ref_rms_dbfs > -100.0f);
  out->err_active = (err_peak > 64) || (out->err_rms_dbfs > -100.0f);

  if (out_bytes != nullptr) {
    *out_bytes = total_bytes;
  }
  return true;
}

void run_looptest(
    float tone_hz,
    float tone_amp,
    uint32_t tone_ms,
    uint32_t base_ms,
    uint32_t wait_ms,
    float threshold_db) {
  Serial.println("LOOPTEST START");

  bool overall_pass = true;
  if (threshold_db < 1.0f || threshold_db > 60.0f) {
    Serial.println("LOOPTEST FAIL PARAM THRESH_DB must be in [1, 60].");
    Serial.println("LOOPTEST RESULT=FAIL");
    Serial.println("LOOPTEST END");
    return;
  }

  if (g_anc.active) {
    anc_stop();
    Serial.println("LOOPTEST INFO ANC stopped before test.");
  }

  String err;
  if (!configure_anc_stream(&err)) {
    Serial.println("LOOPTEST FAIL STREAM_SETUP ERR=" + err);
    Serial.println("LOOPTEST RESULT=FAIL");
    Serial.println("LOOPTEST END");
    return;
  }

  warm_up_mic_i2s();

  if (wait_ms > 0U) {
    Serial.println(String("LOOPTEST WAIT_MS=") + String(wait_ms));
    delay(wait_ms);
  }

  SelfTestMicMetrics base{};
  if (!measure_mics_window_ms(base_ms, &base, &err)) {
    overall_pass = false;
    Serial.println("LOOPTEST FAIL BASELINE ERR=" + err);
  } else {
    Serial.println(
        String("LOOPTEST BASE REF_DBFS=") + String(base.ref_rms_dbfs, 2) +
        " ERR_DBFS=" + String(base.err_rms_dbfs, 2) +
        " FRAMES=" + String(base.frames));
  }

  SelfTestMicMetrics tone{};
  uint32_t tone_bytes = 0;
  err = "";
  if (!play_tone_and_measure_mics(tone_hz, tone_amp, tone_ms, &tone, &tone_bytes, &err)) {
    overall_pass = false;
    Serial.println("LOOPTEST FAIL TONE_MEASURE ERR=" + err);
  } else {
    Serial.println(
        String("LOOPTEST TONE HZ=") + String(tone_hz, 1) +
        " AMP=" + String(tone_amp, 3) +
        " MS=" + String(tone_ms) +
        " REF_DBFS=" + String(tone.ref_rms_dbfs, 2) +
        " ERR_DBFS=" + String(tone.err_rms_dbfs, 2) +
        " FRAMES=" + String(tone.frames) +
        " BYTES=" + String(tone_bytes));
  }

  if (overall_pass) {
    const float delta_ref = tone.ref_rms_dbfs - base.ref_rms_dbfs;
    const float delta_err = tone.err_rms_dbfs - base.err_rms_dbfs;
    const float best_delta = (delta_ref > delta_err) ? delta_ref : delta_err;
    const bool pass = best_delta >= threshold_db;
    if (!pass) {
      overall_pass = false;
    }
    Serial.println(
        String("LOOPTEST DELTA REF_DB=") + String(delta_ref, 2) +
        " ERR_DB=" + String(delta_err, 2) +
        " BEST_DB=" + String(best_delta, 2) +
        " THRESH_DB=" + String(threshold_db, 2) +
        " RESULT=" + (pass ? "PASS" : "FAIL"));
  }

  uninstall_anc_stream_if_installed();
  g_anc.active = false;
  g_anc.phase = AncPhase::Idle;

  Serial.println(String("LOOPTEST RESULT=") + (overall_pass ? "PASS" : "FAIL"));
  Serial.println("LOOPTEST END");
}

bool selftest_play_dac_tone(String* err, uint32_t* out_bytes, float* out_rms_dbfs) {
  return play_dac_tone(
      kSelfTestToneHz,
      kSelfTestToneAmp,
      kSelfTestToneMs,
      err,
      out_bytes,
      out_rms_dbfs);
}

bool selftest_check_phase_transition(String* err, AncPhase* out_final_phase, uint32_t* out_sat_delta) {
  AncConfig cfg = g_anc.cfg;
  cfg.cal_ms = kSelfTestCalMs;
  cfg.adapt_ms = kSelfTestAdaptMs;
  cfg.mu = kAncDefaultMu;
  cfg.mu_cal = kAncDefaultMuCal;
  cfg.output_gain = 1.0f;
  cfg.limiter_dbfs = -10.0f;
  cfg.cal_excitation_amp = 0.35f;
  cfg.log_period_ms = 0;

  if (!anc_start(cfg, err)) {
    return false;
  }

  const uint32_t start_ms = millis();
  const uint32_t sat_before = g_anc.stats.saturated_samples;

  while ((millis() - start_ms) < kSelfTestPhaseTimeoutMs) {
    anc_process_block();
    if (g_anc.phase == AncPhase::Frozen) {
      break;
    }
  }

  const AncPhase final_phase = g_anc.phase;
  if (out_final_phase != nullptr) {
    *out_final_phase = final_phase;
  }
  if (out_sat_delta != nullptr) {
    *out_sat_delta = g_anc.stats.saturated_samples - sat_before;
  }

  const bool ok = (final_phase == AncPhase::Frozen);
  if (!ok && err != nullptr) {
    *err = String("Timed out before FROZEN, final phase=") + anc_phase_name(final_phase);
  }

  anc_stop();
  return ok;
}

void run_selftest() {
  Serial.println("SELFTEST START");

  bool overall_pass = true;
  if (g_anc.active) {
    anc_stop();
    Serial.println("SELFTEST INFO ANC stopped before test.");
  }

  String err;
  if (!configure_anc_stream(&err)) {
    Serial.println("SELFTEST FAIL STREAM_SETUP ERR=" + err);
    Serial.println("SELFTEST RESULT=FAIL");
    Serial.println("SELFTEST END");
    return;
  }

  warm_up_mic_i2s();

  SelfTestMicMetrics mic{};
  bool mic_read_ok = selftest_measure_mics(&mic, &err);
  if (!mic_read_ok) {
    overall_pass = false;
    Serial.println("SELFTEST FAIL MIC_READ ERR=" + err);
  } else {
    const bool any_active = mic.ref_active || mic.err_active;
    const bool both_active = mic.ref_active && mic.err_active;
    if (!any_active) {
      overall_pass = false;
    }
    Serial.println(
        String("SELFTEST MIC REF_DBFS=") + String(mic.ref_rms_dbfs, 2) +
        " ERR_DBFS=" + String(mic.err_rms_dbfs, 2) +
        " REF_ACTIVE=" + yes_no(mic.ref_active) +
        " ERR_ACTIVE=" + yes_no(mic.err_active) +
        " FRAMES=" + String(mic.frames) +
        " RESULT=" + (both_active ? "PASS" : (any_active ? "WARN" : "FAIL")));
  }

  uint32_t tone_bytes = 0;
  float tone_rms_dbfs = -200.0f;
  err = "";
  const bool tone_ok = selftest_play_dac_tone(&err, &tone_bytes, &tone_rms_dbfs);
  if (!tone_ok) {
    overall_pass = false;
    Serial.println("SELFTEST FAIL DAC_WRITE ERR=" + err);
  } else {
    Serial.println(
        String("SELFTEST DAC TONE_HZ=") + String(kSelfTestToneHz, 1) +
        " TONE_RMS_DBFS=" + String(tone_rms_dbfs, 2) +
        " BYTES=" + String(tone_bytes) +
        " RESULT=PASS");
  }

  AncPhase final_phase = AncPhase::Idle;
  uint32_t sat_delta = 0;
  err = "";
  const bool phase_ok = selftest_check_phase_transition(&err, &final_phase, &sat_delta);
  if (!phase_ok) {
    overall_pass = false;
    Serial.println("SELFTEST FAIL PHASE ERR=" + err);
  } else {
    Serial.println(
        String("SELFTEST PHASE FINAL=") + anc_phase_name(final_phase) +
        " SAT_DELTA=" + String(sat_delta) +
        " RESULT=PASS");
  }

  uninstall_anc_stream_if_installed();
  g_anc.active = false;
  g_anc.phase = AncPhase::Idle;

  Serial.println(String("SELFTEST RESULT=") + (overall_pass ? "PASS" : "FAIL"));
  Serial.println("SELFTEST END");
}

// -------------------------
// Status / command handlers
// -------------------------

void print_anc_help() {
  Serial.println("ANC commands:");
  Serial.println("  ANC START [CAL_MS=3000 ADAPT_MS=7000 MU=0.002 MUCAL=0.12 GAIN=1.0 LIMIT_DBFS=-6 CAL_AMP=0.20 SIGN=-1 LOG_MS=1000]");
  Serial.println("  ANC STOP");
  Serial.println("  ANC FREEZE");
  Serial.println("  ANC ADAPT [ADAPT_MS=0]");
  Serial.println("  ANC TONE [HZ=1000 AMP=0.85 MS=3000]");
  Serial.println("  ANC LOOPTEST [HZ=1000 AMP=0.85 MS=2500 BASE_MS=1000 WAIT_MS=1500 THRESH_DB=8]");
  Serial.println("  ANC RESET");
  Serial.println("  ANC STATUS");
  Serial.println("  ANC SELFTEST");
  Serial.println("  LOG_MS: 0 disables periodic logs; 100..60000 ms controls rate.");
  Serial.println("  Note: ANC START uses built-in defaults each run unless options override.");
}

void print_status() {
  static uint32_t s_last_status_ms = 0;
  const uint32_t now_ms = millis();
  if (s_last_status_ms != 0U &&
      (now_ms - s_last_status_ms) < kStatusMinIntervalMs) {
    return;
  }
  s_last_status_ms = now_ms;

  Serial.println("STATUS READY=1");
  Serial.println(String("STATUS PSRAM=") + (psramFound() ? "1" : "0"));
  Serial.println(String("STATUS BAUD=") + String(kSerialBaud));
  Serial.println(String("STATUS SAMPLE_RATE_HZ=") + String(kSampleRateHz));
  Serial.println(String("STATUS MIC_I2S_PORT=") + String(static_cast<int>(kMicI2sPort)));
  Serial.println(String("STATUS MIC_PIN_BCLK=") + String(kMicBclkPin));
  Serial.println(String("STATUS MIC_PIN_WS=") + String(kMicWsPin));
  Serial.println(String("STATUS MIC_PIN_DATA=") + String(kMicDataPin));
  Serial.println(String("STATUS DAC_I2S_PORT=") + String(static_cast<int>(kDacI2sPort)));
  Serial.println(String("STATUS DAC_PIN_BCLK=") + String(kDacBclkPin));
  Serial.println(String("STATUS DAC_PIN_WS=") + String(kDacWsPin));
  Serial.println(String("STATUS DAC_PIN_DATA=") + String(kDacDataPin));
  Serial.println(String("STATUS REF_SLOT=") + ((kReferenceSlot == MicSlot::Left) ? "LEFT" : "RIGHT"));
  Serial.println(String("STATUS ERR_SLOT=") + ((kErrorSlot == MicSlot::Left) ? "LEFT" : "RIGHT"));
  Serial.println(String("STATUS ANC_ACTIVE=") + (g_anc.active ? "1" : "0"));
  Serial.println(String("STATUS ANC_PHASE=") + anc_phase_name(g_anc.phase));
  Serial.println(String("STATUS ANC_CAL_MS=") + String(g_anc.cfg.cal_ms));
  Serial.println(String("STATUS ANC_ADAPT_MS=") + String(g_anc.cfg.adapt_ms));
  Serial.println(String("STATUS ANC_MU=") + String(g_anc.cfg.mu, 6));
  Serial.println(String("STATUS ANC_MUCAL=") + String(g_anc.cfg.mu_cal, 6));
  Serial.println(String("STATUS ANC_GAIN=") + String(g_anc.cfg.output_gain, 4));
  Serial.println(String("STATUS ANC_LIMIT_DBFS=") + String(g_anc.cfg.limiter_dbfs, 2));
  Serial.println(String("STATUS ANC_CAL_AMP=") + String(g_anc.cfg.cal_excitation_amp, 3));
  Serial.println(String("STATUS ANC_SIGN=") + String(g_anc.cfg.adapt_sign, 1));
  Serial.println(String("STATUS ANC_LOG_MS=") + String(g_anc.cfg.log_period_ms));
  Serial.println(String("STATUS ANC_RX_READ_ERRS=") + String(g_anc.stats.rx_read_errors));
  Serial.println(String("STATUS ANC_RX_EMPTY_READS=") + String(g_anc.stats.rx_empty_reads));
  Serial.println(String("STATUS ANC_TX_WRITE_ERRS=") + String(g_anc.stats.tx_write_errors));
  Serial.println(String("STATUS ANC_TX_SHORT_WRITES=") + String(g_anc.stats.tx_short_writes));
}

void handle_rec_command(const String& line) {
  RecCommand cmd;
  String err;
  if (!parse_rec_command(line, &cmd, &err)) {
    if (!line.isEmpty()) {
      Serial.println("ERR " + err);
    }
    return;
  }

  if (g_anc.active) {
    anc_stop();
    Serial.println("DBG ANC stopped to service REC command");
  }

  log_debug(String("rec start ms=") + String(cmd.duration_ms) + " mic=" + String(cmd.mic_id));
  int16_t* pcm = nullptr;
  uint32_t sample_count = 0;
  uint16_t channels = 0;
  if (!capture_to_psram(cmd, &pcm, &sample_count, &channels, &err)) {
    Serial.println("ERR " + err);
    return;
  }

  log_debug(
      String("rec captured samples=") + String(sample_count) +
      " channels=" + String(channels));
  send_capture_binary(cmd, pcm, sample_count, channels);
  heap_caps_free(pcm);
  log_debug("rec payload sent");
  Serial.println("OK REC_DONE");
}

void handle_anc_command(const String& line) {
  String upper = line;
  upper.trim();
  upper.toUpperCase();

  if (upper == "ANC HELP") {
    print_anc_help();
    return;
  }

  if (upper == "ANC STATUS") {
    print_status();
    return;
  }

  if (upper.startsWith("ANC LOOPTEST")) {
    float tone_hz = kToneDefaultHz;
    float tone_amp = kToneDefaultAmp;
    float thresh_db = kLoopTestDefaultThresholdDb;
    uint32_t tone_ms = kLoopTestDefaultToneMs;
    uint32_t base_ms = kLoopTestDefaultBaseMs;
    uint32_t wait_ms = kLoopTestDefaultWaitMs;
    float f = 0.0f;
    uint32_t u32 = 0;
    if (parse_float_option(upper, "HZ", &f)) {
      tone_hz = f;
    }
    if (parse_float_option(upper, "AMP", &f)) {
      tone_amp = f;
    }
    if (parse_float_option(upper, "THRESH_DB", &f)) {
      thresh_db = f;
    }
    if (parse_u32_option(upper, "MS", &u32)) {
      tone_ms = u32;
    }
    if (parse_u32_option(upper, "BASE_MS", &u32)) {
      base_ms = u32;
    }
    if (parse_u32_option(upper, "WAIT_MS", &u32)) {
      wait_ms = u32;
    }

    run_looptest(tone_hz, tone_amp, tone_ms, base_ms, wait_ms, thresh_db);
    return;
  }

  if (upper.startsWith("ANC TONE")) {
    float tone_hz = kToneDefaultHz;
    float tone_amp = kToneDefaultAmp;
    uint32_t tone_ms = kToneDefaultMs;
    float f = 0.0f;
    uint32_t u32 = 0;
    if (parse_float_option(upper, "HZ", &f)) {
      tone_hz = f;
    }
    if (parse_float_option(upper, "AMP", &f)) {
      tone_amp = f;
    }
    if (parse_u32_option(upper, "MS", &u32)) {
      tone_ms = u32;
    }

    if (g_anc.active) {
      anc_stop();
      Serial.println("DBG ANC stopped to play tone");
    }

    String err;
    if (!configure_anc_stream(&err)) {
      Serial.println("ERR " + err);
      return;
    }

    uint32_t bytes = 0;
    float rms_dbfs = -200.0f;
    if (!play_dac_tone(tone_hz, tone_amp, tone_ms, &err, &bytes, &rms_dbfs)) {
      uninstall_anc_stream_if_installed();
      Serial.println("ERR " + err);
      return;
    }

    uninstall_anc_stream_if_installed();
    Serial.println(String("OK ANC_TONE_DONE HZ=") + String(tone_hz, 1) +
                   " AMP=" + String(tone_amp, 3) +
                   " MS=" + String(tone_ms) +
                   " RMS_DBFS=" + String(rms_dbfs, 2) +
                   " BYTES=" + String(bytes));
    return;
  }

  if (upper == "ANC SELFTEST") {
    run_selftest();
    return;
  }

  if (upper == "ANC STOP") {
    anc_stop();
    Serial.println("OK ANC_STOPPED");
    return;
  }

  if (upper == "ANC RESET") {
    anc_reset_state(true);
    Serial.println("OK ANC_RESET");
    return;
  }

  if (upper.startsWith("ANC FREEZE")) {
    if (!g_anc.active) {
      Serial.println("ERR ANC is not active.");
      return;
    }
    anc_set_phase(AncPhase::Frozen);
    return;
  }

  if (upper.startsWith("ANC ADAPT")) {
    if (!g_anc.active) {
      Serial.println("ERR ANC is not active.");
      return;
    }
    uint32_t adapt_ms = 0;
    if (parse_u32_option(upper, "ADAPT_MS", &adapt_ms)) {
      g_anc.cfg.adapt_ms = adapt_ms;
      g_anc.adapt_target_samples =
          static_cast<uint32_t>((static_cast<uint64_t>(adapt_ms) * kSampleRateHz) / 1000ULL);
    } else {
      // Resume indefinite adaptation unless ADAPT_MS explicitly set.
      g_anc.adapt_target_samples = 0;
    }
    anc_set_phase(AncPhase::Adapting);
    return;
  }

  if (upper.startsWith("ANC START") || upper.startsWith("ANC RUN")) {
    // Start from known defaults every run to avoid sticky runtime settings
    // (e.g., ADAPT_MS=0 persisting and causing "stuck in ADAPTING").
    AncConfig cfg{};
    uint32_t u32 = 0;
    float f = 0.0f;
    if (parse_u32_option(upper, "CAL_MS", &u32)) {
      cfg.cal_ms = u32;
    }
    if (parse_u32_option(upper, "ADAPT_MS", &u32)) {
      cfg.adapt_ms = u32;
    }
    if (parse_u32_option(upper, "LOG_MS", &u32)) {
      cfg.log_period_ms = u32;
    }
    if (parse_float_option(upper, "MU", &f)) {
      cfg.mu = f;
    }
    if (parse_float_option(upper, "MUCAL", &f)) {
      cfg.mu_cal = f;
    }
    if (parse_float_option(upper, "GAIN", &f)) {
      cfg.output_gain = f;
    }
    if (parse_float_option(upper, "LIMIT_DBFS", &f)) {
      cfg.limiter_dbfs = f;
    }
    if (parse_float_option(upper, "CAL_AMP", &f)) {
      cfg.cal_excitation_amp = f;
    }
    if (parse_float_option(upper, "SIGN", &f)) {
      cfg.adapt_sign = (f >= 0.0f) ? 1.0f : -1.0f;
    }

    if (g_anc.active) {
      anc_stop();
    }
    String err;
    if (!anc_start(cfg, &err)) {
      Serial.println("ERR " + err);
      return;
    }
    return;
  }

  Serial.println("ERR Unknown ANC command. Use ANC HELP.");
}

void print_ready_banner() {
  Serial.println("ANC_CAPTURE_READY");
  Serial.println("REC format: REC <ms> MIC=<0|1|2>");
  Serial.println("ANC format: ANC START [...], ANC STOP, ANC FREEZE, ANC ADAPT, ANC TONE, ANC LOOPTEST, ANC SELFTEST");
  Serial.println("Quick diagnostic: SELFTEST");
  Serial.println("Speaker tone test: ANC TONE HZ=1000 AMP=0.85 MS=3000");
  Serial.println("Acoustic loop test: ANC LOOPTEST WAIT_MS=1500");
  Serial.println("Use STATUS for full runtime state.");
}

void handle_text_command(const String& line) {
  String cmd = line;
  cmd.trim();
  if (cmd.isEmpty()) {
    return;
  }

  String upper = cmd;
  upper.toUpperCase();

  if (upper == "PING") {
    Serial.println("PONG");
    return;
  }

  if (upper == "STATUS") {
    print_status();
    return;
  }

  if (upper == "SELFTEST") {
    run_selftest();
    return;
  }

  if (upper == "LOOPTEST") {
    run_looptest(
        kToneDefaultHz,
        kToneDefaultAmp,
        kLoopTestDefaultToneMs,
        kLoopTestDefaultBaseMs,
        kLoopTestDefaultWaitMs,
        kLoopTestDefaultThresholdDb);
    return;
  }

  if (upper.startsWith("ANC ")) {
    handle_anc_command(cmd);
    return;
  }

  if (upper.startsWith("REC")) {
    handle_rec_command(cmd);
    return;
  }

  Serial.println("ERR Unknown command.");
}

void pump_serial_input() {
  while (Serial.available() > 0) {
    const int b = Serial.read();
    if (b < 0) {
      return;
    }
    const char c = static_cast<char>(b);
    if (c == '\n' || c == '\r') {
      if (!g_serial_line.isEmpty()) {
        handle_text_command(g_serial_line);
        g_serial_line = "";
        g_serial_last_rx_ms = 0;
      }
      continue;
    }
    if (g_serial_line.length() < 256) {
      g_serial_line += c;
      g_serial_last_rx_ms = millis();
    }
  }

  // Some serial monitors send text with no line ending.
  // If input has been idle briefly, treat the buffered text as one command.
  if (!g_serial_line.isEmpty() && g_serial_last_rx_ms != 0U) {
    const uint32_t now = millis();
    if ((now - g_serial_last_rx_ms) >= kSerialCmdIdleFlushMs) {
      handle_text_command(g_serial_line);
      g_serial_line = "";
      g_serial_last_rx_ms = 0;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  Serial.setTimeout(50);
  delay(250);
  Serial.println(String("BOOT SERIAL_BAUD=") + String(kSerialBaud) +
                 " RESET_REASON=" + reset_reason_name(esp_reset_reason()));
  print_ready_banner();
  log_debug("boot complete");
}

void loop() {
  pump_serial_input();

  if (g_anc.active) {
    anc_process_block();
  } else {
    delay(2);
  }
}
