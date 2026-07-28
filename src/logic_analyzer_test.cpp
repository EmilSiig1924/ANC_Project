#include <Arduino.h>
#include <driver/i2s.h>

namespace {

// Logic analyzer hookup:
//   CH0 -> GPIO4 = I2S BCLK / bit clock
//   CH1 -> GPIO5 = I2S WS / LRCK / frame
//   CH2 -> GPIO6 = I2S DATA
//   GND -> ESP32 GND
constexpr i2s_port_t kI2sPort = I2S_NUM_0;
constexpr int kBclkPin = 4;
constexpr int kWsPin = 5;
constexpr int kDataPin = 6;

constexpr uint32_t kSerialBaud = 115200;
// Keep this deliberately slow so a new analyzer/probe setup is easy to debug.
// Stereo 16-bit I2S gives BCLK = sample_rate * 16 bits * 2 channels = 256 kHz.
constexpr uint32_t kSampleRateHz = 8000;
constexpr size_t kFramesPerChunk = 128;

// Dense bit patterns make the DATA line visibly toggle even before the decoder
// is configured correctly.
constexpr uint16_t kLeftWords[] = {
    0x5555,
    0xAAAA,
    0x3333,
    0xCCCC,
};
constexpr uint16_t kRightWords[] = {
    0xAAAA,
    0x5555,
    0xCCCC,
    0x3333,
};
constexpr size_t kPatternCount = sizeof(kLeftWords) / sizeof(kLeftWords[0]);
constexpr uint32_t kFramesPerPatternWord = 1;

int16_t g_tx_buf[kFramesPerChunk * 2];
uint32_t g_frame_index = 0;

bool setup_i2s_tx() {
  i2s_config_t cfg{};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = kSampleRateHz;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  esp_err_t esp = i2s_driver_install(kI2sPort, &cfg, 0, nullptr);
  if (esp != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", static_cast<int>(esp));
    return false;
  }

  i2s_pin_config_t pin_cfg{};
  pin_cfg.bck_io_num = kBclkPin;
  pin_cfg.ws_io_num = kWsPin;
  pin_cfg.data_out_num = kDataPin;
  pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;

  esp = i2s_set_pin(kI2sPort, &pin_cfg);
  if (esp != ESP_OK) {
    Serial.printf("i2s_set_pin failed: %d\n", static_cast<int>(esp));
    i2s_driver_uninstall(kI2sPort);
    return false;
  }

  esp = i2s_set_clk(kI2sPort, kSampleRateHz, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (esp != ESP_OK) {
    Serial.printf("i2s_set_clk failed: %d\n", static_cast<int>(esp));
    i2s_driver_uninstall(kI2sPort);
    return false;
  }

  i2s_zero_dma_buffer(kI2sPort);
  return true;
}

void fill_test_pattern() {
  for (size_t i = 0; i < kFramesPerChunk; ++i) {
    const size_t pattern_idx = (g_frame_index / kFramesPerPatternWord) % kPatternCount;

    // I2S_CHANNEL_FMT_RIGHT_LEFT uses [Right, Left] interleaving.
    g_tx_buf[(i * 2) + 0] = static_cast<int16_t>(kRightWords[pattern_idx]);
    g_tx_buf[(i * 2) + 1] = static_cast<int16_t>(kLeftWords[pattern_idx]);
    ++g_frame_index;
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(250);

  Serial.println("Logic analyzer I2S test pattern");
  Serial.println("Pins: CH0/GPIO4=BCLK, CH1/GPIO5=WS, CH2/GPIO6=DATA");
  Serial.println("Expected: BCLK=256 kHz, WS=8 kHz, 16-bit stereo, standard I2S");
  Serial.println("Expected repeating words:");
  for (size_t i = 0; i < kPatternCount; ++i) {
    Serial.printf(
        "  Right=0x%04X  Left=0x%04X\n",
        static_cast<unsigned>(kRightWords[i]),
        static_cast<unsigned>(kLeftWords[i]));
  }

  if (!setup_i2s_tx()) {
    Serial.println("Failed to initialize I2S TX.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Streaming test pattern.");
}

void loop() {
  fill_test_pattern();

  size_t bytes_written = 0;
  const esp_err_t esp = i2s_write(
      kI2sPort,
      g_tx_buf,
      sizeof(g_tx_buf),
      &bytes_written,
      pdMS_TO_TICKS(100));

  if (esp != ESP_OK || bytes_written != sizeof(g_tx_buf)) {
    Serial.printf(
        "i2s_write issue: err=%d bytes=%u/%u\n",
        static_cast<int>(esp),
        static_cast<unsigned>(bytes_written),
        static_cast<unsigned>(sizeof(g_tx_buf)));
    delay(50);
  }
}
