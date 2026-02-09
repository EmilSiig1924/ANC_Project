#include <Arduino.h>
#include <driver/i2s.h>

/*
  Minimal I2S tone smoke test adapted from Espressif's official
  Arduino-ESP32 ESP_I2S "Simple_tone" example.
  Source:
  https://raw.githubusercontent.com/espressif/arduino-esp32/master/libraries/ESP_I2S/examples/Simple_tone/Simple_tone.ino

  Wiring used here (PCM5102A):
  - BCLK/SCK <- GPIO4
  - LRCK/WS  <- GPIO5
  - DIN      <- GPIO6
*/

constexpr i2s_port_t kI2sPort = I2S_NUM_0;
constexpr int kI2sLrcPin = 5;
constexpr int kI2sBclkPin = 4;
constexpr int kI2sDinPin = 6;

constexpr int kFrequencyHz = 1000;
constexpr int16_t kAmplitude = 12000;
constexpr uint32_t kSampleRateHz = 48000;
constexpr size_t kFramesPerChunk = 256;

int16_t g_tx_buf[kFramesPerChunk * 2];
uint32_t g_sample_index = 0;

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
  pin_cfg.bck_io_num = kI2sBclkPin;
  pin_cfg.ws_io_num = kI2sLrcPin;
  pin_cfg.data_out_num = kI2sDinPin;
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

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("I2S tone smoke test (official Simple_tone logic, low-level I2S driver)");
  Serial.println("Pins: BCLK=4 LRCK=5 DIN=6");
  Serial.printf("Tone: %d Hz, sample rate: %lu Hz\n", kFrequencyHz, static_cast<unsigned long>(kSampleRateHz));

  if (!setup_i2s_tx()) {
    Serial.println("Failed to initialize I2S TX.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("I2S started. Continuous square tone should be audible.");
}

void loop() {
  const uint32_t half_wavelength = kSampleRateHz / (2U * static_cast<uint32_t>(kFrequencyHz));
  if (half_wavelength == 0U) {
    delay(1000);
    return;
  }

  for (size_t i = 0; i < kFramesPerChunk; ++i) {
    const bool high = ((g_sample_index / half_wavelength) & 1U) == 0U;
    const int16_t sample = high ? kAmplitude : static_cast<int16_t>(-kAmplitude);
    g_tx_buf[i * 2 + 0] = sample;  // Right
    g_tx_buf[i * 2 + 1] = sample;  // Left
    g_sample_index++;
  }

  size_t bytes_written = 0;
  const esp_err_t esp = i2s_write(
      kI2sPort,
      g_tx_buf,
      sizeof(g_tx_buf),
      &bytes_written,
      pdMS_TO_TICKS(100));
  if (esp != ESP_OK) {
    Serial.printf("i2s_write failed: %d\n", static_cast<int>(esp));
    delay(50);
  }
}
