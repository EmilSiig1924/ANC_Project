# ESP32-S3 Capture Pipeline (INMP441, PSRAM, USB Dump)

This project now supports deterministic audio capture on ESP32-S3:

- Record into PSRAM (`REC <ms> MIC=<0|1|2>`)
- Dump binary PCM over USB serial
- Save WAV on PC with `tools/record_usb.py`

## 1) Firmware Setup (PlatformIO)

Files added:

- `platformio.ini`
- `src/main.cpp`

Default capture settings:

- Sample rate: `48,000 Hz`
- PCM format: `16-bit`, `mono` (`MIC=1/2`) or `stereo` (`MIC=0`)
- Default record command: `REC 20000 MIC=1`

Default pins in `src/main.cpp`:

- `I2S port` -> `I2S_NUM_1`
- `BCLK` -> GPIO `14`
- `WS`   -> GPIO `15`
- `SD` (shared by both INMP441) -> GPIO `16`

Mic selection model:

- Both INMP441 mics are on the same I2S data line.
- One mic must be strapped `L`, the other strapped `R`.
- `REC ... MIC=1` and `REC ... MIC=2` select channel slots in firmware.
- `REC ... MIC=0` captures both mics simultaneously as stereo (`ch0=mic1`, `ch1=mic2`).
- If channels are reversed, swap `kMic1Slot` and `kMic2Slot` in `src/main.cpp`.

Edit these constants to match your wiring:

- `kI2sBclkPin`
- `kI2sWsPin`
- `kMic1DataPin`
- `kMic2DataPin`

## 2) Flash Firmware

From repo root:

```bash
pio run -t upload
```

Minimal TX-only tone smoke test firmware (based on official ESP_I2S Simple_tone logic):

```bash
pio run -e esp32s3_tone_smoke -t upload --upload-port COM7
pio device monitor -b 115200
```

Expected serial text from this smoke test:

- `I2S tone smoke test ...`
- `Pins: BCLK=4 LRCK=5 DIN=6`
- `I2S started. Continuous square tone should be audible.`

Open serial monitor if needed:

```bash
pio device monitor -b 115200
```

You should see:

- `ANC_CAPTURE_READY`
- command help line

## 3) Record from PC

Install Python dependency:

```bash
pip install pyserial
```

Capture 20 seconds from mic 1:

```bash
python tools/record_usb.py --mic 1 --duration-ms 20000 --baud 115200
```

Capture from mic 2:

```bash
python tools/record_usb.py --mic 2 --duration-ms 20000 --baud 115200
```

Capture both mics simultaneously (stereo baseline):

```bash
python tools/record_usb.py --mic 0 --duration-ms 3000 --baud 115200 --out Data/baseline_stereo.wav
```

If multiple COM ports exist, specify one:

```bash
python tools/record_usb.py --port COM7 --mic 1
```

Write to a fixed path:

```bash
python tools/record_usb.py --mic 1 --out Data/config_A_ref.wav
```

One-command pair capture + analysis:

```bash
python tools/record_pair_and_analyze.py --port COM7 --duration-ms 20000
```

With default `--ref-mic 2`, this records a simultaneous stereo run capture (both mics at once), maps channels to reference/test files, and runs `analyze_mic_test.py`:

- `Data/pair_<tag>_baseline_stereo.wav`
- `Data/pair_<tag>_baseline_mic1.wav`
- `Data/pair_<tag>_baseline_mic2.wav`
- `Data/pair_<tag>_run_stereo.wav`
- `Data/pair_<tag>_ref_mic2.wav`
- `Data/pair_<tag>_test_mic1.wav`

By default, baseline duration is `3000 ms`. Disable with `--baseline-ms 0`.
By default, plots are saved to `Plots/pair_<tag>.png` and shown after each test.
By default, both PSD and FFT plots are generated.
The runner also prints stage-by-stage prompts and uses a countdown before each capture.
Use `--sequential-run` if you want legacy two-pass mono capture.

Shortcut PowerShell runner (same workflow with your defaults):

```powershell
.\run_pair.ps1
```

Optional examples:

```powershell
.\run_pair.ps1 -Port COM7 -DurationMs 3000 -DebugIO
.\run_pair.ps1 -Tag jigA_01
.\run_pair.ps1 -Tag Test1
.\run_pair.ps1 -Tag Test1 -SavePlot Plots\Test1_custom.png
.\run_pair.ps1 -BaselineMs 0
.\run_pair.ps1 -CountdownS 8
.\run_pair.ps1 -NoCountdown
.\run_pair.ps1 -NoiseAlpha 0.65 -NoiseFloorFrac 0.003
.\run_pair.ps1 -SequentialRun
.\run_pair.ps1 -NoPlotFFT
```

## 4) Analyze Immediately

You can trigger your analysis script right after capture:

```bash
python tools/record_usb.py ^
  --mic 1 ^
  --analyze-script analyze_mic_test.py ^
  --analyze-ref Data/reference.wav
```

The captured file is used as `--mic`, and `--analyze-ref` is used as `--ref`.

## 5) Binary Protocol

Host sends ASCII line:

- `REC 20000 MIC=1\n`

Device replies with binary:

- 32-byte header (`ANC1`, version 1)
- raw PCM payload (`int16`, little-endian)

Header layout (`<4sHHIIIHHII`):

1. magic (`ANC1`)
2. version
3. header size
4. sample rate
5. duration ms
6. mic id
7. bits per sample
8. channels
9. sample count
10. payload bytes

## 6) Debugging

Use this command to enable host-side serial debug output:

```bash
python tools/record_usb.py --port COM7 --mic 1 --duration-ms 3000 --debug-io
```

What you should see before binary transfer starts:

- `Device: STATUS ...` lines from preflight probe
- `Device: DBG rec start ...`
- then `Header: sr=...`

Useful firmware text commands (serial monitor):

- `STATUS` -> prints PSRAM/baud/pin summary
- `PING` -> returns `PONG`
- `ANC AUTOLEVEL TARGET_DB=10 HZ=1000 BASE_MS=1000 TONE_MS=1000 START_AMP=0.05 MIN_AMP=0.005 MAX_AMP=0.35 ITER=8` -> quiet tone auto-level from mic feedback
- `SELFTEST` (or `ANC SELFTEST`) -> runs on-device mic/DAC/phase diagnostics with `PASS/WARN/FAIL` lines
- `ANC TONE HZ=1000 AMP=0.85 MS=3000` -> forces strong DAC tone output (speaker audibility check)
- `ANC LOOPTEST HZ=1000 AMP=0.85 MS=2500 BASE_MS=1000 WAIT_MS=1500 THRESH_DB=8` -> baseline-vs-tone mic rise test (`PASS/FAIL`)

One-command host runner for auto-level + ANC start:

```bash
python tools/anc_autolevel.py --port COM7
```

ANC runtime debug counters in `STATUS`/`LOG ANC`:

- `ANC_RX_READ_ERRS`, `ANC_RX_EMPTY_READS`
- `ANC_TX_WRITE_ERRS`, `ANC_TX_SHORT_WRITES`

If TX counters rise while ANC runs, the DAC I2S stream is failing in real time.

If the script stalls waiting for header:

- Check for `ERR ...` lines (script now surfaces these)
- Run with `--debug-io` to see firmware text
- Ensure no other app has the COM port open

Quick self-test flow from serial monitor:

```text
SELFTEST
```

Expected summary lines:

- `SELFTEST MIC ... RESULT=PASS|WARN|FAIL`
- `SELFTEST DAC ... RESULT=PASS`
- `SELFTEST PHASE ... RESULT=PASS` (verifies `CALIBRATING -> ADAPTING -> FROZEN`)
- `SELFTEST RESULT=PASS|FAIL`

Quick acoustic loop test:

```text
ANC LOOPTEST
```

This reports baseline and tone-window dBFS for ref/error mics plus deltas.  
If `LOOPTEST RESULT=FAIL` with small deltas, digital DAC writes are likely fine but acoustic analog output is not coupling into the microphones.
`WAIT_MS` helps avoid keyboard/handling noise contaminating the baseline window.

## 7) Stage Prompts During Test Runs

`tools/record_pair_and_analyze.py` now prints explicit run-state prompts:

- Stage banners (`Baseline`, `Simultaneous RUN`, `Analyze`) by default
- Optional legacy banners (`Baseline`, `Reference`, `Test`, `Analyze`) with `--sequential-run`
- What action to take (STOP vs RUN condition)
- Countdown before each capture (`--countdown-s`, default `5`)
- Expected per-stage capture+transfer time
- File save locations for WAV and plot outputs

Useful flags:

- `--countdown-s <N>` set countdown length
- `--no-countdown` disable countdowns
- `--no-show-plot` save plots without opening windows
- `--no-save-plot` show only, do not save plots
- `--no-plot-fft` disable FFT and keep PSD-only output
- `--noise-alpha <value>` reduce over-subtraction dips (try `0.6` to `0.8`)
- `--noise-floor-frac <value>` raise subtraction floor (try `0.002` to `0.01`)
- `--sequential-run` use two separate mono run captures instead of one simultaneous stereo capture

## 8) Save FFT/PSD Plots to File

You can save plots directly from `analyze_mic_test.py`:

```bash
python analyze_mic_test.py --ref Data/pair_Test1_ref_mic2.wav --mic Data/pair_Test1_test_mic1.wav --save-plot Plots/Test1.png --show-after-save
```

Notes:

- `--save-plot` automatically enables plotting.
- `--show-after-save` keeps the window visible after saving.
- Pair runner defaults to both `..._fft.png` and `..._psd.png`.
- For direct `analyze_mic_test.py`, add `--plot-fft` to get both files; otherwise it saves PSD-only.
- If PSD has narrow deep dips, try:
  - `--noise-alpha 0.65`
  - `--noise-floor-frac 0.003`

## 9) Verify L/R Mapping (One-Step Check)

To confirm exactly which slot is louder (`Left` vs `Right`) and how that maps to `MIC=1/2`:

```bash
python tools/check_lr_mapping.py --port COM7 --near right
```

What this does:

- records one simultaneous stereo capture with `MIC=0`
- computes RMS/peak level for both channels
- reports channel mapping:
  - `ch0 -> MIC=1 -> slot from src/main.cpp`
  - `ch1 -> MIC=2 -> slot from src/main.cpp`
- prints pass/mismatch against your declared side (`--near left|right`)

Optional:

- `--duration-ms 4000` for longer check
- `--countdown-s 8` or `--no-countdown`
- `--debug-io` for serial debug lines

## 10) Derive Spectrum Band from Real Pump Recording (M4A)

To derive analysis band limits from your actual pump recording instead of dataset defaults:

```bash
python tools/derive_band_from_audio.py --input "Data/999.9 mLH.m4a" --save-plot "Plots/source_999_9_mLH_psd.png"
```

Current defaults in `analyze_mic_test.py` were set from this source:

- `DEFAULT_BAND_LOW = 30.0`
- `DEFAULT_BAND_HIGH = 2410.0`
