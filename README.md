# ANC Project (ESP32-S3 + INMP441 + PCM5102A)

This repository contains firmware and host-side tooling for an active noise control (ANC) prototype:

- Dual INMP441 I2S microphones on one shared data line (L/R slot split)
- ESP32-S3 real-time capture and ANC processing (FxLMS path)
- PCM5102A I2S DAC output to external amplifier/speaker
- USB serial capture protocol to save deterministic WAV files
- Python analysis workflow for lag alignment, RMS/peak dBFS, FFT/PSD, and noise-floor subtraction

This README is the project-level guide. For capture-focused details/examples, see `README_capture.md`.

## 1) Project Structure

- `src/main.cpp`
ANC firmware: capture protocol, FxLMS runtime, serial command interface, diagnostics (`SELFTEST`, `ANC TONE`, `ANC LOOPTEST`).

- `src/simple_tone_smoke.cpp`
Minimal TX-only I2S tone firmware used to isolate DAC/amp/speaker path.

- `platformio.ini`
PlatformIO environments and defaults.

- `tools/record_usb.py`
Triggers `REC ...` on device and saves binary payload as WAV.

- `tools/record_pair_and_analyze.py`
Automated baseline + run capture workflow and analysis invocation.

- `tools/check_lr_mapping.py`
One-shot simultaneous capture check to verify channel/mic mapping.

- `tools/derive_band_from_audio.py`
Derives recommended analysis band from source audio (WAV or ffmpeg-supported formats like M4A).

- `tools/anc_autolevel.py`
Runs on-device `ANC AUTOLEVEL` (quiet tone self-tuning from mic feedback) and optionally starts ANC with tuned defaults.

- `tools/anc_delay_sweep.py`
Sweeps ANC `DELAY` values, scores each run from `LOG ANC` in `FROZEN` phase, and optionally starts ANC with the best delay.

- `tools/play_test_tone.py`
Plays clear DAC test tones (`ANC TONE`) from PC with optional frequency sweep and safe ANC stop-first behavior.

- `analyze_mic_test.py`
Reference/test WAV analysis with auto alignment and spectral metrics.

- `run_pair.ps1`
PowerShell wrapper for `record_pair_and_analyze.py`.

## 2) Hardware Mapping (Current Firmware)

### Microphones (I2S RX)

- I2S port: `I2S_NUM_1`
- `BCLK` = GPIO `14`
- `WS/LRCK` = GPIO `15`
- `DATA IN` = GPIO `16`

### DAC (I2S TX)

- I2S port: `I2S_NUM_0`
- `BCLK` = GPIO `4`
- `WS/LRCK` = GPIO `5`
- `DATA OUT (DIN)` = GPIO `6`

### Mic slot mapping

- Frame format is `RIGHT_LEFT` (slot0=Right, slot1=Left).
- Firmware mapping:
  - `MIC=1` -> `Left` slot
  - `MIC=2` -> `Right` slot
- ANC role binding:
  - Reference = `Left`
  - Error = `Right`

Check current mapping at runtime with `STATUS`.

## 3) Build Environments

`platformio.ini` defines:

- `esp32s3` (default): full ANC/capture firmware (`src/main.cpp`)
- `esp32s3_tone_smoke`: minimal tone smoke firmware (`src/simple_tone_smoke.cpp`)

Default environment is set to `esp32s3`.

## 4) Prerequisites

- VS Code + PlatformIO, or PlatformIO CLI
- Python 3.10+ recommended
- Python packages:

```bash
pip install pyserial numpy matplotlib
```

Optional for non-WAV source-band derivation (M4A input):

- `ffmpeg` on PATH

## 5) Flash and Monitor

Build/upload default ANC firmware:

```bash
python -m platformio run -t upload --upload-port COM7
```

Open serial monitor:

```bash
python -m platformio device monitor -p COM7 -b 115200
```

Recommended monitor settings:

- baud `115200`
- line ending `LF` (firmware also handles no line ending via idle flush)
- only one app owning COM port at a time

### Quiet startup recipe

Use this serial command sequence right after flashing when you want the lowest-risk bring-up:

```text
ANC STOP
STATUS
ANC START CAL_SILENT=1 CAL_MS=1200 ADAPT_MS=3000 LOG_MS=1000 GAIN=0.20 LIMIT_DBFS=-18 DELAY=0 LEAK=0.00005 ADAPT_FLOOR_DBFS=-65 HOWL=1 HOWL_THRESH_DB=8 HOWL_WINDOW_MS=120 HOWL_HOLD_MS=1000 HOWL_MIN_GAIN=0.15
STATUS
```

What this does:

- Runs calibration silently (`CAL_SILENT=1`), so no intentional calibration noise is sent to speaker.
- Starts with low output gain and stricter limiter.
- Keeps howling guard enabled with aggressive protection defaults.

## 6) Firmware Command Reference

### General

- `PING`
- `STATUS`
- `REC <ms> MIC=<0|1|2>`

`REC` returns binary payload after a header:

- `MIC=1` or `2`: mono
- `MIC=0`: stereo simultaneous capture (`ch0=mic1`, `ch1=mic2`)

### ANC controls

- `ANC START [CAL_MS=3000 ADAPT_MS=7000 MU=0.002 MUCAL=0.12 GAIN=1.0 LIMIT_DBFS=-6 CAL_AMP=0.20 CAL_OUT=1.0 CAL_SILENT=0 SIGN=-1 LOG_MS=1000 DELAY=0 LEAK=0.00005 ADAPT_FLOOR_DBFS=-70 HP_HZ=70 LP_HZ=2200 GATE_DBFS=-52 HOWL=1 HOWL_THRESH_DB=10 HOWL_WINDOW_MS=120 HOWL_HOLD_MS=800 HOWL_MIN_GAIN=0.20]`
- `ANC DIRECT START [GAIN=0.15 LIMIT_DBFS=-20 SIGN=-1 LOG_MS=400 DELAY=0 HP_HZ=70 LP_HZ=2200 GATE_DBFS=-52]`
- `ANC STOP`
- `ANC FREEZE`
- `ANC ADAPT [ADAPT_MS=0]`
- `ANC AUTOLEVEL [TARGET_DB=10 HZ=1000 BASE_MS=1000 TONE_MS=1000 START_AMP=0.05 MIN_AMP=0.005 MAX_AMP=0.35 ITER=8]`
- `ANC RESET`
- `ANC STATUS`

Notes:

- `ADAPT_MS=0` means adapt indefinitely.
- `ANC START` resets to defaults each run unless overridden by options.
- `CAL_OUT` controls audible calibration output level (`0..1`), while still running secondary-path identification.
- `CAL_SILENT=1` is an alias for `CAL_OUT=0` (silent calibration).
- `ANC DIRECT START` uses a true direct path: delayed reference inversion (`ref * SIGN * GAIN`) plus limiter, with no FxLMS adaptation updates.
- Direct mode also supports optional conditioning:
  - `HP_HZ` high-pass, `LP_HZ` low-pass, `GATE_DBFS` envelope gate
- `ANC AUTOLEVEL` stores runtime tuned values:
  - `ANC_RUNTIME_TONE_AMP`
  - `ANC_RUNTIME_CAL_AMP`
  - `ANC_RUNTIME_CAL_AMP_TUNED=1`
- `ANC START` uses the latest tuned `ANC_RUNTIME_CAL_AMP` unless `CAL_AMP=...` is passed explicitly.

### Diagnostics

- `SELFTEST` (same as `ANC SELFTEST`)
- `ANC SELFTEST`
- `ANC TONE [HZ=1000 AMP=0.85 MS=3000]`
- `LOOPTEST` (same as `ANC LOOPTEST` defaults)
- `ANC LOOPTEST [HZ=1000 AMP=0.85 MS=2500 BASE_MS=1000 WAIT_MS=1500 THRESH_DB=8]`

What they validate:

- `SELFTEST`: mic activity, DAC write path, ANC phase transition.
- `ANC TONE`: force DAC output without ANC adaptation logic.
- `ANC LOOPTEST`: compare baseline mic levels vs tone window and report `PASS/FAIL` from dB rise.

`STATUS` includes runtime counters:

- `ANC_RX_READ_ERRS`, `ANC_RX_EMPTY_READS`
- `ANC_TX_WRITE_ERRS`, `ANC_TX_SHORT_WRITES`

## 7) Minimal DAC Smoke Test Firmware

To isolate hardware output path from ANC logic:

```bash
python -m platformio run -e esp32s3_tone_smoke -t upload --upload-port COM7
python -m platformio device monitor -p COM7 -b 115200
```

Expected serial output includes:

- `I2S tone smoke test ...`
- `Pins: BCLK=4 LRCK=5 DIN=6`
- `I2S started. Continuous square tone should be audible.`

Return to ANC firmware:

```bash
python -m platformio run -e esp32s3 -t upload --upload-port COM7
```

## 8) Host Capture and Analysis Workflow

### Single capture

```bash
python tools/record_usb.py --port COM7 --mic 1 --duration-ms 20000 --out Data/capture_mic1.wav
```

### Simultaneous capture (both mics)

```bash
python tools/record_usb.py --port COM7 --mic 0 --duration-ms 3000 --out Data/baseline_stereo.wav
```

### Pair run + analysis (recommended)

```bash
python tools/record_pair_and_analyze.py --port COM7 --duration-ms 20000
```

PowerShell shortcut:

```powershell
.\run_pair.ps1
```

### Quiet auto-level then start ANC

```bash
python tools/anc_autolevel.py --port COM7
```

This command:

1. Probes device status
2. Runs `ANC AUTOLEVEL` (short quiet tone iterations against mic baseline)
3. Stores tuned amplitudes in firmware runtime state
4. Starts ANC using tuned `CAL_AMP`

### Delay sweep then start best setting

```bash
python tools/anc_delay_sweep.py --port COM7
```

Default sweep is intentionally short (`DELAY=0,8,16`) to reduce test time.

Example with tighter sweep:

```bash
python tools/anc_delay_sweep.py --port COM7 --delay-start 8 --delay-end 40 --delay-step 2
```

Run sweep in direct invert mode (no FxLMS adaptation):

```bash
python tools/anc_delay_sweep.py --port COM7 --direct --start-best
```

### Clear tone test (speaker chain)

Single quiet tone:

```bash
python tools/play_test_tone.py --port COM7 --hz 1000 --amp 0.08 --ms 2500
```

Stepped sweep:

```bash
python tools/play_test_tone.py --port COM7 --amp 0.06 --ms 1200 --sweep-start-hz 200 --sweep-end-hz 3000 --sweep-step-hz 200
```

Defaults:

- simultaneous run capture
- baseline capture enabled (`3000 ms`)
- `ref-mic` default `2`
- plots saved under `Plots/`
- FFT plotting enabled in pair workflow

### Verify L/R mapping

```bash
python tools/check_lr_mapping.py --port COM7 --near left
```

### Derive analysis band from source audio

```bash
python tools/derive_band_from_audio.py --input "Data/999.9 mLH.m4a" --save-plot Plots/source_psd.png
```

## 9) Analysis Script (`analyze_mic_test.py`)

Core outputs:

- auto lag alignment (unless `--no-auto-align`)
- raw RMS/peak dBFS
- band-limited RMS ratio and correlation
- optional Welch PSD (+ smoothing + optional noise subtraction)
- optional FFT comparison plot

Typical call:

```bash
python analyze_mic_test.py --ref Data/pair_Test_ref_mic2.wav --mic Data/pair_Test_test_mic1.wav --plot --plot-fft --save-plot Plots/pair_Test.png --show-after-save
```

Current default analysis band:

- `30 Hz` to `2410 Hz`

## 10) Common Troubleshooting

### No audible output but firmware reports success

Use this order:

1. `ANC TONE HZ=1000 AMP=0.85 MS=5000`
2. Check `STATUS` counters for TX errors (`ANC_TX_WRITE_ERRS`, `ANC_TX_SHORT_WRITES`)
3. Run `ANC LOOPTEST` to see if tone energy reaches mics acoustically
4. If digital path is clean but loop test fails, inspect hardware chain (DAC -> amp -> speaker)

### Port busy / access denied

- Close all serial monitors/tools before running Python capture scripts.

### Gibberish serial output

- Ensure monitor baud is `115200`.

### Upload fails (`Error 2`)

- Close monitor/capture scripts.
- Replug USB if needed.
- Hold `BOOT` while uploading if board auto-reset is unreliable.

## 11) Data and Git Behavior

Generated artifacts are intentionally ignored in git:

- `Data/`, `data/`
- `Plots/`, `plots/`
- `Processed_Data/`, `processed_data/`, `processed-data/`

This keeps the repository focused on firmware/scripts while letting you run local experiments freely.

## 12) Legacy / Supplemental Docs

- `README_capture.md` remains as a detailed capture/analysis reference with extra examples.
