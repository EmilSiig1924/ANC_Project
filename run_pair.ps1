param(
    [string]$Port = "COM7",
    [int]$DurationMs = 20000,
    [int]$BaselineMs = 3000,
    [ValidateSet(1, 2)]
    [int]$RefMic = 2,
    [int]$Baud = 115200,
    [double]$TimeoutS = 300.0,
    [int]$CountdownS = 5,
    [string]$Tag = "",
    [string]$PlotsDir = "Plots",
    [string]$SavePlot = "",
    [double]$NoiseAlpha = -1,
    [double]$NoiseFloorFrac = -1,
    [switch]$SequentialRun,
    [switch]$PlotFFT,
    [switch]$NoPlotFFT,
    [switch]$NoCountdown,
    [switch]$NoPlot,
    [switch]$NoSavePlot,
    [switch]$NoShowPlot,
    [switch]$DebugIO,
    [switch]$SkipPreflight
)

$ErrorActionPreference = "Stop"

$repoRoot = $PSScriptRoot
$runnerScript = Join-Path $repoRoot "tools\record_pair_and_analyze.py"

if (-not (Test-Path $runnerScript)) {
    throw "Missing script: $runnerScript"
}

$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
if (Test-Path $venvPython) {
    $pythonExe = $venvPython
} else {
    $pythonExe = "python"
}

$args = @(
    $runnerScript,
    "--port", $Port,
    "--duration-ms", $DurationMs,
    "--baseline-ms", $BaselineMs,
    "--ref-mic", $RefMic,
    "--baud", $Baud,
    "--timeout-s", $TimeoutS,
    "--countdown-s", $CountdownS,
    "--plots-dir", $PlotsDir
)

if ($Tag -ne "") {
    $args += @("--tag", $Tag)
}

if ($NoPlotFFT) {
    $args += "--no-plot-fft"
} elseif ($PlotFFT) {
    $args += "--plot-fft"
}

if ($NoCountdown) {
    $args += "--no-countdown"
}

if ($SequentialRun) {
    $args += "--sequential-run"
}

if ($SavePlot -ne "") {
    $args += @("--save-plot", $SavePlot)
}

if ($NoiseAlpha -ge 0) {
    $args += @("--noise-alpha", $NoiseAlpha)
}

if ($NoiseFloorFrac -ge 0) {
    $args += @("--noise-floor-frac", $NoiseFloorFrac)
}

if ($NoPlot) {
    $args += "--no-plot"
}

if ($NoSavePlot) {
    $args += "--no-save-plot"
}

if ($NoShowPlot) {
    $args += "--no-show-plot"
}

if ($DebugIO) {
    $args += "--debug-io"
}

if ($SkipPreflight) {
    $args += "--skip-preflight"
}

Write-Host "Running pair capture + analysis..."
Write-Host "Python:" $pythonExe
Write-Host "Script:" $runnerScript
Write-Host "Port=$Port DurationMs=$DurationMs BaselineMs=$BaselineMs RefMic=$RefMic Baud=$Baud TimeoutS=$TimeoutS CountdownS=$CountdownS PlotsDir=$PlotsDir NoiseAlpha=$NoiseAlpha NoiseFloorFrac=$NoiseFloorFrac SequentialRun=$SequentialRun"

& $pythonExe @args
exit $LASTEXITCODE
