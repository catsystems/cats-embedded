[CmdletBinding()]
param(
  [Parameter(Position = 0, Mandatory = $true)]
  [ValidateSet('setup', 'serve', 'run', 'test')]
  [string]$Command,

  [Parameter(Position = 1)]
  [string]$Scenario,

  [int]$Port = 8787
)

$ErrorActionPreference = 'Stop'
$SimulatorRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$GroundStationRoot = Split-Path -Parent $SimulatorRoot
$RepoRoot = Split-Path -Parent $GroundStationRoot
$PinnedEmscripten = '6.0.6'
$PinnedReleaseCommit = '833aa203ba2283fc2b6adb504a79a3a0d692df81'
$ToolRoot = Join-Path $env:LOCALAPPDATA 'CATS\tools'
$SdkRoot = Join-Path $ToolRoot ('emsdk-' + $PinnedEmscripten)
$PythonPath = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'
$PlatformIoPython = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'

function Resolve-Python {
  $candidate = Get-Command python -ErrorAction SilentlyContinue
  if ($null -ne $candidate) { return $candidate.Source }
  if (Test-Path -LiteralPath $PythonPath) { return $PythonPath }
  if (Test-Path -LiteralPath $PlatformIoPython) { return $PlatformIoPython }
  throw 'Python 3 is required for the dependency-free headless runner.'
}

function Get-EmsdkPath {
  $candidate = Join-Path $SdkRoot 'emsdk.bat'
  if (Test-Path -LiteralPath $candidate) { return $candidate }
  return $null
}

function Get-EmscriptenExecutable {
  param([Parameter(Mandatory = $true)][string]$Name)
  foreach ($extension in @('.exe', '.bat')) {
    $candidate = Join-Path $SdkRoot ("upstream\emscripten\$Name$extension")
    if (Test-Path -LiteralPath $candidate) { return $candidate }
  }
  return $null
}

function Initialize-EmscriptenEnvironment {
  $python = Get-ChildItem -LiteralPath (Join-Path $SdkRoot 'python') -Filter 'python.exe' -Recurse -File |
    Select-Object -First 1
  $node = Get-ChildItem -LiteralPath (Join-Path $SdkRoot 'node') -Filter 'node.exe' -Recurse -File |
    Select-Object -First 1
  if ($null -eq $python -or $null -eq $node) {
    throw "Emscripten $PinnedEmscripten is incomplete. Run setup again."
  }

  $emscriptDir = Join-Path $SdkRoot 'upstream\emscripten'
  $llvmDir = Join-Path $SdkRoot 'upstream\bin'
  $env:EMSDK = $SdkRoot
  $env:EM_CONFIG = Join-Path $SdkRoot '.emscripten'
  $env:EMSDK_PYTHON = $python.FullName
  $env:EMSDK_NODE = $node.FullName
  $env:PATH = @(
    $python.Directory.FullName,
    $node.Directory.FullName,
    $emscriptDir,
    $llvmDir,
    $env:PATH
  ) -join ';'
}

function Invoke-Emsdk {
  param([Parameter(Mandatory = $true)][string[]]$Arguments)
  $emsdk = Get-EmsdkPath
  if ($null -eq $emsdk) { throw "Pinned Emscripten SDK is not installed. Run '$($MyInvocation.MyCommand.Name) setup'." }
  Push-Location $SdkRoot
  try {
    & $emsdk @Arguments
    if ($LASTEXITCODE -ne 0) { throw "emsdk $($Arguments -join ' ') failed with exit code $LASTEXITCODE." }
  } finally { Pop-Location }
}

function Setup-Simulator {
  New-Item -ItemType Directory -Force -Path $ToolRoot | Out-Null
  $emsdk = Get-EmsdkPath
  if ($null -eq $emsdk) {
    $git = Get-Command git -ErrorAction SilentlyContinue
    if ($null -eq $git) { throw 'Git is required to install the pinned Emscripten SDK.' }
    & $git.Source clone --depth 1 --branch main https://github.com/emscripten-core/emsdk.git $SdkRoot
  }
  Invoke-Emsdk -Arguments @('install', $PinnedEmscripten)
  Invoke-Emsdk -Arguments @('activate', $PinnedEmscripten)
  if ($null -eq (Get-EmscriptenExecutable -Name 'emcc')) {
    throw "Emscripten $PinnedEmscripten installed without emcc."
  }
  $marker = Join-Path $SdkRoot 'CATS-emscripten-version.txt'
  Set-Content -LiteralPath $marker -Value "version=$PinnedEmscripten`nreleaseCommit=$PinnedReleaseCommit" -Encoding UTF8
  Write-Host "Emscripten $PinnedEmscripten is ready under $SdkRoot"
}

function Invoke-Headless {
  param([Parameter(Mandatory = $true)][string[]]$Arguments)
  $python = Resolve-Python
  & $python (Join-Path $SimulatorRoot 'gs_sim.py') @Arguments
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

function Build-WasmDirect {
  $empp = Get-EmscriptenExecutable -Name 'em++'
  if ($null -eq $empp) { throw "Emscripten $PinnedEmscripten is missing em++." }
  $compileArgs = @(
    '-std=c++20',
    '-DARDUINO=100',
    '-DGS_SIMULATOR_WASM=1',
    ("-I$(Join-Path $SimulatorRoot 'compat')"),
    ("-I$(Join-Path $GroundStationRoot 'lib\Adafruit_GFX_Library')"),
    ("-I$(Join-Path $GroundStationRoot 'src')"),
    ("-I$(Join-Path $GroundStationRoot 'src\hmi')"),
    (Join-Path $SimulatorRoot 'wasm_api.cpp'),
    (Join-Path $SimulatorRoot 'window_hmi_renderer.cpp'),
    (Join-Path $GroundStationRoot 'lib\Adafruit_GFX_Library\Adafruit_GFX.cpp'),
    (Join-Path $SimulatorRoot 'hmi_controller.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\settings.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\window.cpp'),
    '-o',
    (Join-Path $SimulatorRoot 'web\gs-sim.js'),
    '-sWASM=1',
    '-sMODULARIZE=1',
    '-sEXPORT_ES6=1',
    '-sENVIRONMENT=web',
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8']",
    "-sEXPORTED_FUNCTIONS=['_gs_reset','_gs_press','_gs_release','_gs_hold','_gs_advance','_gs_set_link_json','_gs_set_navigation_json','_gs_set_sensor_json','_gs_set_device_status_json','_gs_set_configuration_json','_gs_set_logs_json','_gs_load_replay_json','_gs_snapshot_json','_gs_framebuffer','_gs_framebuffer_size','_gs_framebuffer_revision']"
  )
  & $empp @compileArgs
  if ($LASTEXITCODE -ne 0) { throw 'Direct WebAssembly build failed.' }
  if (!(Test-Path -LiteralPath (Join-Path $SimulatorRoot 'web\gs-sim.js'))) {
    throw 'Direct WebAssembly build completed without gs-sim.js.'
  }
}

function Build-WasmIfAvailable {
  $emcc = Get-EmscriptenExecutable -Name 'emcc'
  if ($null -eq $emcc) {
    Write-Warning "Emscripten $PinnedEmscripten is not installed; serving the deterministic Python-backed browser bundle. Run setup for WebAssembly builds."
    return
  }
  Initialize-EmscriptenEnvironment
  $buildRoot = Join-Path $SimulatorRoot 'build-wasm'
  New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
  $emccVersion = & $emcc --version
  if ($LASTEXITCODE -ne 0) { throw 'Unable to execute the Emscripten compiler.' }
  Write-Host $emccVersion[0]
  $cmake = Get-Command cmake -ErrorAction SilentlyContinue
  if ($null -eq $cmake) {
    Write-Warning 'CMake is not installed; using the direct Emscripten compiler path.'
    Build-WasmDirect
    return
  }
  $toolchain = Join-Path $SdkRoot 'upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake'
  & cmake -S $SimulatorRoot -B $buildRoot "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
  & cmake --build $buildRoot --config Release
  if ($LASTEXITCODE -ne 0) { throw 'WebAssembly build failed.' }
  $generated = Get-ChildItem -LiteralPath $buildRoot -Filter 'gs-sim.js' -Recurse | Select-Object -First 1
  if ($null -eq $generated) { throw 'WebAssembly build completed without gs-sim.js.' }
  Copy-Item -LiteralPath $generated.FullName -Destination (Join-Path $SimulatorRoot 'web\gs-sim.js') -Force
}

switch ($Command) {
  'setup' { Setup-Simulator }
  'run' {
    if ([string]::IsNullOrWhiteSpace($Scenario)) { throw 'run requires a scenario JSON path.' }
    Invoke-Headless @('run', (Resolve-Path -LiteralPath $Scenario).Path, '--write-snapshots')
  }
  'test' {
    Invoke-Headless @('test', '--root', $RepoRoot)
  }
  'serve' {
    Build-WasmIfAvailable
    $python = Resolve-Python
    $webRoot = Join-Path $SimulatorRoot 'web'
    $server = Start-Process -FilePath $python -ArgumentList '-m', 'http.server', $Port, '--directory', $webRoot -PassThru -WindowStyle Hidden
    Start-Process "http://127.0.0.1:$Port/index.html"
    Write-Host "Ground Station simulator: http://127.0.0.1:$Port/index.html"
    Write-Host "Server process id: $($server.Id)"
  }
}
