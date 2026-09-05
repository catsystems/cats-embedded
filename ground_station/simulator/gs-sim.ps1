[CmdletBinding()]
param(
  [Parameter(Position = 0, Mandatory = $true)]
  [ValidateSet('setup', 'build', 'browser-test', 'fatfs-test', 'fixture', 'serve', 'run', 'test')]
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
$PinnedEmsdkCommit = '9981799f744be74ac67b1c1813ff172f63be0630'
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

function Set-PinnedEmsdkRevision {
  $git = Get-Command git -ErrorAction SilentlyContinue
  if ($null -eq $git) { throw 'Git is required to verify the pinned Emscripten SDK.' }

  $head = (& $git.Source -C $SdkRoot rev-parse HEAD).Trim()
  if ($LASTEXITCODE -ne 0) { throw "Unable to inspect the Emscripten SDK under $SdkRoot." }
  if ($head -eq $PinnedEmsdkCommit) { return }

  & $git.Source -C $SdkRoot fetch --depth 1 origin tag $PinnedEmscripten
  if ($LASTEXITCODE -ne 0) { throw "Unable to fetch Emscripten SDK tag $PinnedEmscripten." }
  & $git.Source -C $SdkRoot checkout --detach $PinnedEmsdkCommit
  if ($LASTEXITCODE -ne 0) { throw "Unable to check out Emscripten SDK commit $PinnedEmsdkCommit." }

  $head = (& $git.Source -C $SdkRoot rev-parse HEAD).Trim()
  if ($head -ne $PinnedEmsdkCommit) {
    throw "Emscripten SDK revision mismatch: expected $PinnedEmsdkCommit, found $head."
  }
}

function Setup-Simulator {
  New-Item -ItemType Directory -Force -Path $ToolRoot | Out-Null
  $emsdk = Get-EmsdkPath
  if ($null -eq $emsdk) {
    $git = Get-Command git -ErrorAction SilentlyContinue
    if ($null -eq $git) { throw 'Git is required to install the pinned Emscripten SDK.' }
    & $git.Source clone --depth 1 --branch $PinnedEmscripten https://github.com/emscripten-core/emsdk.git $SdkRoot
    if ($LASTEXITCODE -ne 0) { throw "Unable to clone Emscripten SDK tag $PinnedEmscripten." }
  }
  Set-PinnedEmsdkRevision
  Invoke-Emsdk -Arguments @('install', $PinnedEmscripten)
  Invoke-Emsdk -Arguments @('activate', $PinnedEmscripten)
  $emcc = Get-EmscriptenExecutable -Name 'emcc'
  if ($null -eq $emcc) {
    throw "Emscripten $PinnedEmscripten installed without emcc."
  }
  Initialize-EmscriptenEnvironment
  $emccVersion = (& $emcc --version) -join "`n"
  if ($LASTEXITCODE -ne 0 -or $emccVersion -notmatch "emcc.*$([regex]::Escape($PinnedEmscripten))") {
    throw "Emscripten version mismatch. Expected $PinnedEmscripten, received: $emccVersion"
  }
  $marker = Join-Path $SdkRoot 'CATS-emscripten-version.txt'
  Set-Content -LiteralPath $marker -Value "version=$PinnedEmscripten`nemsdkCommit=$PinnedEmsdkCommit" -Encoding UTF8
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
    ("-I$(Join-Path $GroundStationRoot 'lib\Adafruit_BusIO')"),
    ("-I$(Join-Path $GroundStationRoot 'lib\NayukiQRCode\src')"),
    ("-I$(Join-Path $GroundStationRoot 'src')"),
    ("-I$(Join-Path $GroundStationRoot 'src\hmi')"),
    (Join-Path $SimulatorRoot 'wasm_api.cpp'),
    (Join-Path $SimulatorRoot 'window_hmi_renderer.cpp'),
    (Join-Path $GroundStationRoot 'lib\Adafruit_GFX_Library\Adafruit_GFX.cpp'),
    (Join-Path $GroundStationRoot 'lib\NayukiQRCode\src\qrcodegen.cpp'),
    (Join-Path $SimulatorRoot 'hmi_controller.cpp'),
    (Join-Path $SimulatorRoot 'self_test_fixture.cpp'),
    (Join-Path $GroundStationRoot 'src\self_test.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\window_self_test.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\location_qr.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\settings.cpp'),
    (Join-Path $GroundStationRoot 'src\hmi\window.cpp'),
    '-o',
    (Join-Path $SimulatorRoot 'web\gs-sim.js'),
    '-sWASM=1',
    '-sSTACK_SIZE=262144',
    '-sMODULARIZE=1',
    '-sEXPORT_ES6=1',
    '-sENVIRONMENT=web',
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8']",
    "-sEXPORTED_FUNCTIONS=['_gs_reset','_gs_restart','_gs_press','_gs_release','_gs_hold','_gs_advance','_gs_set_link_json','_gs_set_navigation_json','_gs_set_sensor_json','_gs_set_device_status_json','_gs_set_configuration_json','_gs_set_logs_json','_gs_load_replay_json','_gs_snapshot_json','_gs_framebuffer','_gs_framebuffer_size','_gs_framebuffer_revision']"
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
  $emccVersion = & $emcc --version
  if ($LASTEXITCODE -ne 0) { throw 'Unable to execute the Emscripten compiler.' }
  Write-Host $emccVersion[0]
  Build-WasmDirect
}

function Resolve-HeadlessBrowser {
  $candidates = @(
    (Join-Path ${env:ProgramFiles(x86)} 'Microsoft\Edge\Application\msedge.exe'),
    (Join-Path $env:ProgramFiles 'Microsoft\Edge\Application\msedge.exe'),
    (Join-Path $env:LOCALAPPDATA 'Microsoft\Edge\Application\msedge.exe'),
    (Join-Path $env:ProgramFiles 'Google\Chrome\Application\chrome.exe'),
    (Join-Path ${env:ProgramFiles(x86)} 'Google\Chrome\Application\chrome.exe')
  )
  foreach ($candidate in $candidates) {
    if (Test-Path -LiteralPath $candidate) { return $candidate }
  }
  throw 'Microsoft Edge or Google Chrome is required for browser-test.'
}

function Get-FreeTcpPort {
  $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
  $listener.Start()
  try { return ([System.Net.IPEndPoint]$listener.LocalEndpoint).Port }
  finally { $listener.Stop() }
}

function Invoke-BrowserRegressionTest {
  $emcc = Get-EmscriptenExecutable -Name 'emcc'
  if ($null -eq $emcc) { throw "Emscripten $PinnedEmscripten is required for browser-test." }
  Initialize-EmscriptenEnvironment
  Build-WasmDirect

  $python = Resolve-Python
  $browser = Resolve-HeadlessBrowser
  $webRoot = Join-Path $SimulatorRoot 'web'
  $testPort = Get-FreeTcpPort
  $profileRoot = Join-Path ([System.IO.Path]::GetTempPath()) ('cats-gs-browser-' + [guid]::NewGuid().ToString('N'))
  New-Item -ItemType Directory -Path $profileRoot | Out-Null
  $server = Start-Process -FilePath $python -ArgumentList '-m', 'http.server', $testPort, '--directory', $webRoot -PassThru -WindowStyle Hidden
  try {
    $url = "http://127.0.0.1:$testPort/browser-tests.html"
    $ready = $false
    foreach ($attempt in 1..50) {
      try {
        Invoke-WebRequest -UseBasicParsing -Uri $url | Out-Null
        $ready = $true
        break
      } catch {
        Start-Sleep -Milliseconds 100
      }
    }
    if (!$ready) { throw 'The browser-test web server did not start.' }

    $browserArguments = @(
      '--headless=new', '--disable-gpu', '--no-first-run', '--no-default-browser-check',
      "--user-data-dir=$profileRoot", '--virtual-time-budget=10000', '--dump-dom', $url
    )
    $standardOutput = Join-Path $profileRoot 'browser-test.html'
    $standardError = Join-Path $profileRoot 'browser-test.stderr'
    $browserProcess = Start-Process -FilePath $browser -ArgumentList $browserArguments -RedirectStandardOutput $standardOutput -RedirectStandardError $standardError -PassThru -Wait -WindowStyle Hidden
    if ($browserProcess.ExitCode -ne 0) { throw "Headless browser exited with code $($browserProcess.ExitCode)." }
    $dom = Get-Content -LiteralPath $standardOutput -Raw
    if ($dom -notmatch 'data-status="passed"') {
      $result = [regex]::Match($dom, '<pre id="result"[^>]*>(.*?)</pre>', 'Singleline').Groups[1].Value
      throw "Browser/WASM regression test failed: $result"
    }
    $reportJson = [regex]::Match($dom, '<pre id="result"[^>]*>(.*?)</pre>', 'Singleline').Groups[1].Value
    $report = [System.Net.WebUtility]::HtmlDecode($reportJson) | ConvertFrom-Json
    Write-Host "browser/WASM: $($report.results.Count) regression checks passed"
  } finally {
    if (!$server.HasExited) { Stop-Process -Id $server.Id -Force }
    Remove-Item -LiteralPath $profileRoot -Recurse -Force -ErrorAction SilentlyContinue
  }
}

function Invoke-FatFsCompatibilityTest {
  $emcc = Get-EmscriptenExecutable -Name 'emcc'
  if ($null -eq $emcc) { throw "Emscripten $PinnedEmscripten is required for the FatFs compatibility test." }
  Initialize-EmscriptenEnvironment

  $testRoot = Join-Path $GroundStationRoot 'tests'
  $fatFsRoot = Join-Path $GroundStationRoot 'lib\FatFs'
  $buildRoot = Join-Path $GroundStationRoot '.pio\fatfs-test'
  $compressedFixture = Join-Path $testRoot 'fixtures\fatfs-r013c-sfd.img.gz'
  $legacyImage = Join-Path $buildRoot 'fatfs-r013c-sfd.img'
  $newImage = Join-Path $buildRoot 'fatfs-r016-sfd.img'
  $runner = Join-Path $buildRoot 'fatfs-compatibility.js'
  New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null

  $input = [System.IO.File]::OpenRead($compressedFixture)
  $output = [System.IO.File]::Create($legacyImage)
  $gzip = New-Object System.IO.Compression.GZipStream($input, [System.IO.Compression.CompressionMode]::Decompress)
  try {
    $gzip.CopyTo($output)
  } finally {
    $gzip.Dispose()
    $output.Dispose()
    $input.Dispose()
  }

  $compileArgs = @(
    (Join-Path $testRoot 'fatfs_compatibility.c'),
    (Join-Path $fatFsRoot 'ff.c'),
    (Join-Path $fatFsRoot 'ffunicode.c'),
    "-I$fatFsRoot",
    '-std=c11',
    '-O2',
    '-sNODERAWFS=1',
    '-sEXIT_RUNTIME=1',
    '-o',
    $runner
  )
  & $emcc @compileArgs
  if ($LASTEXITCODE -ne 0) { throw 'FatFs compatibility harness compilation failed.' }

  & $env:EMSDK_NODE $runner validate $legacyImage
  if ($LASTEXITCODE -ne 0) { throw 'FatFs R0.16 could not safely read and update the R0.13c fixture.' }
  & $env:EMSDK_NODE $runner create $newImage
  if ($LASTEXITCODE -ne 0) { throw 'FatFs R0.16 could not create a test volume.' }
  & $env:EMSDK_NODE $runner validate $newImage
  if ($LASTEXITCODE -ne 0) { throw 'FatFs R0.16 could not remount its test volume safely.' }
  Write-Host 'FatFs legacy/read-write/corrupt-media compatibility tests passed.'
}

switch ($Command) {
  'setup' { Setup-Simulator }
  'build' { Build-WasmIfAvailable }
  'browser-test' { Invoke-BrowserRegressionTest }
  'fatfs-test' { Invoke-FatFsCompatibilityTest }
  'run' {
    if ([string]::IsNullOrWhiteSpace($Scenario)) { throw 'run requires a scenario JSON path.' }
    Invoke-Headless @('run', (Resolve-Path -LiteralPath $Scenario).Path, '--write-snapshots')
  }
  'fixture' {
    if ([string]::IsNullOrWhiteSpace($Scenario)) { throw 'fixture requires a fixture id.' }
    Invoke-Headless @('fixture', $Scenario, '--manifest', (Join-Path $SimulatorRoot 'web\fixtures.json'))
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
