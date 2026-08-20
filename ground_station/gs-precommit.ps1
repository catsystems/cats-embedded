[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$GroundStationRoot = $PSScriptRoot
$RepoRoot = Split-Path -Parent $GroundStationRoot
$ExpectedPlatformIoVersion = '6.1.19'

function Invoke-CommandChecked {
  param(
    [Parameter(Mandatory = $true)][string]$FilePath,
    [Parameter(Mandatory = $true)][string[]]$Arguments
  )

  & $FilePath @Arguments
  if ($LASTEXITCODE -ne 0) {
    throw "$FilePath $($Arguments -join ' ') failed with exit code $LASTEXITCODE."
  }
}

function Resolve-PlatformIo {
  $command = Get-Command platformio -ErrorAction SilentlyContinue
  if ($null -ne $command) { return $command.Source }

  $candidate = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
  if (Test-Path -LiteralPath $candidate) { return $candidate }

  throw 'PlatformIO 6.1.19 is required. Install the root requirements before running this script.'
}

function ConvertTo-WslPath {
  param([Parameter(Mandatory = $true)][string]$WindowsPath)

  $fullPath = [System.IO.Path]::GetFullPath($WindowsPath)
  if ($fullPath -notmatch '^(?<drive>[A-Za-z]):[\\/](?<rest>.*)$') {
    throw "Only local Windows paths can be mapped into WSL: $fullPath"
  }

  $drive = $Matches.drive.ToLowerInvariant()
  $rest = $Matches.rest.Replace('\', '/')
  return "/mnt/$drive/$rest"
}

function Invoke-WslScript {
  param(
    [Parameter(Mandatory = $true)][string]$Script,
    [string[]]$Arguments = @()
  )

  $temporaryScript = Join-Path ([System.IO.Path]::GetTempPath()) ("cats-gs-precommit-$([guid]::NewGuid()).sh")
  try {
    $unixScript = $Script.Replace("`r`n", "`n")
    [System.IO.File]::WriteAllText($temporaryScript, $unixScript, [System.Text.UTF8Encoding]::new($false))
    $wslScript = ConvertTo-WslPath -WindowsPath $temporaryScript

    & wsl.exe -- bash $wslScript @Arguments
    if ($LASTEXITCODE -ne 0) {
      throw "The Ground Station Linux checks failed with exit code $LASTEXITCODE."
    }
  } finally {
    Remove-Item -LiteralPath $temporaryScript -Force -ErrorAction SilentlyContinue
  }
}

$platformIo = Resolve-PlatformIo
$platformIoVersion = (& $platformIo --version) -join "`n"
if ($LASTEXITCODE -ne 0 -or $platformIoVersion -notmatch "\b$([regex]::Escape($ExpectedPlatformIoVersion))\b") {
  throw "PlatformIO $ExpectedPlatformIoVersion is required. Found: $platformIoVersion"
}

$wsl = Get-Command wsl.exe -ErrorAction SilentlyContinue
if ($null -eq $wsl) {
  throw 'WSL with Ubuntu, Python 3.11, and python3.11-venv is required for clang-format and clang-tidy.'
}

$wslRepoRoot = ConvertTo-WslPath -WindowsPath $RepoRoot
if ($wslRepoRoot.Contains("'")) {
  throw "The repository path cannot contain a single quote: $wslRepoRoot"
}
$escapedWslRepoRoot = $wslRepoRoot

$changedSourceFiles = @(
  & git -C $RepoRoot diff --name-only --diff-filter=ACMR HEAD -- ground_station/src
  & git -C $RepoRoot ls-files --others --exclude-standard -- ground_station/src
) | Sort-Object -Unique | Where-Object { $_ -match '\.(c|cc|cpp|cxx|h|hh|hpp|hxx|ino)$' }
if ($changedSourceFiles | Where-Object { $_.Contains("'") }) {
  throw 'Changed source paths containing single quotes are not supported.'
}
$sourceFileArguments = ($changedSourceFiles | ForEach-Object { "'$_'" }) -join ' '

$formatScript = @'
set -euo pipefail
repo='__REPO_ROOT__'
tool_root="$HOME/.cache/cats-gs-precommit"
venv="$tool_root/venv"

cd "$repo"

if [[ ! -x "$venv/bin/platformio" || ! -x "$venv/bin/clang-format" ]]; then
  echo 'WSL pre-commit tools are missing. Follow the one-time setup in ground_station/AGENTS.md.' >&2
  exit 1
fi

platformio_version="$($venv/bin/platformio --version)"
format_version="$($venv/bin/clang-format --version)"
if [[ "$platformio_version" != *'6.1.19'* ]]; then
  echo "PlatformIO 6.1.19 is required in WSL. Found: $platformio_version" >&2
  exit 1
fi
if [[ "$format_version" != *'version 17.'* ]]; then
  echo "clang-format 17 is required in WSL. Found: $format_version" >&2
  exit 1
fi

source_files=(__SOURCE_FILES__)

if (( ${#source_files[@]} > 0 )); then
  "$venv/bin/clang-format" -i -- "${source_files[@]}"
  "$venv/bin/clang-format" --dry-run --Werror -- "${source_files[@]}"
fi
'@.Replace('__REPO_ROOT__', $escapedWslRepoRoot).Replace('__SOURCE_FILES__', $sourceFileArguments)

Write-Host 'Formatting changed Ground Station source files with clang-format 17...'
Invoke-WslScript -Script $formatScript

Write-Host 'Checking the diff for whitespace errors...'
Invoke-CommandChecked -FilePath 'git' -Arguments @('-C', $RepoRoot, 'diff', '--check')

Write-Host 'Building the Ground Station firmware on Windows...'
Invoke-CommandChecked -FilePath $platformIo -Arguments @('run', '-d', $GroundStationRoot)

Write-Host 'Generating the Windows compilation database...'
Invoke-CommandChecked -FilePath $platformIo -Arguments @('run', '-d', $GroundStationRoot, '--target', 'compiledb')

$lintArguments = @('--verify-sync', $wslRepoRoot)
$lintScript = Get-Content -LiteralPath (Join-Path $GroundStationRoot 'gs-wsl-check.sh') -Raw

Write-Host 'Running the Linux Ground Station build and clang-tidy check in WSL...'
Invoke-WslScript -Script $lintScript -Arguments $lintArguments

Push-Location $GroundStationRoot
try {
  Write-Host 'Running the deterministic simulator suite...'
  Invoke-CommandChecked -FilePath (Join-Path $GroundStationRoot 'gs-sim.ps1') -Arguments @('test')

  Write-Host 'Building the WebAssembly simulator...'
  Invoke-CommandChecked -FilePath (Join-Path $GroundStationRoot 'gs-sim.ps1') -Arguments @('build')

  $changedFiles = @(
    & git -C $RepoRoot diff --name-only HEAD -- ground_station
    & git -C $RepoRoot ls-files --others --exclude-standard -- ground_station
  )
  $fatFsChanged = $changedFiles | Where-Object {
    $_ -like 'ground_station/lib/FatFs/*' -or
    $_ -like 'ground_station/tests/fatfs_*' -or
    $_ -like 'ground_station/tests/fixtures/fatfs-*'
  }
  if ($fatFsChanged.Count -gt 0) {
    Write-Host 'Running the FatFs compatibility test...'
    Invoke-CommandChecked -FilePath (Join-Path $GroundStationRoot 'gs-sim.ps1') -Arguments @('fatfs-test')
  }
} finally {
  Pop-Location
}

Write-Host 'Ground Station automated pre-commit checks passed.'
