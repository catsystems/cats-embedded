[CmdletBinding()]
param(
  [Parameter(Position = 0, Mandatory = $true)]
  [ValidateSet('setup', 'build', 'fatfs-test', 'serve', 'run', 'test')]
  [string]$Command,
  [Parameter(Position = 1)]
  [string]$Scenario,
  [int]$Port = 8787
)

$entrypoint = Join-Path $PSScriptRoot 'simulator\gs-sim.ps1'
& $entrypoint -Command $Command -Scenario $Scenario -Port $Port
exit $LASTEXITCODE
