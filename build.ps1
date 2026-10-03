param(
    [ValidateSet('build', 'flash', 'monitor', 'menuconfig', 'size', 'clean', 'reconfigure')]
    [string]$Action = 'build',
    [string]$Port = 'COM29',
    [string]$IdfPath = $env:IDF_PATH
)
$ErrorActionPreference = 'Stop'
if (-not $IdfPath) {
    $settingsFile = Join-Path $PSScriptRoot '.vscode/settings.json'
    if (Test-Path -LiteralPath $settingsFile) {
        $settings = Get-Content -LiteralPath $settingsFile -Raw | ConvertFrom-Json
        $IdfPath = $settings.'idf.currentSetup'
    }
}
if (-not $IdfPath) { $IdfPath = 'C:\Espressif\frameworks\esp-idf-v5.3.3' }
$exportScript = Join-Path $IdfPath 'export.ps1'
if (-not (Test-Path -LiteralPath $exportScript)) {
    throw 'ESP-IDF was not found. Set IDF_PATH or pass -IdfPath with your ESP-IDF 5.3.x directory.'
}
. $exportScript
$idfScript = Join-Path $IdfPath 'tools/idf.py'
$arguments = @('-C', $PSScriptRoot, '-B', (Join-Path $PSScriptRoot 'build/idf'))
if ($Action -eq 'flash' -or $Action -eq 'monitor') { $arguments += @('-p', $Port) }
$arguments += $Action
# Windows PowerShell 5 represents native stderr as ErrorRecords. Let IDF's
# exit code decide success so compiler warnings cannot terminate the wrapper.
$previousPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
    & python $idfScript @arguments
    $buildExitCode = $LASTEXITCODE
} finally {
    $ErrorActionPreference = $previousPreference
}
if ($buildExitCode -ne 0) { exit $buildExitCode }
if ($Action -eq 'build' -or $Action -eq 'flash') {
    $otaFile = Get-Item -LiteralPath (Join-Path $PSScriptRoot 'build/idf/lpovxlm.bin')
    $identity = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'build/idf/generated/build_info.json') -Raw | ConvertFrom-Json
    Write-Host ("Firmware: Ver{0} | Build {1}" -f $identity.version, $identity.buildNumber)
    Write-Host ("Built: {0} (local time)" -f $identity.buildTime)
    Write-Host ("OTA file: {0}" -f $otaFile.FullName)
    Write-Host ("File modified: {0:yyyy-MM-dd HH:mm:ss zzz}" -f $otaFile.LastWriteTime)
}
