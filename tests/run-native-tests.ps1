$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Visual Studio C++ build tools are required for host tests.' }
$installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'Visual Studio C++ toolchain was not found.' }
$vcvars = Join-Path $installation 'VC/Auxiliary/Build/vcvars64.bat'
Push-Location $repo
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    # Native command quoting is confined to known tool/workspace-relative paths.
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I main /I . tests\native_format_test.cpp /Fe:build\native-format-test.exe /Fo:build\native-format-test.obj && build\native-format-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "Native tests failed with exit code $LASTEXITCODE" }
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I tests\stubs /I main /FI sd_read_hook.hpp tests\fseq_reader_test.cpp main\fseq.cpp /Fe:build\fseq-reader-test.exe /Fo:build\ && build\fseq-reader-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "FSEQ reader tests failed with exit code $LASTEXITCODE" }
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I tests\stubs /I main tests\playback_sweep_test.cpp main\fseq.cpp /Fe:build\playback-sweep-test.exe /Fo:build\ && build\playback-sweep-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "Playback sweep tests failed with exit code $LASTEXITCODE" }
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I main tests\sd_benchmark_test.cpp /Fe:build\sd-benchmark-test.exe /Fo:build\sd-benchmark-test.obj && build\sd-benchmark-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "SD benchmark tests failed with exit code $LASTEXITCODE" }
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I main tests\sd_read_test.cpp /Fe:build\sd-read-test.exe /Fo:build\sd-read-test.obj && build\sd-read-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "SD read tests failed with exit code $LASTEXITCODE" }
    & cmd /c "call `"$vcvars`" > nul && cl /nologo /std:c++17 /EHsc /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I main tests\sd_recovery_test.cpp /Fe:build\sd-recovery-test.exe /Fo:build\sd-recovery-test.obj && build\sd-recovery-test.exe"
    if ($LASTEXITCODE -ne 0) { throw "SD recovery tests failed with exit code $LASTEXITCODE" }
} finally { Pop-Location }
