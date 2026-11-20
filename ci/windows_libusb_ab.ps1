$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$options = Get-Content "$PSScriptRoot/options.json" -Raw | ConvertFrom-Json
$mode = if ($options.mode) { [string]$options.mode } else { 'libusb-ab' }
if ($mode -notin @('suite', 'libusb-ab')) { throw 'Unknown test mode' }
$repetitions = 0
if (-not [int]::TryParse([string]$options.repetitions, [ref]$repetitions) -or $repetitions -lt 1 -or $repetitions -gt 100) {
    throw 'repetitions must be an integer between 1 and 100'
}
if ($mode -eq 'suite' -and ($options.stress_seconds -lt 60 -or $options.stress_seconds -gt 86400)) {
    throw 'Streaming duration must be between 60 and 86400 seconds'
}

$env:DEPTHAI_PROTOCOL = 'usb'
$env:DEPTHAI_DEVICE_MXID_LIST = [string]$options.device_mxid
# Keep logging overhead out of the streaming performance comparison.
$env:DEPTHAI_LEVEL = if ($mode -eq 'suite') { 'warn' } else { 'debug' }
$env:LIBUSB_DEBUG = if ($mode -eq 'suite') { '0' } else { '4' }
$env:UBSAN_OPTIONS = 'halt_on_error=1'
$logRoot = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
Copy-Item "$PSScriptRoot/commit.txt", "$PSScriptRoot/options.json" $logRoot
$summary = Join-Path $logRoot 'summary.md'
"| Variant | Phase | CTest exit code |" | Set-Content $summary
"| --- | --- | --- |" | Add-Content $summary

$results = @{}
$phases = if ($mode -eq 'suite') { @('functional', 'stress') } else { @('boot') }
foreach ($variant in @('unpatched', 'patched')) {
    $testDir = Join-Path $PSScriptRoot $variant
    $variantLogs = Join-Path $logRoot $variant
    New-Item -ItemType Directory -Force -Path $variantLogs | Out-Null
    if (-not (Test-Path "$testDir/regression_boot_test.exe") -or -not (Get-ChildItem "$testDir/*usb*.dll")) {
        throw "The $variant test bundle is incomplete"
    }
    Get-ChildItem "$testDir/*.dll", "$testDir/*.exe" | Get-FileHash -Algorithm SHA256 | Format-List | Out-File "$variantLogs/binary-hashes.txt"
    $results[$variant] = @{}
    foreach ($phase in $phases) {
        $testArgs = @('--test-dir', $testDir, '-C', 'Release', '--output-on-failure', '--no-tests=error',
            '--output-log', "$variantLogs/$phase.log", '--output-junit', "$variantLogs/$phase.xml")
        if ($phase -eq 'functional') { $testArgs += @('-L', '^usb$') }
        if ($phase -eq 'stress') { $testArgs += @('-R', '^stability_stress_test$', '--verbose') }
        if ($phase -eq 'boot') { $testArgs += @('-R', '^regression_boot_test$', '--repeat', "until-fail:$repetitions") }
        # Continue through both variants so a baseline failure does not hide patched results.
        & "$PSScriptRoot/ctest.exe" @testArgs
        $results[$variant][$phase] = $LASTEXITCODE
        Copy-Item "$testDir/Testing/Temporary/LastTest.log" "$variantLogs/$phase-LastTest.log" -ErrorAction SilentlyContinue
        "| $variant | $phase | $($results[$variant][$phase]) |" | Add-Content $summary
    }
    # Let the watchdog recover before switching libraries after a failed boot.
    if ($variant -eq 'unpatched') { Start-Sleep -Seconds 15 }
}
$results | ConvertTo-Json | Set-Content "$logRoot/exit-codes.json"

if (@($results['patched'].Values | Where-Object { $_ -ne 0 }).Count -gt 0) {
    throw 'A patched test phase failed; inspect the comparison logs'
}
Get-Content $summary
