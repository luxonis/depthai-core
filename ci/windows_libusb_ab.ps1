$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$options = Get-Content "$PSScriptRoot/options.json" -Raw | ConvertFrom-Json
$repetitions = 0
if (-not [int]::TryParse([string]$options.repetitions, [ref]$repetitions) -or $repetitions -lt 1 -or $repetitions -gt 100) {
    throw 'repetitions must be an integer between 1 and 100'
}

$env:DEPTHAI_PROTOCOL = 'usb'
$env:DEPTHAI_DEVICE_MXID_LIST = [string]$options.device_mxid
$env:DEPTHAI_LEVEL = 'debug'
$env:LIBUSB_DEBUG = '4'
$logRoot = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
Copy-Item "$PSScriptRoot/commit.txt", "$PSScriptRoot/options.json" $logRoot
$summary = Join-Path $logRoot 'summary.md'
"| Variant | CTest exit code |" | Set-Content $summary
"| --- | --- |" | Add-Content $summary

$results = @{}
foreach ($variant in @('unpatched', 'patched')) {
    $testDir = Join-Path $PSScriptRoot $variant
    $variantLogs = Join-Path $logRoot $variant
    New-Item -ItemType Directory -Force -Path $variantLogs | Out-Null
    if (-not (Test-Path "$testDir/regression_boot_test.exe") -or -not (Get-ChildItem "$testDir/*usb*.dll")) {
        throw "The $variant test bundle is incomplete"
    }
    Get-ChildItem "$testDir/*.dll", "$testDir/*.exe" | Get-FileHash -Algorithm SHA256 | Format-List | Out-File "$variantLogs/binary-hashes.txt"
    # A failing baseline must still be followed by a run of the patched build.
    & "$PSScriptRoot/ctest.exe" --test-dir $testDir -C Release -R '^regression_boot_test$' --repeat "until-fail:$repetitions" --output-on-failure --no-tests=error --output-log "$variantLogs/ctest.log" --output-junit "$variantLogs/results.xml"
    $results[$variant] = $LASTEXITCODE
    Copy-Item "$testDir/Testing/Temporary/LastTest*.log" $variantLogs -ErrorAction SilentlyContinue
    "| $variant | $($results[$variant]) |" | Add-Content $summary
    # Let the watchdog recover before switching libraries after a failed boot.
    if ($variant -eq 'unpatched') { Start-Sleep -Seconds 15 }
}
$results | ConvertTo-Json | Set-Content "$logRoot/exit-codes.json"

if ($results['patched'] -ne 0) {
    'The patched regression test failed.' | Add-Content $summary
    throw 'The patched regression test failed; inspect the comparison logs'
}
if ($results['unpatched'] -eq 0) {
    'Both variants passed. The baseline did not reproduce the issue; the comparison is inconclusive.' | Add-Content $summary
} else {
    'Unpatched failed and patched passed. Check the baseline log for the expected endpoint/interface failure.' | Add-Content $summary
}
Get-Content $summary
