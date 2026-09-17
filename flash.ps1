# Build, then flash the same image to every port given. Ports on the command line override
# upload_port in platformio.ini, so the ini is never touched and VS Code has nothing to reload.
#
#   .\flash.ps1 COM32 COM24             devkitc build to both
#   .\flash.ps1 -e devkitm COM24        other environment
#   pio device list                     what is plugged in where

[CmdletBinding(PositionalBinding = $false)]
param(
    [Alias('e')][string]$Env = 'devkitc',
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Ports
)

if (-not $Ports) {
    Write-Host 'usage: .\flash.ps1 [-e devkitc|devkitm] COMx [COMy ...]'
    exit 1
}

# Windows does not care, esptool does: it finds the board's USB PID by comparing the port name
# exactly against pyserial's list, which says "COM29". "com29" matches nothing, so esptool
# cannot tell the port is the C6's USB-Serial/JTAG, falls back to the DTR/RTS reset meant for a
# UART bridge, and dies with a pySerial write timeout.
$Ports = $Ports | ForEach-Object { $_.ToUpper() }

$pio = (Get-Command pio -ErrorAction SilentlyContinue).Source
if (-not $pio) { $pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" }

& $pio run -e $Env
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$failed = @()
foreach ($p in $Ports) {
    Write-Host "`n==== $p ====" -ForegroundColor Cyan
    # Not -t nobuild: that skips the build script which tells esptool where the bootloader and
    # partition table go, and the upload fails on bare firmware.bin. The image is already built,
    # so the build step here is a few-second no-op.
    & $pio run -e $Env -t upload --upload-port $p
    if ($LASTEXITCODE -ne 0) { $failed += $p }
}

if ($failed) {
    Write-Host "`nFAILED: $($failed -join ', ')" -ForegroundColor Red
    exit 1
}
Write-Host "`nflashed: $($Ports -join ', ')" -ForegroundColor Green
