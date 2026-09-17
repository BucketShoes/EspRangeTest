# Build once, then flash the same image to every port given. Ports on the command line override
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

$pio = (Get-Command pio -ErrorAction SilentlyContinue).Source
if (-not $pio) { $pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" }

& $pio run -e $Env
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$failed = @()
foreach ($p in $Ports) {
    Write-Host "`n==== $p ====" -ForegroundColor Cyan
    # nobuild: the image above is already current; this only uploads.
    & $pio run -e $Env -t nobuild -t upload --upload-port $p
    if ($LASTEXITCODE -ne 0) { $failed += $p }
}

if ($failed) {
    Write-Host "`nFAILED: $($failed -join ', ')" -ForegroundColor Red
    exit 1
}
Write-Host "`nflashed: $($Ports -join ', ')" -ForegroundColor Green
