param([string]$Distro = 'Ubuntu-24.04')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
$check = "$linuxRoot/scripts/display/check-gfxredir.sh"
# A stopped distro may need time to establish its X11 socket and Weston log.
$state = 'starting'
for ($attempt = 0; $attempt -lt 20; $attempt++) {
    $state = (& $wslExe -d $Distro -- bash $check | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw 'Cannot query WSLg. Check WSL access and the distribution name.' }
    if ($state -in @('0', '1')) { break }
    Start-Sleep -Milliseconds 500
}
if ($state -eq '1') {
    Write-Output 'WSLg graphics redirection is enabled. No restart was needed.'
    exit 0
}
if ($state -ne '0') { throw "WSLg did not become ready (state: $state). Inspect /mnt/wslg/stderr.log; no global WSL restart was attempted." }
Write-Output 'Restarting the WSLg compositor. Native WSLg windows will close; Ubuntu and VcXsrv remain running.'
& $wslExe -d $Distro --system -- pkill -TERM -x weston
if ($LASTEXITCODE -ne 0) { throw 'Could not restart Weston.' }
for ($attempt = 0; $attempt -lt 15; $attempt++) {
    Start-Sleep -Milliseconds 800
    $state = (& $wslExe -d $Distro -- bash $check | Out-String).Trim()
    if ($state -eq '1') {
        Write-Output 'WSLg graphics redirection recovered. Reopen native GUI applications.'
        exit 0
    }
}
throw 'WSLg has not recovered. Inspect /mnt/wslg/weston.log.'
