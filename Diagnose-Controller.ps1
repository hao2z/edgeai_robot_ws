param(
    [ValidateRange(1,120)][int]$Duration=20,
    [ValidateRange(1,20)][int]$Hz=10,
    [string]$Distro='Ubuntu-24.04'
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$windowsRoot=$PSScriptRoot.Replace('\','/')
$resolvedRoot=& $wslExe -d $Distro -- wslpath -a $windowsRoot
if ($LASTEXITCODE -ne 0 -or -not $resolvedRoot) {throw 'Cannot resolve workspace path in WSL'}
$linuxRoot=$resolvedRoot.Trim()
& $wslExe -d $Distro --cd $linuxRoot -- bash scripts/ros/with-ros-env.sh `
    python3 scripts/ros/controller_probe.py --duration $Duration --hz $Hz
exit $LASTEXITCODE
