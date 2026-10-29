param([string]$Distro = 'Ubuntu-24.04')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
& $wslExe -d $Distro -- bash $linuxRoot/scripts/simulation/stop-simulation.sh
exit $LASTEXITCODE
