param([string]$Distro = 'Ubuntu-24.04')
$ErrorActionPreference = 'Stop'
$linuxRoot = (wsl -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
wsl -d $Distro -- bash $linuxRoot/scripts/simulation/stop-simulation.sh
exit $LASTEXITCODE
