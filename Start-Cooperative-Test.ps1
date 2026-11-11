param([int]$Duration = 600, [int]$DomainId = 86, [string]$Distro = 'Ubuntu-24.04', [string]$ParamsFile = '', [switch]$FilterCheck)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
function Linux-Path([string]$Path) {
    $value = (& $wslExe -d $Distro -- wslpath -a -u ($Path.Replace('\','/')) | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw "Cannot resolve path: $Path" }
    return $value
}
$linuxRoot = Linux-Path $PSScriptRoot
if (-not $ParamsFile) {
    $latest = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'runtime/latest-session.txt')).Trim()
    $previous = (& $wslExe -d $Distro -- wslpath -w $latest | Out-String).Trim()
    $ParamsFile = Join-Path $previous 'nav2-runtime.yaml'
}
if (-not (Test-Path -LiteralPath $ParamsFile)) { throw "Missing runtime config: $ParamsFile" }
& (Join-Path $PSScriptRoot 'Stop-Simulation.ps1') -KeepProcessId $PID -Distro $Distro
$directory = Join-Path $PSScriptRoot ('tests/results/runs/' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-cooperative-truth-test')
[void][IO.Directory]::CreateDirectory($directory)
Copy-Item -LiteralPath $ParamsFile -Destination (Join-Path $directory 'params-source.yaml')
$observer = Start-Process -FilePath $windowsPowerShell -WindowStyle Hidden -PassThru -ArgumentList @(
    '-NoProfile','-ExecutionPolicy','Bypass','-File',('"'+(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')+'"'),
    '-SessionDirectory',('"'+$directory+'"'))
$env:EDGEAI_ROS_BACKEND = 'native'
$linuxDirectory = Linux-Path $directory
$testArgs = @('--directory',$linuxDirectory,'--params',"$linuxDirectory/params-source.yaml",'--duration',$Duration,'--domain',$DomainId)
if ($FilterCheck) { $testArgs += '--filter-check' }
& $wslExe -d $Distro -- env EDGEAI_CAPTURE_ACK=true bash "$linuxRoot/scripts/ros/with-ros-env.sh" bash "$linuxRoot/scripts/simulation/run-cooperative-truth-test.sh" @testArgs
$code = $LASTEXITCODE
Write-Host "Cooperative test result: $directory (exit=$code)"
if ($code -ne 0) { throw "Cooperative test failed; inspect $directory/cooperative.log" }
