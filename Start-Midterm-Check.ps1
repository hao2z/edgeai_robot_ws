param(
    [ValidateSet('all','part2')][string]$Part = 'all',
    [ValidateRange(30,3600)][int]$Duration = 780,
    [ValidateRange(0,120)][int]$VerificationSeconds = 120,
    [ValidateRange(0,232)][int]$DomainId = 86,
    [ValidateSet('rpp','dwb','mppi')][string]$Controller = 'rpp',
    [ValidateSet('lattice','navfn')][string]$Planner = 'lattice',
    [ValidateSet('config','true','false')][string]$DepthObstacles = 'config',
    [string]$Distro = 'Ubuntu-24.04',
    [ValidateSet('native','pixi')][string]$Backend,
    [switch]$CheckOnly,
    [switch]$InCurrentWindow
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
if (-not $Backend) {
    $backendFile = Join-Path $PSScriptRoot 'runtime/ros-backend'
    if ($env:EDGEAI_ROS_BACKEND) { $Backend = $env:EDGEAI_ROS_BACKEND }
    elseif (Test-Path -LiteralPath $backendFile) { $Backend = (Get-Content -LiteralPath $backendFile -Raw).Trim() }
    else { $Backend = 'native' }
}
if ($Backend -notin @('native','pixi')) { throw "Invalid ROS backend: $Backend" }
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
$options = @('--part', $Part, '--duration', $Duration, '--verification-seconds', $VerificationSeconds,
    '--domain', $DomainId, '--controller', $Controller, '--planner', $Planner,
    '--depth-obstacles', $DepthObstacles)
& $wslExe -d $Distro -- env "EDGEAI_ROS_BACKEND=$Backend" bash "$linuxRoot/scripts/simulation/start-midterm-check.sh" @options --check
if ($LASTEXITCODE -ne 0) { throw "ROS backend '$Backend' failed its runtime check; no previous session was stopped." }
if ($CheckOnly) { return }
if (-not $InCurrentWindow) {
    & (Join-Path $PSScriptRoot 'Stop-Simulation.ps1') -KeepProcessId $PID -Distro $Distro
    $consoleArgs = @('-NoLogo','-NoProfile','-NoExit','-ExecutionPolicy','Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow','-Part',$Part,'-Duration',$Duration,
        '-VerificationSeconds',$VerificationSeconds,'-DomainId',$DomainId,
        '-Controller',$Controller,'-Planner',$Planner,'-Distro',"`"$Distro`"",'-Backend',$Backend,
        '-DepthObstacles',$DepthObstacles)
    if ($windowsTerminal) {
        Start-Process $windowsTerminal -ArgumentList (@('-w','new','new-tab','--title','"EdgeAI Midterm Check"',
            '--suppressApplicationTitle','-d',"`"$PSScriptRoot`"", "`"$windowsPowerShell`"") + $consoleArgs) `
            -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process $windowsPowerShell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host '中期检查已在独立终端启动。'
    return
}
$Host.UI.RawUI.WindowTitle = 'EdgeAI Midterm Check'
Write-Host "Midterm parts=$Part : $Controller / $Planner. Backend=$Backend. ROS_DOMAIN_ID=$DomainId. Budget=${Duration}s."
Write-Host 'Ctrl+C stops this session. Results: runtime/latest-midterm-session.txt.'
$runName = "$(Get-Date -Format 'yyyyMMdd-HHmmss-fff')-midterm-domain$DomainId"
$sessionDirectory = Join-Path $PSScriptRoot "tests/results/runs/$runName"
[void][IO.Directory]::CreateDirectory($sessionDirectory)
$observer = Start-Process $windowsPowerShell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass',
    '-File',"`"$(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')`"",'-Midterm',
    '-SessionDirectory',"`"$sessionDirectory`"")
try {
    & $wslExe -d $Distro -- env "EDGEAI_ROS_BACKEND=$Backend" FORCE_COLOR=1 EDGEAI_CAPTURE_ACK=true `
        "EDGEAI_MIDTERM_SESSION_DIR=$linuxRoot/tests/results/runs/$runName" bash "$linuxRoot/scripts/simulation/start-midterm-check.sh" @options
    $result = $LASTEXITCODE
} finally {
    [IO.File]::WriteAllText((Join-Path $sessionDirectory 'exit-code.txt'), "$LASTEXITCODE")
    if (-not $observer.WaitForExit(15000)) { $observer.Kill() }
}
Write-Host "Midterm process ended with code $result. The window stays open for inspection."
if ($result -ne 0) { throw "Midterm check exited with code $result; inspect its session logs." }
