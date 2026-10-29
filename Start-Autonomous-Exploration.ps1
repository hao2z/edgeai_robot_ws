param(
    [ValidateRange(30, 3600)][int]$Duration = 300,
    [string]$Distro = 'Ubuntu-24.04',
    [ValidateSet('simple', 'competition')][string]$Arena = 'competition',
    [ValidateRange(0,232)][int]$DomainId = 86,
    [ValidateSet('mppi', 'dwb')][string]$Controller = 'dwb',
    [ValidateSet('native', 'pixi')][string]$Backend,
    [switch]$CheckOnly,
    [switch]$InCurrentWindow,
    [switch]$TurnTest,
    [switch]$Scan360,
    [ValidateSet('forward','reverse','turn','axes')][string]$MotionTest,
    [ValidateSet('map','rotated')][string]$MotionFrame = 'map',
    [switch]$MotionGoalOnly,
    [switch]$MPPIInvertLinear,
    [switch]$MPPISource,
    [switch]$RecoveryTest
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
if (-not $Backend) {
    $backendFile = Join-Path $PSScriptRoot 'runtime/ros-backend'
    if ($env:EDGEAI_ROS_BACKEND) { $Backend = $env:EDGEAI_ROS_BACKEND }
    elseif (Test-Path -LiteralPath $backendFile) { $Backend = (Get-Content -LiteralPath $backendFile -Raw).Trim() }
    else { $Backend = 'native' }
}
if ($Backend -notin @('native', 'pixi')) { throw "Invalid ROS backend: $Backend" }
if ($MotionTest) { $TurnTest = $true }
if ($Scan360 -and ($TurnTest -or $RecoveryTest)) { throw 'Scan360 cannot be combined with motion/recovery tests.' }
if ($MotionGoalOnly -and (-not $MotionTest -or $Controller -ne 'mppi')) { throw 'MotionGoalOnly requires an MPPI MotionTest.' }
if ($MPPIInvertLinear -and (-not $MotionTest -or $Controller -ne 'mppi')) { throw 'MPPIInvertLinear requires an MPPI MotionTest.' }
if ($MPPISource -and $Controller -ne 'mppi') { throw 'MPPISource requires Controller mppi.' }
if ($TurnTest -and $RecoveryTest) { throw 'Choose either TurnTest or RecoveryTest.' }
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
& $wslExe -d $Distro -- env "EDGEAI_ROS_BACKEND=$Backend" bash "$linuxRoot/scripts/simulation/start-autonomous-exploration.sh" --check --duration $Duration --controller $Controller --arena $Arena --domain $DomainId
if ($LASTEXITCODE -ne 0) { throw "ROS backend '$Backend' failed its runtime check. Build this backend with scripts/setup/prepare-open-source.sh before starting a test." }
if ($CheckOnly) { return }
if (-not $InCurrentWindow) {
    & (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow', '-Duration', $Duration, '-Distro', "`"$Distro`"", '-Arena', $Arena, '-DomainId', $DomainId)
    $consoleArgs += @('-Controller', $Controller, '-Backend', $Backend)
    if ($TurnTest) { $consoleArgs += '-TurnTest' }
    if ($Scan360) { $consoleArgs += '-Scan360' }
    if ($MotionTest) { $consoleArgs += @('-MotionTest', $MotionTest) }
    $consoleArgs += @('-MotionFrame', $MotionFrame)
    if ($MotionGoalOnly) { $consoleArgs += '-MotionGoalOnly' }
    if ($MPPIInvertLinear) { $consoleArgs += '-MPPIInvertLinear' }
    if ($MPPISource) { $consoleArgs += '-MPPISource' }
    if ($RecoveryTest) { $consoleArgs += '-RecoveryTest' }
    if ($windowsTerminal) {
        Start-Process $windowsTerminal -ArgumentList (@('-w', 'new', 'new-tab', '--title', '"EdgeAI Autonomous Exploration"',
            '--suppressApplicationTitle', '-d', "`"$PSScriptRoot`"", "`"$windowsPowerShell`"") + $consoleArgs) -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process $windowsPowerShell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host 'Autonomous exploration window opened.'
    return
}
$Host.UI.RawUI.WindowTitle = 'EdgeAI Autonomous Exploration'
Write-Host "Single robot: SLAM Toolbox + Nav2 $Controller + frontier_exploration_ros2. Backend=$Backend. ROS_DOMAIN_ID=$DomainId." -ForegroundColor Cyan
Write-Host 'The robot moves autonomously in Gazebo. Ctrl+C stops this session.'
# Also cover callers invoking the worker directly.
& (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro -ProcessesOnly
$tag = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$runName = "$tag-domain$DomainId"
if ($TurnTest) { $runName += '-turn' }
if ($RecoveryTest) { $runName += '-recovery' }
$sessionDirectory = Join-Path $PSScriptRoot "tests/results/runs/$runName"
[void](New-Item -ItemType Directory -Path $sessionDirectory -Force)
[void](New-Item -ItemType File -Path (Join-Path $sessionDirectory 'events.log') -Force)
$linuxSession = "$linuxRoot/tests/results/runs/$runName"
$mapPath = "$linuxSession/map"
Write-Host "Session logs: $sessionDirectory" -ForegroundColor Cyan
# Open a visible timestamped event monitor even when the worker was started by an agent.
& (Join-Path $PSScriptRoot 'Open-ROS-Monitor.ps1') -Distro $Distro -DomainId $DomainId -SessionDirectory $sessionDirectory
$observer = Start-Process $windowsPowerShell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', "`"$(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')`"", '-SessionDirectory', "`"$sessionDirectory`"")
$scanMode = if ($Scan360) { 'true' } else { 'false' }
$turnMode = if ($TurnTest) { 'true' } else { 'false' }
$recoveryMode = if ($RecoveryTest) { 'true' } else { 'false' }
$motionAblation = if ($MotionGoalOnly) { 'goal_only' } else { '' }
$invertLinear = if ($MPPIInvertLinear) { 'true' } else { 'false' }
$mppiSourceFlag = if ($MPPISource) { 'true' } else { 'false' }
# Use the same managed Linux entry as a direct WSL invocation. PowerShell owns
# only Windows windows/capture; Linux owns test execution and history retention.
Write-Host 'Starting the managed WSL test entry...' -ForegroundColor Cyan
& $wslExe -d $Distro -- env "EDGEAI_ROS_BACKEND=$Backend" EDGEAI_MANAGED_SESSION=true "EDGEAI_MPPI_USE_SOURCE=$mppiSourceFlag" "EDGEAI_MPPI_INVERT_LINEAR=$invertLinear" "EDGEAI_MOTION_FRAME=$MotionFrame" "EDGEAI_MOTION_ABLATION=$motionAblation" "EDGEAI_MOTION_TEST=$MotionTest" "EDGEAI_CONTROLLER=$Controller" "EDGEAI_SESSION_DIR=$linuxSession" "EDGEAI_SCAN360=$scanMode" "EDGEAI_TURN_TEST=$turnMode" "EDGEAI_RECOVERY_TEST=$recoveryMode" FORCE_COLOR=1 EDGEAI_CAPTURE_ACK=true bash $linuxRoot/scripts/simulation/start-autonomous-exploration.sh $Duration $mapPath $Arena $DomainId
$result = $LASTEXITCODE
if (-not $observer.WaitForExit(15000)) { $observer.Kill() }
Write-Host "Exploration process ended with code $result. The window stays open for inspection."
if ($result -ne 0) { throw "Exploration failed with code $result." }
