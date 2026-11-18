param(
    # Operator display/log language: zh = 中文, en = English.
    # debug = 完整诊断和界面；performance = 精简诊断、关闭 Gazebo 界面、RViz 15 FPS。
    [ValidateSet('debug','performance')][string]$RuntimeProfile = 'performance',
    [ValidateSet('zh','en')][string]$Language = 'en',
    # $true = 双机联合建图；$false = 单机建图后双机救援。
    [switch]$JointMapping = $true,
    [switch]$DisableJointMapping,  # 显式关闭；兼容 Windows PowerShell 独立终端传参。
    [ValidateRange(30, 3600)][int]$Duration = 3600,
    [string]$Distro = 'Ubuntu-24.04',
    [ValidateSet('simple', 'competition', 'midterm')][string]$Arena = 'competition',
    # Leave empty to use the selected arena's manifest. Count 0 means infer from the list.
    [ValidatePattern('^(?:[0-9]+:(?:normal|cooperative)(?:,[0-9]+:(?:normal|cooperative))*)$')][string]$Targets = '',
    [ValidateRange(0, 50)][int]$TargetCount = 0,
    [ValidateRange(0,232)][int]$DomainId = 86,
    [ValidateSet('mppi', 'dwb', 'rpp')][string]$Controller = 'rpp',
    [ValidateSet('lattice','navfn')][string]$Planner = 'lattice',
    [ValidateSet('config','true','false')][string]$DepthObstacles = 'config',
    [ValidateSet('native', 'pixi')][string]$Backend,
    [switch]$CheckOnly,
    [switch]$InCurrentWindow,
    [switch]$NavigationTest,
    [switch]$CooperativeRescue,
    [switch]$DisableCooperativeRescue,
    [switch]$SingleRobot,
    [switch]$TurnTest,
    [switch]$Scan360,
    [ValidateSet('forward','reverse','turn','axes','points')][string]$MotionTest,
    [ValidateSet('simple_points','u_wall','narrow_u','narrow_spaces')][string]$PointScenario = 'simple_points',
    [ValidateSet('map','rotated')][string]$MotionFrame = 'map',
    [switch]$MotionGoalOnly,
    [switch]$MPPIInvertLinear,
    [switch]$MPPISource,
    [switch]$RecoveryTest
)
$ErrorActionPreference = 'Stop'
$env:EDGEAI_LANGUAGE = $Language
$env:EDGEAI_RUNTIME_PROFILE = $RuntimeProfile
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
if (-not $Backend) {
    $backendFile = Join-Path $PSScriptRoot 'runtime/ros-backend'
    if ($env:EDGEAI_ROS_BACKEND) { $Backend = $env:EDGEAI_ROS_BACKEND }
    elseif (Test-Path -LiteralPath $backendFile) { $Backend = (Get-Content -LiteralPath $backendFile -Raw).Trim() }
    else { $Backend = 'native' }
}
if ($Backend -notin @('native', 'pixi')) { throw "Invalid ROS backend: $Backend" }
if ($MotionTest) { $TurnTest = $true }
# Default mission selection; explicit caller choices remain available.
if (-not $PSBoundParameters.ContainsKey('CooperativeRescue')) {
    $CooperativeRescue = ($Arena -in @('midterm', 'competition') -and -not ($NavigationTest -or $TurnTest -or $RecoveryTest -or $Scan360))
}
if ($SingleRobot -or $DisableCooperativeRescue) { $CooperativeRescue = $false }
# Specialized test entries keep their own mode when the editable default is enabled.
if ($DisableJointMapping -or (-not $PSBoundParameters.ContainsKey('JointMapping') -and -not $CooperativeRescue)) { $JointMapping = $false }
$env:EDGEAI_JOINT_MAPPING = ([bool]$JointMapping).ToString().ToLowerInvariant()
$cooperativeRescueFlag = ([bool]$CooperativeRescue).ToString().ToLowerInvariant()
if ($JointMapping -and ($SingleRobot -or ($Arena -eq 'midterm' -and -not $CooperativeRescue) -or $NavigationTest -or $TurnTest -or $RecoveryTest -or $Scan360)) { throw 'JointMapping requires an integrated two-robot mission.' }
# End default mission selection.
if ($CooperativeRescue -and ($Arena -notin @('midterm', 'competition') -or $NavigationTest -or $TurnTest -or $RecoveryTest -or $Scan360)) { throw 'CooperativeRescue requires a normal midterm or competition mission.' }
if ($NavigationTest -and ($Scan360 -or $RecoveryTest)) { throw 'NavigationTest cannot combine Scan360 or recovery fault injection.' }
if ($Scan360 -and ($TurnTest -or $RecoveryTest)) { throw 'Scan360 cannot be combined with motion/recovery tests.' }
if ($MotionGoalOnly -and (-not $MotionTest -or $Controller -ne 'mppi')) { throw 'MotionGoalOnly requires an MPPI MotionTest.' }
if ($MPPIInvertLinear -and (-not $MotionTest -or $Controller -ne 'mppi')) { throw 'MPPIInvertLinear requires an MPPI MotionTest.' }
if ($MPPISource -and $Controller -ne 'mppi') { throw 'MPPISource requires Controller mppi.' }
if ($TurnTest -and $RecoveryTest) { throw 'Choose either TurnTest or RecoveryTest.' }
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
& $wslExe -d $Distro -- env "EDGEAI_RUNTIME_PROFILE=$RuntimeProfile" "EDGEAI_JOINT_MAPPING=$($env:EDGEAI_JOINT_MAPPING)" "EDGEAI_LANGUAGE=$Language" "EDGEAI_ROS_BACKEND=$Backend" "EDGEAI_DEPTH_OBSTACLES=$DepthObstacles" "EDGEAI_TARGETS=$Targets" "EDGEAI_TARGET_COUNT=$TargetCount" "EDGEAI_COOPERATIVE_RESCUE=$cooperativeRescueFlag" bash "$linuxRoot/scripts/simulation/start-autonomous-exploration.sh" --check --duration $Duration --controller $Controller --planner $Planner --arena $Arena --domain $DomainId
if ($LASTEXITCODE -ne 0) { throw "ROS backend '$Backend' failed its runtime check. Build this backend with scripts/setup/prepare-open-source.sh before starting a test." }
if ($CheckOnly) { return }
if (-not $InCurrentWindow) {
    & (Join-Path $PSScriptRoot 'Stop-Simulation.ps1') -KeepProcessId $PID -Distro $Distro
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow', '-Duration', $Duration, '-Distro', "`"$Distro`"", '-Arena', $Arena, '-DomainId', $DomainId)
    $consoleArgs += @('-Controller', $Controller, '-Backend', $Backend, '-Language', $Language, '-RuntimeProfile', $RuntimeProfile)
    $consoleArgs += @('-Planner', $Planner)
    $consoleArgs += @('-DepthObstacles', $DepthObstacles, '-TargetCount', $TargetCount)
    if ($Targets) { $consoleArgs += @('-Targets', $Targets) }
    if ($JointMapping) { $consoleArgs += '-JointMapping' } else { $consoleArgs += '-DisableJointMapping' }
    if ($TurnTest) { $consoleArgs += '-TurnTest' }
    if ($NavigationTest) { $consoleArgs += '-NavigationTest' }
    if ($CooperativeRescue) { $consoleArgs += '-CooperativeRescue' } else { $consoleArgs += '-DisableCooperativeRescue' }
    if ($Scan360) { $consoleArgs += '-Scan360' }
    if ($MotionTest) { $consoleArgs += @('-MotionTest', $MotionTest) }
    $consoleArgs += @('-PointScenario', $PointScenario)
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
    Write-Host (Get-ProjectText '自主任务已在独立终端启动。' 'Autonomous mission opened in a separate terminal.')
    return
}
$Host.UI.RawUI.WindowTitle = if ($NavigationTest) { 'EdgeAI Navigation Test' } else { 'EdgeAI Autonomous Exploration' }
if ($NavigationTest) {
    Write-Host (Get-ProjectText "导航测试：静态真值地图 + Gazebo 真值定位 + Nav2 $Controller；ROS_DOMAIN_ID=$DomainId。" "Navigation test: static true map + Gazebo truth localization + Nav2 $Controller. ROS_DOMAIN_ID=$DomainId.")
    Write-Host (Get-ProjectText '使用 RViz 2D Goal Pose 选择导航点，Ctrl+C 停止本轮。' 'Use RViz 2D Goal Pose to select navigation goals. Ctrl+C stops this session.')
} else {
    Write-Host (Get-ProjectText "SLAM Toolbox + Nav2 $Controller + frontier_exploration_ros2；协作救援=$CooperativeRescue；后端=$Backend；ROS_DOMAIN_ID=$DomainId。" "SLAM Toolbox + Nav2 $Controller + frontier_exploration_ros2. Cooperative rescue=$CooperativeRescue. Backend=$Backend. ROS_DOMAIN_ID=$DomainId.")
    Write-Host (Get-ProjectText '机器人将在 Gazebo 中自主运动，Ctrl+C 停止本轮。' 'The robot moves autonomously in Gazebo. Ctrl+C stops this session.')
}
# Also cover callers invoking the worker directly.
& (Join-Path $PSScriptRoot 'Stop-Simulation.ps1') -KeepProcessId $PID -Distro $Distro -ProcessesOnly
$tag = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$runName = "$tag-domain$DomainId"
if ($TurnTest) { $runName += '-turn' }
if ($RecoveryTest) { $runName += '-recovery' }
if ($NavigationTest) { $runName += '-navigation-test' }
$sessionDirectory = Join-Path $PSScriptRoot "tests/results/runs/$runName"
[void](New-Item -ItemType Directory -Path $sessionDirectory -Force)
[void](New-Item -ItemType File -Path (Join-Path $sessionDirectory 'events.log') -Force)
$linuxSession = "$linuxRoot/tests/results/runs/$runName"
$mapPath = "$linuxSession/map"
Write-Host (Get-ProjectText "本轮日志：$sessionDirectory" "Session logs: $sessionDirectory")
# The worker already streams events here. Open-ROS-Monitor.ps1 remains a manual tool.
$observer = $null
if ($RuntimeProfile -eq 'debug') {
$observer = Start-Process $windowsPowerShell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', "`"$(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')`"", '-SessionDirectory', "`"$sessionDirectory`"")
}
$captureAck = if ($RuntimeProfile -eq 'debug') { 'true' } else { 'false' }
$scanMode = if ($Scan360) { 'true' } else { 'false' }
$turnMode = if ($TurnTest) { 'true' } else { 'false' }
$recoveryMode = if ($RecoveryTest) { 'true' } else { 'false' }
$navigationTestMode = if ($NavigationTest) { 'true' } else { 'false' }
$motionAblation = if ($MotionGoalOnly) { 'goal_only' } else { '' }
$invertLinear = if ($MPPIInvertLinear) { 'true' } else { 'false' }
$mppiSourceFlag = if ($MPPISource) { 'true' } else { 'false' }
# Use the same managed Linux entry as a direct WSL invocation. PowerShell owns
# only Windows windows/capture; Linux owns test execution and history retention.
Write-Host (Get-ProjectText '正在启动 WSL 托管任务……' 'Starting the managed WSL test entry...')
& $wslExe -d $Distro -- env "EDGEAI_RUNTIME_PROFILE=$RuntimeProfile" "EDGEAI_LANGUAGE=$Language" "EDGEAI_JOINT_MAPPING=$($env:EDGEAI_JOINT_MAPPING)" "EDGEAI_ROS_BACKEND=$Backend" "EDGEAI_DEPTH_OBSTACLES=$DepthObstacles" "EDGEAI_TARGETS=$Targets" "EDGEAI_TARGET_COUNT=$TargetCount" EDGEAI_MANAGED_SESSION=true "EDGEAI_COOPERATIVE_RESCUE=$cooperativeRescueFlag" "EDGEAI_GLOBAL_PLANNER=$Planner" "EDGEAI_NAVIGATION_TEST=$navigationTestMode" "EDGEAI_MPPI_USE_SOURCE=$mppiSourceFlag" "EDGEAI_MPPI_INVERT_LINEAR=$invertLinear" "EDGEAI_MOTION_FRAME=$MotionFrame" "EDGEAI_MOTION_ABLATION=$motionAblation" "EDGEAI_MOTION_TEST=$MotionTest" "EDGEAI_POINT_TEST_SCENARIO=$PointScenario" "EDGEAI_CONTROLLER=$Controller" "EDGEAI_SESSION_DIR=$linuxSession" "EDGEAI_SCAN360=$scanMode" "EDGEAI_TURN_TEST=$turnMode" "EDGEAI_RECOVERY_TEST=$recoveryMode" FORCE_COLOR=1 "EDGEAI_CAPTURE_ACK=$captureAck" bash $linuxRoot/scripts/simulation/start-autonomous-exploration.sh $Duration $mapPath $Arena $DomainId
$result = $LASTEXITCODE
if ($observer -and -not $observer.WaitForExit(15000)) { $observer.Kill() }
Write-Host (Get-ProjectText "自主任务已结束（退出码 $result），窗口保留以便检查。" "Exploration process ended with code $result. The window stays open for inspection.")
if ($result -ne 0) { throw "Exploration failed with code $result." }
