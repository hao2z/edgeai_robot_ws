param(
    [ValidateRange(30, 3600)][int]$Duration = 300,
    [string]$Distro = 'Ubuntu-24.04',
    [ValidateSet('simple', 'competition')][string]$Arena = 'competition',
    [ValidateRange(0,232)][int]$DomainId = 86,
    [switch]$InCurrentWindow
)
$ErrorActionPreference = 'Stop'
$linuxRoot = (wsl -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
if (-not $InCurrentWindow) {
    & (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow', '-Duration', $Duration, '-Distro', "`"$Distro`"", '-Arena', $Arena, '-DomainId', $DomainId)
    if (Get-Command wt.exe -ErrorAction SilentlyContinue) {
        Start-Process wt.exe -ArgumentList (@('-w', 'new', 'new-tab', '--title', '"EdgeAI Autonomous Exploration"',
            '--suppressApplicationTitle', 'powershell') + $consoleArgs) -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process powershell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host 'Autonomous exploration window opened.'
    return
}
$Host.UI.RawUI.WindowTitle = 'EdgeAI Autonomous Exploration'
Write-Host "Single robot: SLAM Toolbox + Nav2 DWB + frontier_exploration_ros2. ROS_DOMAIN_ID=$DomainId." -ForegroundColor Cyan
Write-Host 'The robot moves autonomously in Gazebo. Ctrl+C stops this session.'
# Also cover callers invoking the worker directly.
& (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro -ProcessesOnly
$tag = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$sessionDirectory = Join-Path $PSScriptRoot "tests/results/runs/$tag-domain$DomainId"
[void](New-Item -ItemType Directory -Path $sessionDirectory -Force)
[void](New-Item -ItemType File -Path (Join-Path $sessionDirectory 'events.log') -Force)
$linuxSession = "$linuxRoot/tests/results/runs/$tag-domain$DomainId"
$mapPath = "$linuxSession/map"
Write-Host "Session logs: $sessionDirectory" -ForegroundColor Cyan
& (Join-Path $PSScriptRoot 'Open-ROS-Terminal.ps1') -Distro $Distro -DomainId $DomainId
& (Join-Path $PSScriptRoot 'Open-ROS-Monitor.ps1') -Distro $Distro -DomainId $DomainId -SessionDirectory $sessionDirectory
$observer = Start-Process powershell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', "`"$(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')`"", '-SessionDirectory', "`"$sessionDirectory`"")
wsl -d $Distro -- env "EDGEAI_SESSION_DIR=$linuxSession" EDGEAI_CAPTURE_ACK=true bash $linuxRoot/scripts/simulation/start-autonomous-exploration.sh $Duration $mapPath $Arena $DomainId
$result = $LASTEXITCODE
if (-not $observer.WaitForExit(15000)) { $observer.Kill() }
Write-Host "Exploration process ended with code $result. The window stays open for inspection."
if ($result -ne 0) { throw "Exploration failed with code $result." }
