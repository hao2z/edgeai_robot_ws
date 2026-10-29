param([string]$Distro = 'Ubuntu-24.04', [ValidateRange(-1, 232)][int]$DomainId = -1, [string]$SessionDirectory, [switch]$InCurrentWindow)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
if (-not $InCurrentWindow) {
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow', '-Distro', "`"$Distro`"", '-DomainId', $DomainId)
    if ($SessionDirectory) { $consoleArgs += @('-SessionDirectory', "`"$SessionDirectory`"") }
    if ($windowsTerminal) {
        Start-Process $windowsTerminal -ArgumentList (@('-w', 'new', 'new-tab', '--title', "`"EdgeAI ROS Live Monitor | domain $DomainId`"",
            '--suppressApplicationTitle', '-d', "`"$PSScriptRoot`"", "`"$windowsPowerShell`"") + $consoleArgs) -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process $windowsPowerShell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host 'ROS live monitor window opened.'
    return
}
$Host.UI.RawUI.WindowTitle = "EdgeAI ROS Live Monitor | $Distro"
if ($SessionDirectory) {
    [Console]::OutputEncoding = [Text.Encoding]::UTF8
    Write-Host "Live key events: $SessionDirectory\events.log" -ForegroundColor Cyan
    Write-Host 'Raw simulator/planner/navigation logs are saved in the same session directory.'
    Get-Content -LiteralPath (Join-Path $SessionDirectory 'events.log') -Encoding UTF8 -Wait | ForEach-Object {
        $eventColor = if ($_ -match '\[ERROR\]|启动失败|必要进程提前退出') { 'Red' }
            elseif ($_ -match '\[WARN\]|超时') { 'Yellow' }
            elseif ($_ -match '\[OK\]|已就绪') { 'Green' }
            elseif ($_ -match '\[summary\]|\[session\]') { 'Cyan' }
            else { 'Gray' }
        if ($env:NO_COLOR) { Write-Output $_ } else { Write-Host $_ -ForegroundColor $eventColor }
    }
    return
}
Write-Host 'Live /rosout messages and graph status. No additional log file is saved.' -ForegroundColor Cyan
Write-Host 'Only new ROS messages are shown; Gazebo native startup output is in the simulation window.'
Write-Host 'Ctrl+C stops this monitor without stopping the simulation.'
if ($DomainId -ge 0) {
    Write-Host "ROS_DOMAIN_ID=$DomainId"
    & $wslExe -d $Distro -- bash $linuxRoot/scripts/ros/ros-command.sh env "ROS_DOMAIN_ID=$DomainId" python -u $linuxRoot/scripts/ros/monitor_ros.py
} else {
    & $wslExe -d $Distro -- bash $linuxRoot/scripts/ros/ros-command.sh python -u $linuxRoot/scripts/ros/monitor_ros.py
}
if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) { throw "ROS monitor exited with code $LASTEXITCODE." }
