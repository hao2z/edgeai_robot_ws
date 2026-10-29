param(
    [string]$Distro = 'Ubuntu-24.04',
    [ValidateSet('WSLg', 'VcXsrv')][string]$DisplayServer = 'WSLg',
    [ValidateRange(-1, 232)][int]$DomainId = -1,
    [switch]$InCurrentWindow
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
if (-not $InCurrentWindow) {
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow',
        '-Distro', "`"$Distro`"", '-DisplayServer', $DisplayServer, '-DomainId', $DomainId)
    if ($windowsTerminal) {
        Start-Process $windowsTerminal -ArgumentList (@('-w', 'new', 'new-tab', '--title', "`"EdgeAI ROS Commands | domain $DomainId`"",
            '--suppressApplicationTitle', '-d', "`"$PSScriptRoot`"", "`"$windowsPowerShell`"") + $consoleArgs) -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process $windowsPowerShell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host 'ROS command window opened.'
    return
}
$Host.UI.RawUI.WindowTitle = "EdgeAI ROS Commands | $Distro"
Write-Host 'ROS command shell: Pixi and the built workspace are loaded automatically.' -ForegroundColor Cyan
Write-Host 'Useful commands: ros2 node list; ros2 topic list; ros2 topic hz /robot1/scan'
$domainArgument = if ($DomainId -ge 0) { $DomainId.ToString() } else { '' }
if ($DomainId -ge 0) { Write-Host "ROS_DOMAIN_ID=$DomainId" }
& $wslExe -d $Distro -- bash $linuxRoot/scripts/ros/open-ros-terminal.sh $DisplayServer.ToLowerInvariant() $domainArgument
if ($LASTEXITCODE -ne 0) { throw "ROS shell exited with code $LASTEXITCODE." }
