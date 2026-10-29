param(
    [ValidateSet('VcXsrv', 'WSLg')][string]$DisplayServer = 'WSLg',
    [ValidateRange(1, 4)][int]$Robots = 1,
    [switch]$SLAM,
    [string]$Distro = 'Ubuntu-24.04',
    [switch]$InCurrentWindow
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
if (-not $InCurrentWindow) {
    & (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro
    $consoleArgs = @('-NoLogo', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-InCurrentWindow',
        '-DisplayServer', $DisplayServer, '-Robots', $Robots, '-Distro', "`"$Distro`"")
    if ($SLAM) { $consoleArgs += '-SLAM' }
    if ($windowsTerminal) {
        Start-Process $windowsTerminal -ArgumentList (@('-w', 'new', 'new-tab', '--title', '"EdgeAI Simulation"',
            '--suppressApplicationTitle', '-d', "`"$PSScriptRoot`"", "`"$windowsPowerShell`"") + $consoleArgs) -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    } else {
        Start-Process $windowsPowerShell -ArgumentList $consoleArgs -WorkingDirectory $PSScriptRoot -WindowStyle Normal
    }
    Write-Host 'Simulation output window opened.'
    return
}
$Host.UI.RawUI.WindowTitle = "EdgeAI Simulation | $Distro | $Robots robot(s)"
& (Join-Path $PSScriptRoot 'Close-ROS-Terminals.ps1') -KeepProcessId $PID -Distro $Distro -ProcessesOnly
$tag = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$sessionDirectory = Join-Path $PSScriptRoot "tests/results/runs/$tag-simulation"
[void][IO.Directory]::CreateDirectory($sessionDirectory)
$linuxSession = "$linuxRoot/tests/results/runs/$tag-simulation"
Write-Host "Live simulation logs: $sessionDirectory" -ForegroundColor Cyan
Write-Host "Stop the simulation with Stop-Simulation.ps1 from another PowerShell window."
$checkDir = Join-Path $PSScriptRoot 'runtime'
if ($DisplayServer -eq 'VcXsrv') {
    $serverPath = 'D:\VcXsrv\vcxsrv.exe'
    $authPath = Join-Path $checkDir 'vcxsrv.xauth'
    $logPath = Join-Path $checkDir 'vcxsrv.log'
    if (-not (Test-Path -LiteralPath $serverPath)) { throw "VcXsrv is missing: $serverPath" }
    $listener = Get-NetTCPConnection -State Listen -LocalPort 6001 -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($listener) {
        $server = Get-CimInstance Win32_Process -Filter "ProcessId=$($listener.OwningProcess)"
        if ($server.ExecutablePath -ne $serverPath -or $server.CommandLine -notlike '*Competition*vcxsrv.xauth*') {
            throw 'Display :1 is already occupied by another X server.'
        }
    } else {
        & $wslExe -d $Distro -- python3 $linuxRoot/scripts/display/configure-vcxsrv-auth.py
        if ($LASTEXITCODE -ne 0) { throw 'Xauthority creation failed.' }
        $sid = [Security.Principal.WindowsIdentity]::GetCurrent().User
        # icacls changes the DACL without requesting SACL privileges.
        & (Join-Path $nativeSystem 'icacls.exe') $authPath '/inheritance:r' '/grant:r' "*$($sid.Value):(F)" | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Xauthority access restriction failed.' }
        $serverArgs = ":1 -multiwindow -clipboard -wgl -listen tcp -auth `"$authPath`" -logfile `"$logPath`""
        $server = Start-Process -FilePath $serverPath -ArgumentList $serverArgs -WindowStyle Hidden -PassThru
        $server.Id | Set-Content (Join-Path $checkDir 'vcxsrv.pid')
        Start-Sleep -Milliseconds 800
    }
}
$backend = $DisplayServer.ToLowerInvariant()
if ($DisplayServer -eq 'WSLg') {
    # Check before opening any of this session's windows. Repair only the known
    # gfxredir=0 failure; leave healthy sessions and the Ubuntu VM running.
    & $windowsPowerShell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'Repair-WSLg.ps1') -Distro $Distro
    if ($LASTEXITCODE -ne 0) { throw 'WSLg readiness/graphics redirection check failed.' }
}
$multi = if ($Robots -gt 1) { 'true' } else { 'false' }
$mapping = if ($SLAM) { 'true' } else { 'false' }
& (Join-Path $PSScriptRoot 'Open-ROS-Terminal.ps1') -Distro $Distro -DisplayServer $DisplayServer
$observer = Start-Process $windowsPowerShell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass',
    '-File', "`"$(Join-Path $PSScriptRoot 'scripts/capture/Observe-TestWindows.ps1')`"", '-SessionDirectory', "`"$sessionDirectory`"", '-SimulationOnly')
& $wslExe -d $Distro -- env "EDGEAI_SESSION_DIR=$linuxSession" EDGEAI_CAPTURE_ACK=true bash $linuxRoot/scripts/simulation/run-simulation.sh $backend "robot_count:=$Robots" "multi_robot:=$multi" "slam:=$mapping"
$simulationExitCode = $LASTEXITCODE
if (-not $observer.WaitForExit(15000)) { $observer.Kill() }
Write-Host "Simulation ended (exit code $simulationExitCode). This window stays open for inspection." -ForegroundColor Cyan
if ($simulationExitCode -ne 0) { throw "Simulation exited with code $simulationExitCode." }
