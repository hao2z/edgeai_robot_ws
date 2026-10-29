param([int]$KeepProcessId = $PID, [string]$Distro = 'Ubuntu-24.04', [switch]$ProcessesOnly)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/ros/windows-host-tools.ps1')
$linuxRoot = (& $wslExe -d $Distro -- wslpath -a -u ($PSScriptRoot.Replace('\','/')) | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve repository path in WSL.' }
# Finish scoped Linux cleanup before terminating the WSL client windows.
& $wslExe -d $Distro -- python3 $linuxRoot/scripts/simulation/stop-test-sessions.py
if ($LASTEXITCODE -ne 0) { throw 'Previous exploration session could not be stopped safely.' }
& $wslExe -d $Distro -- bash $linuxRoot/scripts/simulation/stop-simulation.sh
if ($LASTEXITCODE -ne 0) { throw 'Previous simulation could not be stopped.' }
$processes = @(Get-CimInstance Win32_Process)
$scriptNames = @('Open-ROS-Terminal.ps1', 'Open-ROS-Monitor.ps1',
    'Start-Autonomous-Exploration.ps1', 'Start-Simulation.ps1')
$paths = $scriptNames | ForEach-Object { [regex]::Escape((Join-Path $PSScriptRoot $_)) }
$pathPattern = '(?i)(?:' + ($paths -join '|') + ')(?:"|\s|$)'
$targets = @($processes | Where-Object {
    $_.ProcessId -ne $KeepProcessId -and $_.ProcessId -ne $PID -and
    $_.Name -in @('powershell.exe', 'pwsh.exe') -and
    $_.CommandLine -match $pathPattern -and
    $_.CommandLine -match '(?i)(?:^|\s)-InCurrentWindow(?:\s|$)'
})
foreach ($target in $targets) {
    # A Windows Terminal host can own unrelated windows. Close only these
    # project workers and their WSL clients, never the shared terminal host.
    foreach ($client in @($processes | Where-Object {
        $_.ParentProcessId -eq $target.ProcessId -and $_.Name -eq 'wsl.exe'
    })) {
        Stop-Process -Id $client.ProcessId -ErrorAction SilentlyContinue
    }
    Stop-Process -Id $target.ProcessId -ErrorAction SilentlyContinue
}
Write-Host "Closed $($targets.Count) previous project terminal(s)."
if (-not $ProcessesOnly) {
    & (Join-Path $PSScriptRoot 'scripts/ros/Close-ProjectTerminalWindows.ps1')
}
