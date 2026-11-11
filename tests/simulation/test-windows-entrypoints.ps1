# Isolated dispatch/stop regression: never starts WSL or touches running sessions.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$tempBase = [IO.Path]::GetTempPath()
$sandbox = Join-Path $tempBase ('edgeai-entrypoints-' + [guid]::NewGuid())
[void](New-Item -ItemType Directory -Path $sandbox)
function Assert-True($value, $message) { if (-not $value) { throw $message } }
try {
    Copy-Item -LiteralPath (Join-Path $repo 'Start-Navigation-Test.ps1') -Destination $sandbox
    @'
param($Arena, $Distro, $Duration, $DomainId, $Controller, $Planner,
      [switch]$NavigationTest, [switch]$CheckOnly, [switch]$InCurrentWindow)
[pscustomobject]@{Kind='mission'; Arena=$Arena; Distro=$Distro; Current=[bool]$InCurrentWindow;
    Navigation=[bool]$NavigationTest; Check=[bool]$CheckOnly; Controller=$Controller; Planner=$Planner}
'@ | Set-Content (Join-Path $sandbox 'Start-Autonomous-Mission.ps1')
    $entry = Join-Path $sandbox 'Start-Navigation-Test.ps1'
    $result = & $entry -Distro TestDistro -Controller dwb -Planner navfn -CheckOnly -InCurrentWindow
    Assert-True ($result.Kind -eq 'mission' -and $result.Arena -eq 'midterm' -and $result.Distro -eq 'TestDistro' -and $result.Current -and $result.Navigation -and $result.Check -and $result.Controller -eq 'dwb' -and $result.Planner -eq 'navfn') 'Navigation shortcut forwarding failed'
    # Invalid scene options must fail before any WSL, window or cleanup action.
    $rejected = $false
    try { & (Join-Path $repo 'Start-Simulation.ps1') -Arena midterm -Robots 3 } catch { $rejected = $_.Exception.Message -match 'Midterm supports' }
    Assert-True $rejected 'Unsupported robot count was not rejected before startup'
    Copy-Item -LiteralPath (Join-Path $repo 'Stop-Simulation.ps1') -Destination $sandbox
    [void](New-Item -ItemType Directory -Path (Join-Path $sandbox 'scripts/ros'))
    @'
$wslExe = 'Invoke-FakeWsl'
function Invoke-FakeWsl {
    $global:LASTEXITCODE = 0
    if ($args -contains 'wslpath') { '/workspace'; return }
    $global:edgeaiTestStopCalls.Add(($args -join ' '))
    if ($global:edgeaiTestFailMission -and ($args -join ' ') -match 'stop-test-sessions.py') { $global:LASTEXITCODE = 1 }
}
'@ | Set-Content (Join-Path $sandbox 'scripts/ros/windows-host-tools.ps1')
    $global:edgeaiTestStopCalls = [Collections.Generic.List[string]]::new()
    $global:edgeaiTestFailMission = $false
    & (Join-Path $sandbox 'Stop-Simulation.ps1') -Distro TestDistro -KeepTerminals
    Assert-True ($global:edgeaiTestStopCalls.Count -eq 2 -and $global:edgeaiTestStopCalls[0] -match 'stop-test-sessions.py' -and $global:edgeaiTestStopCalls[1] -match 'stop-simulation.sh') 'Stop order failed'
    Assert-True ($global:edgeaiTestStopCalls[0] -match 'TestDistro') 'Stop distro lost'
    # Mock Windows process discovery and UI closing as well as WSL.
    function Get-CimInstance { param($ClassName) @() }
    '$global:edgeaiTestWindowsClosed = $true' | Set-Content (Join-Path $sandbox 'scripts/ros/Close-ProjectTerminalWindows.ps1')
    $global:edgeaiTestWindowsClosed = $false
    & (Join-Path $sandbox 'Stop-Simulation.ps1')
    Assert-True $global:edgeaiTestWindowsClosed 'Default stop must close project terminals'
    $global:edgeaiTestStopCalls.Clear()
    $global:edgeaiTestFailMission = $true
    $rejected = $false
    try { & (Join-Path $sandbox 'Stop-Simulation.ps1') } catch { $rejected = $true }
    Assert-True ($rejected -and $global:edgeaiTestStopCalls.Count -eq 1) 'Cleanup failure must be reported'
    Write-Host 'PASS: navigation shortcut, simulation option rejection, stop ordering and failure propagation'
} finally {
    Remove-Variable edgeaiTestStopCalls, edgeaiTestFailMission, edgeaiTestWindowsClosed -Scope Global -ErrorAction SilentlyContinue
    $resolved = [IO.Path]::GetFullPath($sandbox)
    $expected = [IO.Path]::GetFullPath($tempBase).TrimEnd('\') + '\edgeai-entrypoints-'
    if (-not $resolved.StartsWith($expected, [StringComparison]::OrdinalIgnoreCase)) { throw 'Unsafe temporary cleanup path' }
    Remove-Item -LiteralPath $resolved -Recurse -Force
}
