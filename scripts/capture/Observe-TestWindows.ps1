param([Parameter(Mandatory=$true)][string]$SessionDirectory, [switch]$SimulationOnly)
$ErrorActionPreference = 'Stop'
$screenshots = Join-Path $SessionDirectory 'screenshots'
[void][IO.Directory]::CreateDirectory($screenshots)
$capture = Join-Path $PSScriptRoot 'Capture-SimulationWindow.ps1'
$events = Join-Path $SessionDirectory 'events.log'
$log = Join-Path $screenshots 'capture.log'
function Write-Event([string]$Message) {
    # One Linux writer owns events.log. Transfer Windows events as small files
    # to avoid WSL/Windows file sharing conflicts with live readers.
    $path = Join-Path $screenshots "$([Guid]::NewGuid().ToString('N'))"
    [IO.File]::WriteAllText("$path.pending", "$(Get-Date -Format o) [capture] $Message", [Text.UTF8Encoding]::new($false))
    [IO.File]::Move("$path.pending", "$path.event")
}
function Capture-Pair([string]$Phase) {
    foreach ($target in @('Gazebo','RViz')) {
        $path = Join-Path $screenshots "$Phase-$($target.ToLower()).png"
        $arguments = @('-NoProfile', '-ExecutionPolicy','Bypass','-File',$capture,'-Target',$target,'-OutputPath',$path)
        if ($target -eq 'Gazebo') { $arguments += '-Maximize' }
        $ErrorActionPreference = 'Continue'
        $output = & powershell @arguments 2>&1
        $code = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        $output | Out-File -LiteralPath $log -Encoding utf8 -Append
        if ($code -eq 0) {
            Write-Event "截图已保存：$Phase / $target（图像有效性检查通过，画面内容仍需人工核对）"
        } else {
            Write-Event "截图失败：$Phase / $target，详见 screenshots/capture.log"
        }
    }
}
try {
    $deadline = (Get-Date).AddMinutes(3)
    $gazeboReady = $false
    while ((Get-Date) -lt $deadline) {
        if (Test-Path (Join-Path $SessionDirectory 'exit-code.txt')) { return }
        if (-not $gazeboReady) {
            $ErrorActionPreference = 'Continue'
            $output = & powershell -NoProfile -ExecutionPolicy Bypass -File $capture -Target Gazebo -Maximize -OutputPath (Join-Path $screenshots 'gui-startup-gazebo.png') 2>&1
            $code = $LASTEXITCODE
            $ErrorActionPreference = 'Stop'
            if ($code -eq 0) { $gazeboReady = $true; $output | Out-File $log -Encoding utf8 -Append }
        }
        if ((Test-Path (Join-Path $SessionDirectory 'ready')) -or (Test-Path (Join-Path $SessionDirectory 'final-capture'))) { break }
        if ($SimulationOnly -and $gazeboReady) { break }
        Start-Sleep -Seconds 2
    }
    Capture-Pair 'startup'
    $lastCapture = Get-Date
    $counter = 0
    while (-not (Test-Path (Join-Path $SessionDirectory 'exit-code.txt'))) {
        if (Test-Path (Join-Path $SessionDirectory 'final-capture')) {
            Capture-Pair 'final'
            [IO.File]::WriteAllText((Join-Path $SessionDirectory 'capture-finished'), (Get-Date -Format o))
            return
        }
        if (((Get-Date) - $lastCapture).TotalSeconds -ge 60) {
            $counter++
            Capture-Pair "progress-$counter"
            $lastCapture = Get-Date
        }
        Start-Sleep -Seconds 1
    }
} catch {
    $_ | Out-File -LiteralPath $log -Encoding utf8 -Append
    Write-Event "截图观察进程异常：$($_.Exception.Message)"
}
