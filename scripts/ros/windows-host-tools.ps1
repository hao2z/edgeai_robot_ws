# Dot-source from Windows entry points. Terminal can inherit an incomplete PATH.
# Keep resolution local to this process; do not change the user's system PATH.
# WSL callers may forward an incomplete PATHEXT (for example, only .CPL).
# PowerShell then treats even an absolute wsl.exe as a document in a pipeline.
$executableExtensions = @('.COM', '.EXE', '.BAT', '.CMD')
$inheritedExtensions = @($env:PATHEXT -split ';' | Where-Object { $_ })
$env:PATHEXT = (($executableExtensions + $inheritedExtensions) | Select-Object -Unique) -join ';'
$nativeSystem = Join-Path $env:SystemRoot 'System32'
if ([Environment]::Is64BitOperatingSystem -and -not [Environment]::Is64BitProcess) {
    $nativeSystem = Join-Path $env:SystemRoot 'Sysnative'
}
$windowsPowerShell = Join-Path $nativeSystem 'WindowsPowerShell/v1.0/powershell.exe'
$wslExe = Join-Path $nativeSystem 'wsl.exe'
foreach ($requiredTool in @($windowsPowerShell, $wslExe)) {
    if (-not (Test-Path -LiteralPath $requiredTool -PathType Leaf)) {
        throw "Required Windows executable is missing: $requiredTool"
    }
}
$windowsTerminal = $null
$terminalCommand = Get-Command wt.exe -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($terminalCommand) {
    $windowsTerminal = $terminalCommand.Source
} elseif ($env:LOCALAPPDATA) {
    $terminalAlias = Join-Path $env:LOCALAPPDATA 'Microsoft/WindowsApps/wt.exe'
    if (Test-Path -LiteralPath $terminalAlias -PathType Leaf) {
        $windowsTerminal = $terminalAlias
    }
}
