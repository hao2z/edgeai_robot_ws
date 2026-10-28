param([string]$OutputPath = (Join-Path $PSScriptRoot '../../tests/results/screenshots/windows-desktop.png'))
$ErrorActionPreference = 'Stop'
Add-Type @'
using System.Runtime.InteropServices;
public static class EdgeAIScreenCapture {
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
}
'@
[void][EdgeAIScreenCapture]::SetProcessDPIAware()
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$bounds = [System.Windows.Forms.SystemInformation]::VirtualScreen
$bitmap = [System.Drawing.Bitmap]::new($bounds.Width, $bounds.Height)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
try {
    $graphics.CopyFromScreen($bounds.X, $bounds.Y, 0, 0, $bounds.Size, [System.Drawing.CopyPixelOperation]::SourceCopy)
    $bitmap.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
    [pscustomobject]@{ Path = $OutputPath; Width = $bounds.Width; Height = $bounds.Height; SessionId = (Get-Process -Id $PID).SessionId } | ConvertTo-Json -Compress
} finally {
    $graphics.Dispose()
    $bitmap.Dispose()
}
