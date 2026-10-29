param(
    [ValidateSet('Gazebo', 'RViz')][string]$Target = 'Gazebo',
    [ValidateSet('Auto', 'PrintWindow', 'Screen')][string]$Method = 'Auto',
    [string]$OutputPath,
    [int]$WindowIndex = -1,
    [switch]$Maximize,
    [switch]$Background,
    [switch]$FitWorkArea
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '../ros/windows-host-tools.ps1')
if (-not $OutputPath) { $OutputPath = Join-Path $PSScriptRoot "../../tests/results/screenshots/$($Target.ToLower())-$($Method.ToLower()).png" }
[void][IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName([IO.Path]::GetFullPath($OutputPath)))
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public static class EdgeAIWindowCapture {
    public delegate bool EnumCallback(IntPtr h, IntPtr p);
    [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left, Top, Right, Bottom; }
    [StructLayout(LayoutKind.Sequential)] public struct MonitorInfo { public uint Size; public Rect Monitor; public Rect Work; public uint Flags; }
    [DllImport("user32.dll")] public static extern IntPtr MonitorFromWindow(IntPtr h, uint flags);
    [DllImport("user32.dll")] public static extern bool GetMonitorInfo(IntPtr monitor, ref MonitorInfo info);
    public static void FitWorkArea(IntPtr h) {
        var info = new MonitorInfo(); info.Size = (uint)Marshal.SizeOf(info);
        if (!GetMonitorInfo(MonitorFromWindow(h, 2), ref info)) throw new InvalidOperationException("Cannot read monitor work area");
        ShowWindowAsync(h, 9);
        SetWindowPos(h, IntPtr.Zero, info.Work.Left, info.Work.Top,
            info.Work.Right-info.Work.Left, info.Work.Bottom-info.Work.Top, 0x14);
    }
    [StructLayout(LayoutKind.Sequential)] public struct Point { public int X; public int Y; }
    [StructLayout(LayoutKind.Sequential)] public struct WindowPlacement { public uint Length; public uint Flags; public uint ShowCmd; public Point MinPosition; public Point MaxPosition; public Rect NormalPosition; }
    [DllImport("user32.dll")] public static extern bool GetWindowPlacement(IntPtr h, ref WindowPlacement placement);
    public static Rect CaptureBounds(IntPtr h) {
        Rect bounds; GetWindowRect(h, out bounds);
        if (IsIconic(h)) {
            var placement = new WindowPlacement(); placement.Length = (uint)Marshal.SizeOf(placement);
            if (GetWindowPlacement(h, ref placement)) bounds = placement.NormalPosition;
        }
        return bounds;
    }
    public class WindowInfo { public IntPtr Handle; public string Title; public string ClassName; public bool Minimized; public Rect Bounds; }
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumCallback cb, IntPtr p);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetClassName(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out Rect r);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr dc, uint flags);
    [DllImport("user32.dll")] public static extern bool ShowWindowAsync(IntPtr h, int n);
    [DllImport("user32.dll")] public static extern bool IsZoomed(IntPtr h);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int w, int hgt, uint flags);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    public static WindowInfo[] Find(string target, bool includeHidden) {
        var result = new List<WindowInfo>();
        EnumWindows((h,p) => {
            var title = new StringBuilder(2048); GetWindowText(h,title,title.Capacity);
            var cls = new StringBuilder(256); GetClassName(h,cls,cls.Capacity);
            string text = title.ToString();
            bool match = target == "Gazebo" ? text.StartsWith("Gazebo Sim") : text.Contains(" - RViz");
            if (match && (includeHidden || IsWindowVisible(h))) {
                Rect rect; GetWindowRect(h, out rect);
                result.Add(new WindowInfo {Handle=h, Title=text, ClassName=cls.ToString(), Minimized=IsIconic(h), Bounds=rect});
            }
            return true;
        }, IntPtr.Zero);
        return result.ToArray();
    }
}
'@
[void][EdgeAIWindowCapture]::SetProcessDPIAware()
if ($Background -and ($Maximize -or $FitWorkArea -or $Method -eq 'Screen')) {
    throw 'Background capture cannot resize/maximize windows or use foreground screen capture.'
}
$windows = @([EdgeAIWindowCapture]::Find($Target, [bool]$Background))
if ($Background -and $windows.Count -eq 0) {
    [pscustomobject]@{Target=$Target; Skipped=$true; Reason='Window unavailable'; Success=$false} | ConvertTo-Json -Compress
    return
}
if ($WindowIndex -lt 0) {
    if ($windows.Count -ne 1) { throw "Expected one $Target window; found $($windows.Count). Specify -WindowIndex for a fleet." }
    $WindowIndex = 0
}
if ($WindowIndex -ge $windows.Count) { throw "Window index $WindowIndex is out of range for $($windows.Count) windows." }
$window = $windows[$WindowIndex]
if ($FitWorkArea) {
    [EdgeAIWindowCapture]::FitWorkArea($window.Handle)
    Start-Sleep -Milliseconds 500
}
if ($Maximize) {
    [void][EdgeAIWindowCapture]::ShowWindowAsync($window.Handle, 3)
    Start-Sleep -Milliseconds 800
}
$captureMethod = if ($Method -eq 'Auto') { 'PrintWindow' } else { $Method }
if (-not $Background -and ([EdgeAIWindowCapture]::IsIconic($window.Handle) -or (-not $Maximize -and (($window.Bounds.Right-$window.Bounds.Left) -lt 300 -or ($window.Bounds.Bottom-$window.Bounds.Top) -lt 200)))) {
    [void][EdgeAIWindowCapture]::ShowWindowAsync($window.Handle, 9)
    Start-Sleep -Milliseconds 800
}
if ($captureMethod -eq 'Screen') {
    if ([EdgeAIWindowCapture]::IsIconic($window.Handle)) { [void][EdgeAIWindowCapture]::ShowWindowAsync($window.Handle, 9) }
    Start-Sleep -Milliseconds 300
    [void][EdgeAIWindowCapture]::SetWindowPos($window.Handle, [IntPtr](-1), 0, 0, 0, 0, 0x43)
    [void][EdgeAIWindowCapture]::SetForegroundWindow($window.Handle)
    [void][EdgeAIWindowCapture]::SetWindowPos($window.Handle, [IntPtr](-2), 0, 0, 0, 0, 0x43)
    Start-Sleep -Milliseconds 800
    if ([EdgeAIWindowCapture]::GetForegroundWindow() -ne $window.Handle) {
        throw "Could not bring $Target to the foreground; screenshot would include other windows."
    }
}
$bounds = [EdgeAIWindowCapture+Rect]::new()
$bounds = [EdgeAIWindowCapture]::CaptureBounds($window.Handle)
$width = $bounds.Right - $bounds.Left
$height = $bounds.Bottom - $bounds.Top
if ($width -lt 300 -or $height -lt 200) { throw 'Window is not fully restored; refusing to capture only a title bar.' }
$bitmap = [Drawing.Bitmap]::new($width, $height)
$graphics = [Drawing.Graphics]::FromImage($bitmap)
try {
    if ($captureMethod -eq 'PrintWindow') {
        $dc = $graphics.GetHdc()
        try { $success = [EdgeAIWindowCapture]::PrintWindow($window.Handle, $dc, 2) }
        finally { $graphics.ReleaseHdc($dc) }
    } else {
        $graphics.CopyFromScreen($bounds.Left, $bounds.Top, 0, 0, [Drawing.Size]::new($width, $height))
        $success = $true
    }
    $bitmap.Save($OutputPath, [Drawing.Imaging.ImageFormat]::Png)
    $colors = [Collections.Generic.HashSet[int]]::new()
    $dark = 0; $samples = 0
    for ($y=10; $y -lt $height-10; $y+=19) {
        for ($x=10; $x -lt $width-10; $x+=19) {
            $pixel = $bitmap.GetPixel($x,$y)
            [void]$colors.Add($pixel.ToArgb()); $samples++
            if ($pixel.R -lt 5 -and $pixel.G -lt 5 -and $pixel.B -lt 5) { $dark++ }
        }
    }
    if (-not $Background -and $Method -eq 'Auto' -and (-not $success -or $colors.Count -lt 8 -or $dark/[double]$samples -gt 0.98)) {
        $fallbackArguments = @('-ExecutionPolicy','Bypass','-File',$PSCommandPath,'-Target',$Target,'-WindowIndex',$WindowIndex,'-Method','Screen','-OutputPath',$OutputPath)
        if ($Maximize) { $fallbackArguments += '-Maximize' }
        & $windowsPowerShell @fallbackArguments
        exit $LASTEXITCODE
    }
    $valid = $success -and $colors.Count -ge 8 -and $dark/[double]$samples -le 0.98
    $metadata = [pscustomobject]@{Time=(Get-Date).ToString('o'); Target=$Target; Method=$captureMethod; Success=$valid; Path=$OutputPath; Width=$width; Height=$height; Maximized=[EdgeAIWindowCapture]::IsZoomed($window.Handle); ClassName=$window.ClassName; UniqueSampleColors=$colors.Count; BlackFraction=[Math]::Round($dark/[double]$samples,3)} | ConvertTo-Json -Compress
    $metadata | Set-Content -LiteralPath "$OutputPath.json" -Encoding UTF8
    $metadata
    if (-not $valid -and $Background) {
        [pscustomobject]@{Target=$Target; Skipped=$true; Reason='Background image unavailable; window state preserved'; Success=$false} | ConvertTo-Json -Compress
    } elseif (-not $valid) { throw 'Capture failed image validity checks; inspect metadata and retry.' }
} finally { $graphics.Dispose(); $bitmap.Dispose() }
