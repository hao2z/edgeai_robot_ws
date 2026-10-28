$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class EdgeAITerminalWindows {
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr h, uint message, IntPtr w, IntPtr l);
}
'@
$root = [Windows.Automation.AutomationElement]::RootElement
$tabCondition = [Windows.Automation.PropertyCondition]::new(
    [Windows.Automation.AutomationElement]::ControlTypeProperty, [Windows.Automation.ControlType]::TabItem)
$buttonCondition = [Windows.Automation.PropertyCondition]::new(
    [Windows.Automation.AutomationElement]::ControlTypeProperty, [Windows.Automation.ControlType]::Button)
$pattern = '^EdgeAI (?:ROS Commands|ROS Live Monitor|Autonomous Exploration|Simulation)(?:\s*\||$)'
$closed = 0
foreach ($window in $root.FindAll([Windows.Automation.TreeScope]::Children, [Windows.Automation.Condition]::TrueCondition)) {
    try {
        if ($window.Current.ClassName -ne 'CASCADIA_HOSTING_WINDOW_CLASS') { continue }
        $tabs = @($window.FindAll([Windows.Automation.TreeScope]::Descendants, $tabCondition))
        $projectTabs = @($tabs | Where-Object { $_.Current.Name -match $pattern })
        if ($projectTabs.Count -eq 0) { continue }
        if ($projectTabs.Count -eq $tabs.Count) {
            # Every tab is ours. Closing this HWND leaves other Terminal windows alone.
            [void][EdgeAITerminalWindows]::PostMessage([IntPtr]$window.Current.NativeWindowHandle, 0x10, [IntPtr]::Zero, [IntPtr]::Zero)
            $closed += $projectTabs.Count
        } else {
            foreach ($tab in $projectTabs) {
                foreach ($button in $tab.FindAll([Windows.Automation.TreeScope]::Descendants, $buttonCondition)) {
                    if ($button.Current.Name -match '^(Close tab|关闭标签页)$') {
                        $invoke = $button.GetCurrentPattern([Windows.Automation.InvokePattern]::Pattern)
                        $invoke.Invoke()
                        $closed++
                        break
                    }
                }
            }
        }
    } catch [Windows.Automation.ElementNotAvailableException] {
        # The last tab may disappear while the worker process exits.
    }
}
Write-Host "Closed $closed project terminal tab(s), including exited tabs."
