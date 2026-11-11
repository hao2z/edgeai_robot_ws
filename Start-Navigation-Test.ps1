param(
    [ValidateRange(30,3600)][int]$Duration=3600,
    [string]$Distro='Ubuntu-24.04',
    [ValidateRange(0,232)][int]$DomainId=86,
    [ValidateSet('dwb','mppi','rpp')][string]$Controller='rpp',
    [ValidateSet('lattice','navfn')][string]$Planner='lattice',
    [switch]$CheckOnly,
    [switch]$InCurrentWindow
)
$ErrorActionPreference='Stop'
& (Join-Path $PSScriptRoot 'Start-Autonomous-Mission.ps1') -NavigationTest -Arena midterm `
    -Duration $Duration -Distro $Distro -DomainId $DomainId -Controller $Controller -Planner $Planner `
    -CheckOnly:$CheckOnly -InCurrentWindow:$InCurrentWindow
