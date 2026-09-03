[CmdletBinding()]
param(
    [ValidateRange(5, 120)]
    [int]$DetectionTimeoutSeconds = 30,

    [ValidateRange(1, 4)]
    [int]$ExpectedActiveControllers = 1,

    [ValidateRange(2, 30)]
    [int]$InputTestSeconds = 6,

    [ValidateRange(100, 5000)]
    [int]$RumbleMilliseconds = 600,

    [switch]$SkipInteractive,

    [string]$OutputPath = (Join-Path ([System.IO.Path]::GetTempPath()) "switch-pico-adapter-feasibility-results.json")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if ($PSVersionTable.PSEdition -eq "Core" -and -not $IsWindows) {
    throw "This test must run on Windows."
}

Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;

public static class SwitchPicoXInput {
    [StructLayout(LayoutKind.Sequential)]
    public struct Gamepad {
        public ushort Buttons;
        public byte LeftTrigger;
        public byte RightTrigger;
        public short LeftThumbX;
        public short LeftThumbY;
        public short RightThumbX;
        public short RightThumbY;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct State {
        public uint PacketNumber;
        public Gamepad Gamepad;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Vibration {
        public ushort LeftMotorSpeed;
        public ushort RightMotorSpeed;
    }

    [DllImport("xinput1_4.dll", EntryPoint = "XInputGetState")]
    public static extern uint GetState(uint userIndex, out State state);

    [DllImport("xinput1_4.dll", EntryPoint = "XInputSetState")]
    public static extern uint SetState(uint userIndex, ref Vibration vibration);
}
"@

$script:Results = [System.Collections.Generic.List[object]]::new()

function Add-Result {
    param(
        [string]$Name,
        [bool]$Passed,
        [string]$Detail,
        [bool]$Required = $true
    )

    $script:Results.Add([pscustomobject]@{
        Name = $Name
        Passed = $Passed
        Required = $Required
        Detail = $Detail
    })

    $color = if ($Passed) { "Green" } elseif ($Required) { "Red" } else { "Yellow" }
    $label = if ($Passed) { "PASS" } elseif ($Required) { "FAIL" } else { "SKIP" }
    Write-Host ("[{0}] {1}: {2}" -f $label, $Name, $Detail) -ForegroundColor $color
}

function Get-UsbIdentityDevices {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Vid,
        [Parameter(Mandatory = $true)]
        [string]$ProductId
    )

    $pattern = "VID_{0}&PID_{1}" -f $Vid.ToUpperInvariant(), $ProductId.ToUpperInvariant()
    if (Get-Command Get-PnpDevice -ErrorAction SilentlyContinue) {
        return @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
            Where-Object { $_.InstanceId -like "*$pattern*" })
    }

    return @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
        Where-Object { $_.PNPDeviceID -like "*$pattern*" })
}

function Get-XInputState {
    param([ValidateRange(0, 3)][int]$Index)

    $state = [SwitchPicoXInput+State]::new()
    $status = [SwitchPicoXInput]::GetState([uint32]$Index, [ref]$state)
    return [pscustomobject]@{
        Index = $Index
        Connected = ($status -eq 0)
        Status = $status
        PacketNumber = $state.PacketNumber
        Buttons = $state.Gamepad.Buttons
        LeftTrigger = $state.Gamepad.LeftTrigger
        RightTrigger = $state.Gamepad.RightTrigger
        LeftThumbX = $state.Gamepad.LeftThumbX
        LeftThumbY = $state.Gamepad.LeftThumbY
        RightThumbX = $state.Gamepad.RightThumbX
        RightThumbY = $state.Gamepad.RightThumbY
    }
}

function Get-ConnectedXInputStates {
    return @(0..3 | ForEach-Object { Get-XInputState -Index $_ } |
        Where-Object { $_.Connected })
}

function Test-StateChanged {
    param($Before, $After)

    return $Before.PacketNumber -ne $After.PacketNumber -or
        $Before.Buttons -ne $After.Buttons -or
        $Before.LeftTrigger -ne $After.LeftTrigger -or
        $Before.RightTrigger -ne $After.RightTrigger -or
        $Before.LeftThumbX -ne $After.LeftThumbX -or
        $Before.LeftThumbY -ne $After.LeftThumbY -or
        $Before.RightThumbX -ne $After.RightThumbX -or
        $Before.RightThumbY -ne $After.RightThumbY
}

function Set-XInputRumble {
    param(
        [ValidateRange(0, 3)][int]$Index,
        [ValidateRange(0, 65535)][int]$Left,
        [ValidateRange(0, 65535)][int]$Right
    )

    $vibration = [SwitchPicoXInput+Vibration]::new()
    $vibration.LeftMotorSpeed = [uint16]$Left
    $vibration.RightMotorSpeed = [uint16]$Right
    return [SwitchPicoXInput]::SetState([uint32]$Index, [ref]$vibration)
}

function Read-YesNo {
    param([string]$Prompt)

    while ($true) {
        $answer = (Read-Host "$Prompt [y/n]").Trim().ToLowerInvariant()
        if ($answer -eq "y" -or $answer -eq "yes") { return $true }
        if ($answer -eq "n" -or $answer -eq "no") { return $false }
    }
}

Write-Host "Switch Pico Windows XInput feasibility test" -ForegroundColor Cyan
Write-Host "Prototype identities: Switch probe 057E:2009 -> XInput CAFE:4010"
Write-Host "Disconnect other XInput controllers before continuing."

$baseline = @(Get-ConnectedXInputStates)
Add-Result -Name "Clean XInput baseline" -Passed ($baseline.Count -eq 0) `
    -Detail ("{0} XInput controller(s) connected before the Pico" -f $baseline.Count)

if (-not $SkipInteractive) {
    Write-Host ""
    Write-Host "Disconnect the Pico from USB." -ForegroundColor Yellow
    [void](Read-Host "Press Enter after it is disconnected")

    $disconnectDeadline = (Get-Date).AddSeconds(10)
    do {
        $switchPresent = @(Get-UsbIdentityDevices -Vid "057E" -ProductId "2009").Count -gt 0
        $xinputPresent = @(Get-UsbIdentityDevices -Vid "CAFE" -ProductId "4010").Count -gt 0
        if (-not $switchPresent -and -not $xinputPresent) { break }
        Start-Sleep -Milliseconds 100
    } while ((Get-Date) -lt $disconnectDeadline)

    Add-Result -Name "Pico disconnected" `
        -Passed (-not $switchPresent -and -not $xinputPresent) `
        -Detail "Neither prototype USB identity is present"

    Write-Host "Connect the Pico to this Windows PC now." -ForegroundColor Yellow
} else {
    Write-Host "Non-interactive mode: inspecting the currently connected Pico." -ForegroundColor Yellow
}

$transitionStart = Get-Date
$deadline = $transitionStart.AddSeconds($DetectionTimeoutSeconds)
$seenSwitch = $false
$seenXInput = $false
$switchSeenAt = $null
$xinputSeenAt = $null

while ((Get-Date) -lt $deadline) {
    if (-not $seenSwitch -and @(Get-UsbIdentityDevices -Vid "057E" -ProductId "2009").Count -gt 0) {
        $seenSwitch = $true
        $switchSeenAt = (Get-Date)
        Write-Host "Observed Switch probe identity 057E:2009" -ForegroundColor DarkCyan
    }

    if (@(Get-UsbIdentityDevices -Vid "CAFE" -ProductId "4010").Count -gt 0) {
        $seenXInput = $true
        $xinputSeenAt = (Get-Date)
        Write-Host "Observed XInput identity CAFE:4010" -ForegroundColor DarkCyan
        break
    }
    Start-Sleep -Milliseconds 50
}

if ($SkipInteractive -and $seenXInput) {
    Add-Result -Name "Automatic identity transition" -Passed $true -Required $false `
        -Detail "XInput identity is present; initial Switch probe was not observed in this run"
} else {
    $transitionPassed = $seenSwitch -and $seenXInput -and $xinputSeenAt -gt $switchSeenAt
    $transitionMs = if ($transitionPassed) {
        [math]::Round(($xinputSeenAt - $switchSeenAt).TotalMilliseconds)
    } else { -1 }
    Add-Result -Name "Automatic identity transition" -Passed $transitionPassed `
        -Detail ("SwitchSeen={0}, XInputSeen={1}, transition={2} ms" -f $seenSwitch, $seenXInput, $transitionMs)
}

$xinputPnp = @(Get-UsbIdentityDevices -Vid "CAFE" -ProductId "4010")
Add-Result -Name "XInput PnP identity" -Passed ($xinputPnp.Count -gt 0) `
    -Detail ("Found {0} present PnP node(s) for CAFE:4010" -f $xinputPnp.Count)

$problemDevices = @($xinputPnp | Where-Object {
    $_.PSObject.Properties.Name -contains "Status" -and $_.Status -notin @("OK", "Unknown")
})
Add-Result -Name "PnP device health" -Passed ($problemDevices.Count -eq 0) `
    -Detail ("{0} unhealthy PnP node(s)" -f $problemDevices.Count)

Start-Sleep -Seconds 2
$connectedStates = @(Get-ConnectedXInputStates)
Add-Result -Name "Four XInput API slots" -Passed ($connectedStates.Count -eq 4) `
    -Detail ("XInputGetState reports {0}/4 connected slots" -f $connectedStates.Count)

$inputSummaries = @()
if (-not $SkipInteractive) {
    for ($index = 0; $index -lt $ExpectedActiveControllers; $index++) {
        Write-Host ""
        Write-Host ("For {0} seconds, exercise physical controller slot {1}:" -f $InputTestSeconds, ($index + 1)) -ForegroundColor Yellow
        Write-Host "  D-pad, face buttons, both shoulders, both triggers, and both sticks."
        Write-Host "  Also press Home/Guide and Capture/Share to observe their raw, non-portable XUSB bits."
        [void](Read-Host "Press Enter to start capture")

        $before = Get-XInputState -Index $index
        $changed = $false
        $samples = 0
        $observedControls = [ordered]@{
            DPad = $false
            FaceButtons = $false
            Shoulders = $false
            LeftTrigger = $false
            RightTrigger = $false
            LeftStick = $false
            RightStick = $false
        }
        $observedRawSystemButtons = [ordered]@{
            GuideBit0x0400 = $false
            ShareBit0x0800 = $false
        }
        $captureDeadline = (Get-Date).AddSeconds($InputTestSeconds)
        while ((Get-Date) -lt $captureDeadline) {
            $after = Get-XInputState -Index $index
            if (Test-StateChanged -Before $before -After $after) {
                $changed = $true
            }
            if (($after.Buttons -band 0x000F) -ne 0) { $observedControls.DPad = $true }
            if (($after.Buttons -band 0xF000) -ne 0) { $observedControls.FaceButtons = $true }
            if (($after.Buttons -band 0x0300) -ne 0) { $observedControls.Shoulders = $true }
            if (($after.Buttons -band 0x0400) -ne 0) { $observedRawSystemButtons.GuideBit0x0400 = $true }
            if (($after.Buttons -band 0x0800) -ne 0) { $observedRawSystemButtons.ShareBit0x0800 = $true }
            if ($after.LeftTrigger -gt 0) { $observedControls.LeftTrigger = $true }
            if ($after.RightTrigger -gt 0) { $observedControls.RightTrigger = $true }
            if ([math]::Abs([int]$after.LeftThumbX) -gt 8000 -or
                [math]::Abs([int]$after.LeftThumbY) -gt 8000) {
                $observedControls.LeftStick = $true
            }
            if ([math]::Abs([int]$after.RightThumbX) -gt 8000 -or
                [math]::Abs([int]$after.RightThumbY) -gt 8000) {
                $observedControls.RightStick = $true
            }
            $before = $after
            $samples++
            Start-Sleep -Milliseconds 20
        }
        $missingControls = @($observedControls.GetEnumerator() |
            Where-Object { -not $_.Value } |
            ForEach-Object { $_.Key })
        $inputPassed = $changed -and $missingControls.Count -eq 0
        $inputSummaries += [pscustomobject]@{
            Slot = $index
            Changed = $changed
            Samples = $samples
            LastPacket = $before.PacketNumber
            Observed = $observedControls
            Missing = $missingControls
            RawSystemButtonBits = $observedRawSystemButtons
        }
        Add-Result -Name ("XInput slot {0} complete input" -f ($index + 1)) `
            -Passed $inputPassed `
            -Detail ("samples={0}, packet={1}, missing=[{2}]" -f $samples, $before.PacketNumber, ($missingControls -join ", "))
        Write-Host ("  Raw wButtons observations (not portable XInput qualification): Guide 0x0400={0}, Share 0x0800={1}" -f `
            $observedRawSystemButtons.GuideBit0x0400, $observedRawSystemButtons.ShareBit0x0800) -ForegroundColor DarkCyan
    }

    for ($index = 0; $index -lt $ExpectedActiveControllers; $index++) {
        Write-Host ""
        Write-Host ("Sending rumble to XInput slot {0} for {1} ms." -f ($index + 1), $RumbleMilliseconds) -ForegroundColor Yellow
        $startStatus = Set-XInputRumble -Index $index -Left 65535 -Right 49152
        Start-Sleep -Milliseconds $RumbleMilliseconds
        $stopStatus = Set-XInputRumble -Index $index -Left 0 -Right 0
        $observed = Read-YesNo -Prompt ("Did only physical controller slot {0} vibrate" -f ($index + 1))
        Add-Result -Name ("XInput slot {0} rumble/isolation" -f ($index + 1)) `
            -Passed ($startStatus -eq 0 -and $stopStatus -eq 0 -and $observed) `
            -Detail ("XInputSetState start={0}, stop={1}, observed={2}" -f $startStatus, $stopStatus, $observed)
    }
} else {
    Add-Result -Name "Interactive input and rumble" -Passed $true -Required $false `
        -Detail "Skipped by -SkipInteractive"
}

$pnpDetails = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
    Where-Object { $_.PNPDeviceID -like "*VID_CAFE&PID_4010*" } |
    Select-Object Name, PNPDeviceID, Service, ConfigManagerErrorCode)

$report = [ordered]@{
    TimestampUtc = [DateTime]::UtcNow.ToString("o")
    ComputerName = $env:COMPUTERNAME
    PowerShellVersion = $PSVersionTable.PSVersion.ToString()
    Parameters = [ordered]@{
        DetectionTimeoutSeconds = $DetectionTimeoutSeconds
        ExpectedActiveControllers = $ExpectedActiveControllers
        InputTestSeconds = $InputTestSeconds
        RumbleMilliseconds = $RumbleMilliseconds
        SkipInteractive = [bool]$SkipInteractive
    }
    Transition = [ordered]@{
        SeenSwitch = $seenSwitch
        SeenXInput = $seenXInput
        SwitchSeenAt = if ($switchSeenAt) { $switchSeenAt.ToUniversalTime().ToString("o") } else { $null }
        XInputSeenAt = if ($xinputSeenAt) { $xinputSeenAt.ToUniversalTime().ToString("o") } else { $null }
    }
    ConnectedXInputSlots = @($connectedStates | Select-Object Index, PacketNumber)
    InputSummaries = $inputSummaries
    PnpDevices = $pnpDetails
    Results = @($script:Results)
}

$reportJson = $report | ConvertTo-Json -Depth 8
[System.IO.File]::WriteAllText(
    $OutputPath,
    $reportJson,
    [System.Text.UTF8Encoding]::new($false))
Write-Host ""
Write-Host ("Wrote report: {0}" -f (Resolve-Path $OutputPath)) -ForegroundColor Cyan

$failed = @($script:Results | Where-Object { $_.Required -and -not $_.Passed })
if ($failed.Count -gt 0) {
    Write-Host ("Feasibility test failed: {0} required check(s) failed." -f $failed.Count) -ForegroundColor Red
    exit 1
}

Write-Host "All required feasibility checks passed." -ForegroundColor Green
exit 0
