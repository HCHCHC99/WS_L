<#
  foc_exp.ps1 - headless HC32F460 FOC experiment runner (J-Link Commander only)

  Runs one comm_mode experiment (21 = open loop, 22 = I-F current loop,
  23 = align), sets the Watch-tunable variables by direct RAM write, samples
  the g_foc_* observables once per second while the core is running, and
  writes a CSV + summary.

  The four direction combos the handoff asks for are:

      pi_off_180=0  enc_dir=+1   ->  .\foc_exp.ps1 -Mode 22 -PiOff180 0 -EncDir 1
      pi_off_180=0  enc_dir=-1   ->  .\foc_exp.ps1 -Mode 22 -PiOff180 0 -EncDir -1
      pi_off_180=1  enc_dir=+1   ->  .\foc_exp.ps1 -Mode 22 -PiOff180 1 -EncDir 1
      pi_off_180=1  enc_dir=-1   ->  .\foc_exp.ps1 -Mode 22 -PiOff180 1 -EncDir -1

      mode21 control:           ->  .\foc_exp.ps1 -Mode 21 -FlashHex

  Requirements:
    - J-Link probe on SWD, HC32F460JETA powered (12V bus)
    - F:\SEGGER\JLink\JLink.exe (V7.92d)
    - The .map file must match the firmware actually flashed (use -FlashHex
      after any rebuild so both stay in sync).

  Safety: the core is halted only while comm_mode=0 (PWM off). During
  sampling the core is RUNNING and J-Link uses background RAM reads.
#>

param(
    [ValidateSet(21,22,23)][int]$Mode          = 22,
    [int]$PiOff180                            = 1,
    [int]$EncDir                               = -1,
    [int]$CurSign                              = -1,
    [float]$IqRefMa                            = 2000,
    [float]$VmaxV                              = 1.0,
    [float]$VrampVS                            = 0.5,
    [float]$HoldIqMa                           = 800,
    [float]$Kp                                 = 0.1,
    [float]$Ki                                 = 240,
    [float]$OutputMaxV                         = 1.0,
    [float]$OlFreqHz                           = 5.0,
    [float]$OlVoltV                            = 0.4,
    [int]$DurationSec                          = 20,
    [switch]$FlashHex,
    [string]$JLinkExe = 'F:\SEGGER\JLink\JLink.exe',
    [string]$MapPath  = 'D:\WS_L\ws_L_v.inmop_curloop\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\MDK\output\debug\template.map',
    [string]$HexPath  = 'D:\WS_L\ws_L_v.inmop_curloop\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\MDK\output\debug\template.hex',
    [string]$OutDir   = 'D:\WS_L\ws_L_v.inmop_curloop\scripts\foc_exp\out'
)

$ErrorActionPreference = 'Stop'

function Get-SymAddr([string]$name) {
    foreach ($line in [System.IO.File]::ReadAllLines($MapPath)) {
        if ($line -match ('^\s*' + [regex]::Escape($name) + '\s+(0x[0-9A-Fa-f]+)')) {
            return [Convert]::ToUInt32($Matches[1], 16)
        }
    }
    throw "symbol not found in map: $name"
}

function To-FloatHex([double]$v) {
    $b = [BitConverter]::GetBytes([single]$v)
    return ('{0:X8}' -f [BitConverter]::ToUInt32($b, 0))
}

function From-FloatHex([string]$hex) {
    $u = [Convert]::ToUInt32($hex, 16)
    return [BitConverter]::ToSingle([BitConverter]::GetBytes($u), 0)
}

# ---------- symbol addresses (parsed fresh from the map each run) ----------
$a_comm_mode   = Get-SymAddr 'comm_mode'
$a_enc_speed   = Get-SymAddr 'g_enc_speed_rpm'
$a_theta       = Get-SymAddr 'g_foc_theta_rad'
$a_active      = Get-SymAddr 'g_foc_active'
$a_ol_freq     = Get-SymAddr 'g_foc_openloop_freq_hz'
$a_ol_volt     = Get-SymAddr 'g_foc_openloop_volt_v'
$a_cur_sign    = Get-SymAddr 'g_foc_cur_sign'
$a_enc_dir     = Get-SymAddr 'g_foc_enc_dir'
$a_pi_off      = Get-SymAddr 'g_foc_pi_off_180'
$a_iq_ref_cmd  = Get-SymAddr 'g_foc_iq_ref_cmd_ma'
$a_iq_ma       = Get-SymAddr 'g_foc_iq_ma'
$a_id_ma       = Get-SymAddr 'g_foc_id_ma'
$a_vq          = Get-SymAddr 'g_foc_vq'
$a_vmax        = Get-SymAddr 'g_foc_vmax_v'
$a_vramp       = Get-SymAddr 'g_foc_vramp_v_s'
$a_vlim        = Get-SymAddr 'g_foc_vlim_v'
$a_if_freq     = Get-SymAddr 'g_foc_if_freq_hz'
$a_if_diff     = Get-SymAddr 'g_foc_if_diff_rad'
$a_if_sweep    = Get-SymAddr 'g_foc_if_sweep_cHz'
$a_if_hold_iq  = Get-SymAddr 'g_foc_if_hold_iq_ma'
$a_if_sync     = Get-SymAddr 'g_foc_if_sync'
$a_if_evt      = Get-SymAddr 'g_foc_if_evt'
$a_if_evt_v1   = Get-SymAddr 'g_foc_if_evt_v1'
$a_align_state = Get-SymAddr 'g_foc_align_state'
$a_fault       = Get-SymAddr 'g_foc_fault'
$a_pid_id      = Get-SymAddr 'g_foc_pid_id_cfg'
$a_pid_iq      = Get-SymAddr 'g_foc_pid_iq_cfg'

# pid_config_t layout (ARMCC, sizeof=36): kp +4, ki +8, output_max +20
$a_id_kp  = $a_pid_id + 4
$a_id_ki  = $a_pid_id + 8
$a_id_omx = $a_pid_id + 20
$a_iq_kp  = $a_pid_iq + 4
$a_iq_ki  = $a_pid_iq + 8
$a_iq_omx = $a_pid_iq + 20

function Hex32([uint32]$v) { return '{0:X8}' -f $v }
function S32Hex([int]$v) { return ('{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes($v), 0)) }

# ---------- J-Link Commander script ----------
$lines = New-Object System.Collections.Generic.List[string]
$lines.Add('EoE 0')
$lines.Add('si SWD')
$lines.Add('speed 4000')
$lines.Add('device HC32F460JETA')
$lines.Add('connect')
if ($FlashHex) { $lines.Add("loadfile $HexPath") }
$lines.Add('r')
$lines.Add('g')
$lines.Add('Sleep 700')
$lines.Add('halt')
# --- write tunables while motor is stopped (comm_mode is 0 after reset) ---
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_cur_sign),   (S32Hex $CurSign)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_enc_dir),    (S32Hex $EncDir)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_pi_off),     (S32Hex $PiOff180)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_iq_ref_cmd), (To-FloatHex $IqRefMa)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_vmax),       (To-FloatHex $VmaxV)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_vramp),      (To-FloatHex $VrampVS)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_if_hold_iq), (To-FloatHex $HoldIqMa)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_iq_kp),      (To-FloatHex $Kp)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_iq_ki),      (To-FloatHex $Ki)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_iq_omx),     (To-FloatHex $OutputMaxV)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_id_kp),      (To-FloatHex $Kp)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_id_ki),      (To-FloatHex $Ki)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_id_omx),     (To-FloatHex $OutputMaxV)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_ol_freq),    (To-FloatHex $OlFreqHz)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_ol_volt),    (To-FloatHex $OlVoltV)))
$lines.Add(("w4 {0} {1}" -f (Hex32 $a_comm_mode),  (S32Hex $Mode)))
$lines.Add('g')

# --- sample once per second while the core is running ---
for ($i = 1; $i -le $DurationSec; $i++) {
    $lines.Add('Sleep 1000')
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_vq))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_vlim))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_iq_ma))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_iq_ref_cmd))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_id_ma))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_enc_speed))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_theta))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_diff))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_freq))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_sweep))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_sync))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_evt))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_if_evt_v1))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_align_state))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_fault))
    $lines.Add("mem32 {0}, 1" -f (Hex32 $a_active))
}

# --- stop cleanly: comm_mode=0 while running (PWM off), then disconnect ---
$lines.Add('Sleep 200')
$lines.Add("w4 {0} 0" -f (Hex32 $a_comm_mode))
$lines.Add('Sleep 200')
$lines.Add('exit')

$ts = Get-Date -Format 'yyyyMMdd_HHmmss'
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
$scriptPath = Join-Path $OutDir "run_$ts.jlink"
$outPath    = Join-Path $OutDir "run_$ts.out.txt"
$csvPath    = Join-Path $OutDir "run_$ts.csv"

[System.IO.File]::WriteAllLines($scriptPath, $lines, (New-Object System.Text.UTF8Encoding($false)))

# ---------- run ----------
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $JLinkExe
$psi.Arguments = '-NoGui 1 -CommandFile "{0}"' -f $scriptPath
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError  = $true
$psi.UseShellExecute = $false
$p = [System.Diagnostics.Process]::Start($psi)
$stdout = $p.StandardOutput.ReadToEnd()
$stderr = $p.StandardError.ReadToEnd()
$p.WaitForExit()
[System.IO.File]::WriteAllText($outPath, $stdout + "`r`n=== STDERR ===`r`n" + $stderr, (New-Object System.Text.UTF8Encoding($false)))

# ---------- parse mem32 output ----------
$rows = New-Object System.Collections.Generic.List[object]
$idx = 0
$cur = @{}
foreach ($line in ($stdout -split "`r?`n")) {
    if ($line -match '([0-9A-Fa-f]{8})\s*=\s*([0-9A-Fa-f]{8})') {
        $addr = [Convert]::ToUInt32($Matches[1], 16)
        $val  = [Convert]::ToUInt32($Matches[2], 16)
        switch ($addr) {
            $a_vq         { $cur.vq    = [Math]::Round((From-FloatHex $Matches[2]) * 1000.0, 1) }
            $a_vlim       { $cur.vlim  = [Math]::Round((From-FloatHex $Matches[2]) * 1000.0, 1) }
            $a_iq_ma      { $cur.iq    = [Math]::Round((From-FloatHex $Matches[2]), 1) }
            $a_iq_ref_cmd { $cur.iqref = [Math]::Round((From-FloatHex $Matches[2]), 1) }
            $a_id_ma      { $cur.id    = [Math]::Round((From-FloatHex $Matches[2]), 1) }
            $a_enc_speed  { $cur.spd   = [Math]::Round((From-FloatHex $Matches[2]), 1) }
            $a_theta      { $cur.theta = [Math]::Round((From-FloatHex $Matches[2]) * 1000.0, 1) }
            $a_if_diff    { $cur.diff  = [Math]::Round((From-FloatHex $Matches[2]) * 1000.0, 1) }
            $a_if_freq    { $cur.freq  = [Math]::Round((From-FloatHex $Matches[2]) * 100.0, 1) }
            $a_if_sweep   { $cur.sweep = [Math]::Round((From-FloatHex $Matches[2]), 1) }
            $a_if_sync    { $cur.sync  = $val }
            $a_if_evt     { $cur.evt   = $val }
            $a_if_evt_v1  { $cur.evtv1 = $val }
            $a_align_state{ $cur.state = $val }
            $a_fault      { $cur.fault = $val }
            $a_active     {
                $cur.active = $val
                $idx++
                $rows.Add([pscustomobject]@{
                    t      = $idx
                    mode   = $Mode
                    pi     = $PiOff180
                    enc    = $EncDir
                    vq_mV  = $cur.vq
                    vlim_mV= $cur.vlim
                    iq_mA  = $cur.iq
                    iqref_mA=$cur.iqref
                    id_mA  = $cur.id
                    spd_rpm= $cur.spd
                    theta_mrad=$cur.theta
                    diff_mrad=$cur.diff
                    freq_cHz=$cur.freq
                    sweep_cHz=$cur.sweep
                    sync   = $cur.sync
                    evt    = $cur.evt
                    evtv1  = $cur.evtv1
                    state  = $cur.state
                    fault  = $cur.fault
                    active = $cur.active
                })
                $cur = @{}
            }
        }
    }
}

$rows | Export-Csv -Path $csvPath -NoTypeInformation -Encoding UTF8

# ---------- summary ----------
Write-Output ''
Write-Output ("run      : {0}" -f $ts)
Write-Output ("mode     : {0}   pi_off_180={1} enc_dir={2} cur_sign={3}" -f $Mode,$PiOff180,$EncDir,$CurSign)
Write-Output ("samples  : {0}" -f $rows.Count)
if ($rows.Count -gt 0) {
    $last = $rows[$rows.Count-1]
    $spdMin = ($rows | Measure-Object -Property spd_rpm -Minimum).Minimum
    $spdMax = ($rows | Measure-Object -Property spd_rpm -Maximum).Maximum
    $vqMin  = ($rows | Measure-Object -Property vq_mV -Minimum).Minimum
    $vqMax  = ($rows | Measure-Object -Property vq_mV -Maximum).Maximum
    Write-Output ("spd range: {0} .. {1} rpm" -f $spdMin, $spdMax)
    Write-Output ("vq  range: {0} .. {1} mV  (output_max = {2} mV)" -f $vqMin, $vqMax, [Math]::Round($OutputMaxV*1000))
    Write-Output ("vlim last: {0} mV   iq last: {1} mA / ref {2} mA" -f $last.vlim_mV, $last.iq_mA, $last.iqref_mA)
    Write-Output ("diff last: {0} mrad   sweep last: {1} cHz   fault: {2}" -f $last.diff_mrad, $last.sweep_cHz, $last.fault)
    if (($rows | Where-Object { $_.evt -eq 2 }).Count -gt 0) {
        Write-Output 'RESULT   : HANDOVER OK (evt=2 -> RUN mode)'
    } elseif (($rows | Where-Object { $_.fault -eq 2 }).Count -gt 0) {
        Write-Output 'RESULT   : I-F TIMEOUT (fault=2)'
    } elseif (($rows | Where-Object { $_.fault -eq 1 }).Count -gt 0) {
        Write-Output 'RESULT   : OVER-CURRENT FAULT (fault=1)'
    } else {
        Write-Output 'RESULT   : still running (no event/fault in window)'
    }
}
Write-Output ("csv      : {0}" -f $csvPath)
Write-Output ("raw log  : {0}" -f $outPath)
Write-Output ("jlink cmd: {0}" -f $scriptPath)