param(
    [int]$Count = 3,
    [int]$StartNonce = 0
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..")
$TbDir = Join-Path $RepoRoot "tb"
$RunOne = Join-Path $ScriptDir "run_fullkey_test.bat"
$Verifier = Join-Path $RepoRoot "DOC\Falcon\official\falcon-round3\falcon-round3\Reference_Implementation\falcon512\falcon512int\verify_rtl_signature.exe"

$rows = @()

Push-Location $RepoRoot
try {
    for ($caseIdx = 0; $caseIdx -lt $Count; $caseIdx++) {
        $nonce = $StartNonce + $caseIdx
        $log = Join-Path $TbDir ("batch_sign_nonce_{0}.log" -f $nonce)
        Write-Host ("=== Batch sign {0}/{1}: RNG_NONCE={2} ===" -f ($caseIdx + 1), $Count, $nonce)

        $simOutput = & cmd /c "`"$RunOne`" DUMP_SIG ALLOW_SIG_MISMATCH +RNG_NONCE=$nonce" 2>&1
        $simText = ($simOutput | Out-String)
        Set-Content -Path $log -Value $simText -Encoding ASCII

        $verOutput = & {
            Push-Location $TbDir
            try { & $Verifier 2>&1 } finally { Pop-Location }
        }
        $verText = ($verOutput | Out-String)
        Add-Content -Path $log -Value "`n=== Official verifier ===`n$verText" -Encoding ASCII

        $cycles = 0
        $attempts = 0
        $rejects = 0
        $maxRejects = 0
        $normSq = 0
        $bound = 0

        if ($simText -match "total_cycles=(\d+)") { $cycles = [int]$Matches[1] }
        if ($simText -match "SamplerZ rejection stats: attempts=(\d+) rejects=(\d+) max_rejects_per_cmd=(\d+)") {
            $attempts = [int]$Matches[1]
            $rejects = [int]$Matches[2]
            $maxRejects = [int]$Matches[3]
        }
        if ($simText -match "Norm debug: accept=\d+ norm_sq=(\d+) norm_status=0x[0-9a-fA-F]+ bound=(\d+)") {
            $normSq = [int]$Matches[1]
            $bound = [int]$Matches[2]
        }

        $verifyPass = $verText -match "verify_raw=PASS"
        $rows += [pscustomobject]@{
            Nonce = $nonce
            Cycles = $cycles
            Attempts = $attempts
            Rejects = $rejects
            AvgRejects = if ($attempts -ge 0) { [math]::Round($rejects / 1024.0, 3) } else { 0 }
            MaxRejects = $maxRejects
            NormSq = $normSq
            Bound = $bound
            Verify = if ($verifyPass) { "PASS" } else { "FAIL" }
            Log = $log
        }

        Write-Host ("  cycles={0} rejects={1} attempts={2} avg_rejects/cmd={3:N3} verify={4}" -f `
            $cycles, $rejects, $attempts, ($rejects / 1024.0), $(if ($verifyPass) { "PASS" } else { "FAIL" }))

        if (-not $verifyPass) {
            throw "Official verifier failed for RNG_NONCE=$nonce"
        }
    }
}
finally {
    Pop-Location
}

$totalCycles = ($rows | Measure-Object -Property Cycles -Sum).Sum
$totalAttempts = ($rows | Measure-Object -Property Attempts -Sum).Sum
$totalRejects = ($rows | Measure-Object -Property Rejects -Sum).Sum
$maxRejectsAll = ($rows | Measure-Object -Property MaxRejects -Maximum).Maximum

Write-Host ""
Write-Host "=== FalconSign batch summary ==="
$rows | Format-Table -AutoSize
Write-Host ("Average cycles/sign      : {0:N1}" -f ($totalCycles / $rows.Count))
Write-Host ("Average rejects/sign     : {0:N1}" -f ($totalRejects / $rows.Count))
Write-Host ("Average rejects/cmd      : {0:N3}" -f ($totalRejects / ($rows.Count * 1024.0)))
Write-Host ("Average attempts/cmd     : {0:N3}" -f ($totalAttempts / ($rows.Count * 1024.0)))
Write-Host ("Max rejects per command  : {0}" -f $maxRejectsAll)
