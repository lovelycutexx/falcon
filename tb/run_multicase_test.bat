@echo off
REM ============================================================
REM FalconSign Multi-Case Full-Flow Test — Run Script
REM ============================================================
REM Runs the multi-case testbench through all test scenarios:
REM   Case 0: BYPASS_FS  — identity test (skip ffSampling)
REM   Case 1: FORCE_ACCEPT — real ffSampling, skip rejection
REM   Case 2: FULL SIGN   — complete signing with rejection
REM   Case 3: FULL PIPELINE — SH→HP→FC→FS→VD→IV→FI→N1→RC
REM
REM Prerequisites:
REM   1. iverilog (Icarus Verilog) in PATH
REM   2. Key material .hex files in tb/ directory
REM      (run gen_falcon_hw_key first to generate them)
REM
REM Usage:
REM   run_multicase_test.bat              — basic run
REM   run_multicase_test.bat DUMP_VCD     — generate VCD waveform
REM   run_multicase_test.bat DUMP_PIPE    — dump intermediate data
REM   run_multicase_test.bat STOP_ON_FAIL — stop at first failure
REM   run_multicase_test.bat DUMP_VCD DUMP_PIPE STOP_ON_FAIL
REM ============================================================

setlocal enabledelayedexpansion
set "SCRIPT_DIR=%~dp0"
set "TBDIR=%SCRIPT_DIR%"
set "RTDIR=%SCRIPT_DIR%..\rtl\falcon"
set "TB=tb_falconsign_top_multicase"
set "VVP=%TB%.vvp"
set "PLUSARGS="

REM Parse arguments
for %%a in (%*) do (
    if /I "%%a"=="DUMP_VCD"       set "PLUSARGS=!PLUSARGS! +DUMP_VCD"
    if /I "%%a"=="DUMP_PIPE"      set "PLUSARGS=!PLUSARGS! +DUMP_PIPE"
    if /I "%%a"=="STOP_ON_FAIL"   set "PLUSARGS=!PLUSARGS! +STOP_ON_FAIL"
    if /I "%%a"=="DUMP_FS_TRACE"  set "PLUSARGS=!PLUSARGS! +FS_TRACE_TASKS"
)

REM Check directories
if not exist "%TBDIR%" (
    echo ERROR: testbench directory not found: %TBDIR%
    exit /b 1
)
if not exist "%RTDIR%" (
    echo ERROR: RTL directory not found: %RTDIR%
    exit /b 1
)

pushd "%TBDIR%"

REM Check required hex files
set "MISSING=0"
for %%f in (
    t0_target.hex  t1_target.hex
    b00.hex  b01.hex  b10.hex  b11.hex
    tree_full_poly.hex  h_ntt.hex  hm.hex
) do (
    if not exist "%%f" (
        echo ERROR: %%f not found in %TBDIR%
        echo   Run gen_falcon_hw_key first to generate key material.
        set "MISSING=1"
    )
)
if "!MISSING!"=="1" (
    popd
    exit /b 1
)

REM Copy twiddle ROMs from DOC if needed
if not exist "DOC" mkdir "DOC"
if not exist "DOC\twiddle_rom_re.hex" (
    if exist "..\..\DOC\twiddle_rom_re.hex" (
        copy /Y "..\..\DOC\twiddle_rom_re.hex" "DOC\" >nul
    )
)
if not exist "DOC\twiddle_rom_im.hex" (
    if exist "..\..\DOC\twiddle_rom_im.hex" (
        copy /Y "..\..\DOC\twiddle_rom_im.hex" "DOC\" >nul
    )
)
if not exist "DOC\ntt_twiddle_fwd.hex" (
    if exist "..\..\DOC\ntt_twiddle_fwd.hex" (
        copy /Y "..\..\DOC\ntt_twiddle_fwd.hex" "DOC\" >nul
    )
)
if not exist "DOC\ntt_psi_table.hex" (
    if exist "..\..\DOC\ntt_psi_table.hex" (
        copy /Y "..\..\DOC\ntt_psi_table.hex" "DOC\" >nul
    )
)

REM ============================================================
REM Compile
REM ============================================================
echo.
echo === Compiling %TB% ===
echo Source: %RTDIR%
echo.

iverilog -g2012 -o "%VVP%" ^
    "%RTDIR%\falconsign_top.v" ^
    "%RTDIR%\falconsign_memory.v" ^
    "%RTDIR%\falconsign_shake256.v" ^
    "%RTDIR%\falconsign_keccak_core.v" ^
    "%RTDIR%\falconsign_hash_to_point.v" ^
    "%RTDIR%\falconsign_word_fifo.v" ^
    "%RTDIR%\falcon_fp_fpu.v" ^
    "%RTDIR%\falcon_f64_add.v" ^
    "%RTDIR%\falcon_f64_mul.v" ^
    "%RTDIR%\falcon_f64_fft_exu.v" ^
    "%RTDIR%\falcon_f64_complex_bfly.v" ^
    "%RTDIR%\falcon_fft_addr_gen_cfg.v" ^
    "%RTDIR%\falconsign_twiddle_rom.v" ^
    "%RTDIR%\falconsign_gm_rom.v" ^
    "%RTDIR%\falcon_f64_ffsampling_exu.v" ^
    "%RTDIR%\falconsign_ffsampling_task_update.v" ^
    "%RTDIR%\falcon_ffsampling_iter_ctrl.v" ^
    "%RTDIR%\falconsign_samplerz_top.v" ^
    "%RTDIR%\falconsign_bs_cdt_rom.v" ^
    "%RTDIR%\falconsign_chacha20_rng.v" ^
    "%RTDIR%\falcon_f64_bhat_mul_exu.v" ^
    "%RTDIR%\falcon_f64_vec_sub_exu.v" ^
    "%RTDIR%\falconsign_fpr_to_int16.v" ^
    "%RTDIR%\falconsign_ntt_exu.v" ^
    "%RTDIR%\falconsign_ntt_bfly.v" ^
    "%RTDIR%\falconsign_ntt_addr_gen.v" ^
    "%RTDIR%\falconsign_ntt_cg_addr.v" ^
    "%RTDIR%\falconsign_ntt_psi_rom.v" ^
    "%RTDIR%\falconsign_ntt_twiddle_rom.v" ^
    "%RTDIR%\falconsign_norm_check.v" ^
    "%RTDIR%\falconsign_norm_i16_check.v" ^
    "%RTDIR%\falconsign_norm_i16_sig_check.v" ^
    "%TB%.v"

if %ERRORLEVEL% neq 0 (
    echo.
    echo ERROR: Compilation failed.
    popd
    exit /b 1
)
echo Compilation OK.
echo.

REM ============================================================
REM Run simulation
REM ============================================================
echo === Running simulation ===
echo Plusargs:%PLUSARGS%
echo.

vvp "%VVP%" ^
    +TWIDDLE_RE=DOC/twiddle_rom_re.hex ^
    +TWIDDLE_IM=DOC/twiddle_rom_im.hex ^
    %PLUSARGS%

set "SIM_STATUS=%ERRORLEVEL%"

echo.
if %SIM_STATUS% equ 0 (
    echo === Simulation exited OK ===
) else (
    echo === Simulation exited with status %SIM_STATUS% ===
)

popd
exit /b %SIM_STATUS%
