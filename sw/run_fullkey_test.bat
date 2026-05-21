@echo off
REM ============================================================
REM FalconSign Full Key Integration Test - Run Script
REM ============================================================
REM Run build_and_run.bat first to generate/copy the hex files.
REM Usage:
REM   run_fullkey_test.bat
REM   run_fullkey_test.bat DUMP_VCD
REM   run_fullkey_test.bat FORCE_ACCEPT
REM   run_fullkey_test.bat BYPASS_FS
REM ============================================================

setlocal enabledelayedexpansion
set "SCRIPT_DIR=%~dp0"
set "TBDIR=%SCRIPT_DIR%..\tb"
set "RT=%SCRIPT_DIR%..\rtl\falcon"
set "TB=tb_falconsign_top_fullkey"
set "VVP=%TB%.vvp"
set "EXTRA_ARGS="
set "DUMP_ARG="

for %%a in (%*) do (
    if /I "%%a"=="DUMP_VCD" set "DUMP_ARG=+DUMP_VCD"
    if /I "%%a"=="FORCE_ACCEPT" set "EXTRA_ARGS=!EXTRA_ARGS! +FORCE_ACCEPT"
    if /I "%%a"=="BYPASS_FS" set "EXTRA_ARGS=!EXTRA_ARGS! +BYPASS_FS"
    if /I "%%a"=="FS_SAMPLE_MU" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_SAMPLE_MU"
    if /I "%%a"=="ZERO_TREE" set "EXTRA_ARGS=!EXTRA_ARGS! +ZERO_TREE"
    if /I "%%a"=="FS_ADJUST_NOP" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_ADJUST_NOP"
    if /I "%%a"=="FS_TRACE_TASKS" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_TASKS"
    if /I "%%a"=="FS_TRACE_EXU" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_EXU"
)

if not exist "%TBDIR%" (
    echo ERROR: testbench directory not found:
    echo   %TBDIR%
    exit /b 1
)
if not exist "%RT%" (
    echo ERROR: RTL directory not found:
    echo   %RT%
    exit /b 1
)

pushd "%TBDIR%"

if not exist "DOC" mkdir "DOC"
if not exist "DOC\twiddle_rom_re.hex" copy /Y "..\..\DOC\twiddle_rom_re.hex" "DOC\" >nul
if not exist "DOC\twiddle_rom_im.hex" copy /Y "..\..\DOC\twiddle_rom_im.hex" "DOC\" >nul
if not exist "DOC\ntt_twiddle_fwd.hex" copy /Y "..\..\DOC\ntt_twiddle_fwd.hex" "DOC\" >nul
if not exist "DOC\ntt_psi_table.hex" copy /Y "..\..\DOC\ntt_psi_table.hex" "DOC\" >nul
copy /Y "..\sw\gm_rom_re.hex" "DOC\" >nul
copy /Y "..\sw\gm_rom_im.hex" "DOC\" >nul

for %%f in (
    t0_target.hex t1_target.hex
    b00.hex b01.hex b10.hex b11.hex
    tree_full_poly.hex h_ntt.hex hm.hex
) do (
    if not exist "%%f" (
        echo ERROR: %%f not found. Run build_and_run.bat first.
        popd
        exit /b 1
    )
)

echo === Compiling %TB% ===
iverilog -g2012 -o "%VVP%" ^
    "%RT%\falconsign_top.v" ^
    "%RT%\falconsign_memory.v" ^
    "%RT%\falconsign_shake256.v" ^
    "%RT%\falconsign_keccak_core.v" ^
    "%RT%\falconsign_hash_to_point.v" ^
    "%RT%\falconsign_word_fifo.v" ^
    "%RT%\falcon_fp_fpu.v" ^
    "%RT%\falcon_f64_add.v" ^
    "%RT%\falcon_f64_mul.v" ^
    "%RT%\falcon_f64_fft_exu.v" ^
    "%RT%\falcon_f64_complex_bfly.v" ^
    "%RT%\falcon_fft_addr_gen_cfg.v" ^
    "%RT%\falconsign_twiddle_rom.v" ^
    "%RT%\falconsign_gm_rom.v" ^
    "%RT%\falcon_f64_ffsampling_exu.v" ^
    "%RT%\falconsign_ffsampling_task_update.v" ^
    "%RT%\falcon_ffsampling_iter_ctrl.v" ^
    "%RT%\falconsign_samplerz_top.v" ^
    "%RT%\falconsign_bs_cdt_rom.v" ^
    "%RT%\falconsign_chacha20_rng.v" ^
    "%RT%\falcon_f64_bhat_mul_exu.v" ^
    "%RT%\falcon_f64_vec_sub_exu.v" ^
    "%RT%\falconsign_fpr_to_int16.v" ^
    "%RT%\falconsign_ntt_exu.v" ^
    "%RT%\falconsign_ntt_bfly.v" ^
    "%RT%\falconsign_ntt_addr_gen.v" ^
    "%RT%\falconsign_ntt_psi_rom.v" ^
    "%RT%\falconsign_ntt_twiddle_rom.v" ^
    "%RT%\falconsign_norm_check.v" ^
    "%RT%\falconsign_norm_i16_check.v" ^
    "%RT%\falconsign_norm_i16_sig_check.v" ^
    "%TB%.v"
if %ERRORLEVEL% neq 0 (
    echo ERROR: Compilation failed.
    popd
    exit /b 1
)

echo === Running simulation ===
vvp "%VVP%" %DUMP_ARG% +TWIDDLE_RE=DOC/twiddle_rom_re.hex +TWIDDLE_IM=DOC/twiddle_rom_im.hex %EXTRA_ARGS%
set "SIM_STATUS=%ERRORLEVEL%"
popd
exit /b %SIM_STATUS%
