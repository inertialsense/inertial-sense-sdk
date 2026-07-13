@echo off
:: =============================================================================
:: generate-doxygen.bat
::
:: Description:
::   Generates Doxygen HTML (and optionally PDF) documentation for the
::   Inertial Sense SDK on Windows.  Runtime settings are injected by piping
::   the base Doxyfile together with override lines into `doxygen -`, so the
::   committed scripts\Doxyfile is never modified.
::
:: Usage:
::   generate-doxygen.bat [output_dir] [/pdf]
::
::   output_dir   Directory where docs are written (default: .\docs)
::   /pdf         Also generate a PDF via LaTeX (requires pdflatex in PATH)
::
:: Examples:
::   generate-doxygen.bat
::   generate-doxygen.bat .\build\docs
::   generate-doxygen.bat .\build\docs /pdf
::
:: Notes:
::   - Run from any directory; the script resolves paths relative to itself.
::   - Requires doxygen.exe in PATH (https://www.doxygen.nl/download.html).
::   - For PDF output, a TeX distribution such as MiKTeX or TeX Live must be
::     installed and pdflatex.exe must be in PATH.
:: =============================================================================

setlocal enabledelayedexpansion

:: ---------------------------------------------------------------------------
:: Resolve repo root (one level up from this script)
:: ---------------------------------------------------------------------------

set "SCRIPT_DIR=%~dp0"
:: SCRIPT_DIR ends with \, navigate up two levels (windows\ -> scripts\ -> repo root)
pushd "%SCRIPT_DIR%..\.."
set "REPO_ROOT=%CD%"
popd

set "DOXYFILE=%REPO_ROOT%\scripts\Doxyfile"

:: ---------------------------------------------------------------------------
:: Defaults
:: ---------------------------------------------------------------------------

set "OUTPUT_DIR=%REPO_ROOT%\docs"
set "GENERATE_PDF=NO"

:: ---------------------------------------------------------------------------
:: Argument parsing
:: ---------------------------------------------------------------------------

:parse_args
if "%~1"=="" goto :args_done
if /i "%~1"=="/pdf" (
    set "GENERATE_PDF=YES"
    shift
    goto :parse_args
)
if /i "%~1"=="/h" goto :usage
if /i "%~1"=="/help" goto :usage
:: Treat any other first non-flag argument as the output directory
set "OUTPUT_DIR=%~1"
shift
goto :parse_args
:args_done

:: ---------------------------------------------------------------------------
:: Dependency check: doxygen
:: ---------------------------------------------------------------------------

where doxygen >nul 2>&1
if errorlevel 1 (
    echo ERROR: doxygen not found in PATH.
    echo   Download and install from: https://www.doxygen.nl/download.html
    echo   Then add the install directory to your system PATH.
    exit /b 1
)

:: ---------------------------------------------------------------------------
:: Dependency check: pdflatex (only needed for /pdf)
:: ---------------------------------------------------------------------------

if "%GENERATE_PDF%"=="YES" (
    where pdflatex >nul 2>&1
    if errorlevel 1 (
        echo ERROR: pdflatex not found in PATH ^(required for /pdf^).
        echo   Install MiKTeX from: https://miktex.org/download
        echo   or TeX Live from:    https://tug.org/texlive/
        exit /b 1
    )
)

:: ---------------------------------------------------------------------------
:: Create output directory
:: ---------------------------------------------------------------------------

if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

:: ---------------------------------------------------------------------------
:: Determine version via git tag (best-effort; empty string is fine)
:: ---------------------------------------------------------------------------

set "VERSION="
for /f "delims=" %%v in ('git -C "%REPO_ROOT%" describe --tags --abbrev=0 2^>nul') do set "VERSION=%%v"

:: ---------------------------------------------------------------------------
:: Resolve source INPUT directories
:: ---------------------------------------------------------------------------

set "INPUT_DIRS="
if exist "%REPO_ROOT%\src" (
    set "INPUT_DIRS=%REPO_ROOT%\src"
)
if exist "%REPO_ROOT%\cltool\src" (
    if defined INPUT_DIRS (
        set "INPUT_DIRS=!INPUT_DIRS! %REPO_ROOT%\cltool\src"
    ) else (
        set "INPUT_DIRS=%REPO_ROOT%\cltool\src"
    )
)

if not defined INPUT_DIRS (
    echo ERROR: No source directories found under %REPO_ROOT%
    exit /b 1
)

:: ---------------------------------------------------------------------------
:: Status output
:: ---------------------------------------------------------------------------

echo Generating Inertial Sense SDK documentation...
echo   Repo root   : %REPO_ROOT%
echo   Source dirs : %INPUT_DIRS%
echo   Output dir  : %OUTPUT_DIR%
if defined VERSION (echo   Version     : %VERSION%) else (echo   Version     : ^<not detected^>)
echo   PDF         : %GENERATE_PDF%
echo.

:: ---------------------------------------------------------------------------
:: Build the piped Doxyfile override and run doxygen
::
:: Windows does not have a clean equivalent of bash process substitution, so
:: we write the overrides to a temp file, concatenate with the base Doxyfile,
:: and pipe the combined content into `doxygen -`.
:: ---------------------------------------------------------------------------

set "TMPOVERRIDE=%TEMP%\is_sdk_doxy_overrides_%RANDOM%.txt"

(
    echo.
    echo # --- Runtime overrides injected by generate-doxygen.bat ---
    echo OUTPUT_DIRECTORY       = %OUTPUT_DIR%
    echo INPUT                  = %INPUT_DIRS%
    echo GENERATE_LATEX         = %GENERATE_PDF%
    if defined VERSION echo PROJECT_NUMBER         = %VERSION%
    if exist "%REPO_ROOT%\README.md" echo USE_MDFILE_AS_MAINPAGE = %REPO_ROOT%\README.md
) > "%TMPOVERRIDE%"

type "%DOXYFILE%" "%TMPOVERRIDE%" | doxygen -
set "DOXY_EXIT=%errorlevel%"

del /f /q "%TMPOVERRIDE%" 2>nul

if %DOXY_EXIT% neq 0 (
    echo ERROR: doxygen exited with code %DOXY_EXIT%.
    exit /b %DOXY_EXIT%
)

echo.
echo HTML documentation written to: %OUTPUT_DIR%\html\index.html

:: ---------------------------------------------------------------------------
:: Optional PDF build
:: ---------------------------------------------------------------------------

if "%GENERATE_PDF%"=="YES" (
    set "LATEX_DIR=%OUTPUT_DIR%\latex"
    echo.
    echo Building PDF from LaTeX sources in: !LATEX_DIR!
    if not exist "!LATEX_DIR!" (
        echo ERROR: LaTeX output directory not found: !LATEX_DIR!
        exit /b 1
    )
    pushd "!LATEX_DIR!"
    make
    set "MAKE_EXIT=!errorlevel!"
    popd
    if !MAKE_EXIT! neq 0 (
        echo ERROR: LaTeX make failed with code !MAKE_EXIT!.
        exit /b !MAKE_EXIT!
    )
    if exist "!LATEX_DIR!\refman.pdf" (
        echo PDF written to: !LATEX_DIR!\refman.pdf
    ) else (
        echo ERROR: PDF build finished but refman.pdf not found.
        exit /b 1
    )
)

echo.
echo Done.
exit /b 0

:: ---------------------------------------------------------------------------
:usage
echo.
echo Usage: generate-doxygen.bat [output_dir] [/pdf]
echo.
echo   output_dir   Output directory for generated docs (default: .\docs^)
echo   /pdf         Also generate a PDF via LaTeX (requires pdflatex^)
echo.
exit /b 0
