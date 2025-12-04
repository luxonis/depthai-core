@echo off
setlocal enabledelayedexpansion

REM Usage:
REM   get_artifacts_and_run_tests.cmd <repo> <gh_token> <dest> <artifact_id>

set "REPO=%~1"
set "GH_TOKEN=%~2"
set "DEST=%~3"
set "ARTIFACT_ID=%~4"

if "%REPO%"==""       ( echo [!] REPO required & exit /b 2 )
if "%GH_TOKEN%"==""   ( echo [!] GH_TOKEN required & exit /b 2 )
if "%DEST%"==""       ( echo [!] DEST required & exit /b 2 )
if "%ARTIFACT_ID%"=="" ( echo [!] ARTIFACT_ID required & exit /b 2 )

echo [*] REPO=%REPO%
echo [*] DEST =%DEST%
echo [*] ARTIFACT_ID=%ARTIFACT_ID%

echo [*] Recreating %DEST%
rmdir /S /Q "%DEST%" 2>nul
mkdir "%DEST%" || exit /b 1


set "ZIP=%TEMP%\artifact_%ARTIFACT_ID%.zip"
echo [*] Downloading artifact id=%ARTIFACT_ID%
curl -sSL -H "Authorization: Bearer %GH_TOKEN%" -H "Accept: application/vnd.github+json" ^
  --retry 5 --retry-delay 2 --retry-all-errors ^
  -o "%ZIP%" "https://api.github.com/repos/%REPO%/actions/artifacts/%ARTIFACT_ID%/zip" || exit /b 1

echo [*] Extracting to %DEST%
tar -xf "%ZIP%" -C "%DEST%"
del /q "%ZIP%"

call venv\Scripts\activate.bat
python -c "import sys; print(sys.executable)"

if exist "%DEST%\tests" (
  echo [*] Running tests from %DEST%\tests
  cd "%DEST%\tests"
  python run_tests.py --rvc4
) else (
  echo [*] No tests directory at %DEST%\tests — skipping test phase.
  exit /b 1
)
