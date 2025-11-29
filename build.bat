
@echo off
setlocal

REM --- Configuration ---
set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
set "OUT_DIR=build"
set "EXE_NAME=Aegis.exe"

REM --- Setup Environment ---
if not defined DevEnvDir (
    call "%VS_PATH%"
)

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"

REM --- Sources ---
REM Core ImGui
set "IMGUI_SRC=external\imgui\imgui.cpp external\imgui\imgui_demo.cpp external\imgui\imgui_draw.cpp external\imgui\imgui_tables.cpp external\imgui\imgui_widgets.cpp"
REM Backends (DX11 + Win32)
set "BACKEND_SRC=external\imgui\backends\imgui_impl_dx11.cpp external\imgui\backends\imgui_impl_win32.cpp"
REM App
set "APP_SRC=src\main.cpp src\network\UdpSocket.cpp src\radar\TrackManager.cpp src\radar\Track.cpp src\physics\KalmanFilter.cpp src\physics\ExtendedKalmanFilter.cpp"

REM --- Includes ---
set "INCLUDES=/Iinclude /Iexternal\glm /Iexternal\imgui /Iexternal\imgui\backends"

REM --- Flags ---
set "CFLAGS=/nologo /std:c++20 /W4 /EHsc /MD /O2"
set "LDFLAGS=/link /out:%OUT_DIR%\%EXE_NAME% d3d11.lib d3dcompiler.lib dxgi.lib"

REM --- Compile ---
echo Compiling...
cl %CFLAGS% %APP_SRC% %IMGUI_SRC% %BACKEND_SRC% %INCLUDES% /D_CRT_SECURE_NO_WARNINGS /DUNICODE /D_UNICODE /Fo%OUT_DIR%\ %LDFLAGS%

REM --- Compile Sender ---
echo Compiling Sender...
set "SENDER_SRC=src\sender_main.cpp src\network\UdpSocket.cpp src\radar\TargetGenerator.cpp"
cl %CFLAGS% %SENDER_SRC% %INCLUDES% /D_CRT_SECURE_NO_WARNINGS /Fo%OUT_DIR%\ /link /out:%OUT_DIR%\Sender.exe ws2_32.lib

if %errorlevel% neq 0 exit /b %errorlevel%

echo Compiling Tests...
cl /EHsc /std:c++17 %INCLUDES% /I src tests\test_kalman.cpp src\physics\KalmanFilter.cpp /Fe:build\test_kalman.exe
if %errorlevel% neq 0 exit /b %errorlevel%

echo Compiling EKF Tests...
cl /EHsc /std:c++20 %INCLUDES% /I src tests\test_ekf.cpp src\physics\ExtendedKalmanFilter.cpp /Fe:build\test_ekf.exe /Fo%OUT_DIR%\
if %errorlevel% neq 0 exit /b %errorlevel%

if %ERRORLEVEL% EQU 0 (
    echo Build Successful! Run %OUT_DIR%\%EXE_NAME% or %OUT_DIR%\Sender.exe
) else (
    echo Build Failed!
)
