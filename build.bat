@echo off

:: build.bat
:: build.bat debug
:: build.bat release
:: build.bad <debug/release> run

set release=0
if "%~1"=="release" set release=1

if "%release%"=="1" (
echo -----RELEASE BUILD-----
cl.exe ^
	/W4 /MD /I include /I lib\glfw\include ^
	main.c ^
	/link /LIBPATH:lib\glfw\bin\win /out:client.exe glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib vcruntime.lib msvcrt.lib
echo -----RELEASE BUILD-----
)

if "%release%"=="0" (
echo -----DEBUG BUILD-----
cl.exe ^
    /W4 /Zi /Od /MDd /I include /I lib\glfw\include ^
    main.c ^
    /link /MACHINE:X64 /LIBPATH:lib\glfw\bin\win /out:client_debug.exe glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib vcruntime.lib msvcrtd.lib
echo -----DEBUG BUILD-----
)

if %errorlevel% neq 0 exit /b %errorlevel%

if "%~2"=="run" (
	if "%release%"=="1" client.exe
	if "%release%"=="0" client_debug.exe
)