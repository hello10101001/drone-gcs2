@echo off
chcp 65001 >NUL
rem ==== 노트북 브리지: 드론 ─ 텔레메트리 무선기(USB) ─ 이 노트북 ─ Wi-Fi ─ 태블릿 앱 ====
rem 아래 네 줄만 고치세요.
rem   COM     : 장치 관리자 → 포트(COM 및 LPT)에서 무선기 번호 (예: COM3)
rem   BAUD    : 무선기 속도 (SiK 기본 57600)
rem   PI      : 드론 라즈베리파이 촬영 프로그램 주소 (사진·영상). 없으면 PI= 로 비우기
rem   TOKEN   : 앱의 브리지 비밀번호, 파이 config.env 의 TOKEN 과 같게
set COM=COM3
set BAUD=57600
set PI=http://10.14.61.56:8080
set TOKEN=drone1234

cd /d "%~dp0"
set PY=python
where python >NUL 2>NUL || set PY=py
%PY% -c "import pymavlink, serial" 2>NUL || (echo [설치] pymavlink, pyserial 설치 중... & %PY% -m pip install pymavlink pyserial)
set PIARG=
if not "%PI%"=="" set PIARG=--pi %PI%
echo.
echo 태블릿 앱: 설정 - 연결 방식 "브리지 · Wi-Fi" - 주소 ws://(이 노트북 IP):8766  /  비밀번호 %TOKEN%
echo 이 노트북 IP:
ipconfig | findstr /R /C:"IPv4"
echo QGroundControl도 같이 켜면 무선기를 같이 씁니다 (UDP 14550 자동 연결).
echo.
%PY% -u gcs_bridge.py --mav %COM%,%BAUD% %PIARG% --qgc udpout:127.0.0.1:14550 --token %TOKEN%
pause
