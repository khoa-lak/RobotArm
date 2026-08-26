@echo off
cd /d %~dp0
echo ============================================================
echo KICH HOAT MOI TRUONG AO VA CAI DAT THU VIEN...
echo ============================================================

if exist venv\Scripts\activate.bat (
    call venv\Scripts\activate.bat
) else if exist ..\venv\Scripts\activate.bat (
    call ..\venv\Scripts\activate.bat
) else if exist ..\firmware-AR4_ar4-hmi\venv\Scripts\activate.bat (
    call ..\firmware-AR4_ar4-hmi\venv\Scripts\activate.bat
) else (
    echo [INFO] Khong tim thay venv san co. Dang tao virtual environment moi (venv)...
    py -3.12 -m venv venv || python -m venv venv
    if exist venv\Scripts\activate.bat (
        call venv\Scripts\activate.bat
    ) else (
        echo [WARNING] Khong the tao venv tu dong. Dung python he thong.
    )
)

echo.
echo Dang cap nhat cac thu vien tu requirements.txt...
pip install -r requirements.txt

echo.
echo ============================================================
echo BIEN DICH THU VIEN DONG HOC C++...
echo ============================================================
python build_kinematics.py

echo.
echo ============================================================
echo KHOI DONG PHAN MEM ROBOT HMI MOI...
echo ============================================================
python main.py

if %errorlevel% neq 0 (
    echo.
    echo Chuong trinh bi dung voi ma loi %errorlevel%. Nhan phim bat ky de thoat.
    pause > nul
)
