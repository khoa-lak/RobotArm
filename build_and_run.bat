@echo off
echo =======================================================
echo Dang khoi dong ung dung RobotArm HMI...
echo =======================================================

:: 1. Kiem tra Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Khong tim thay Python! Vui long cai dat Python va add vao PATH.
    pause
    exit /b
)

:: 2. Tao thu muc venv neu chua co
if not exist venv (
    echo Dang tao moi truong ao venv...
    python -m venv venv
)

:: 3. Kich hoat moi truong ao
echo Dang kich hoat moi truong ao...
call venv\Scripts\activate

:: 4. Upgrade pip va cai dat dependencies
echo Dang cap nhat pip va cai dat cac thu vien phu thuoc...
python -m pip install --upgrade pip
pip install -r requirements.txt

:: 5. Bien dich module C++ kinematics
echo Dang bien dich module dong hoc C++...
python build_kinematics.py

:: 6. Chay ung dung
echo Dang khoi chay ung dung...
python main.py

pause
