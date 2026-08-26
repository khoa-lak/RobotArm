# RobotArm - Modern Robot Control HMI

Giao diện điều khiển cánh tay robot AR4 (6 bậc tự do - 6 DOF) hiện đại viết bằng **Python**, **CustomTkinter** và **VTK 3D Engine**.

## 🌟 Tính Năng Chính
- **Giao diện hiện đại (Modern Dark UI)**: Tùy biến với CustomTkinter.
- **3D Robot Simulation (VTK)**: Tải và hiển thị mô hình 3D STL của từng khớp cánh tay, cập nhật trực tiếp theo thời gian thực (Real-time Kinematics Forward Display).
- **C++ Kinematics Engine**: Thư viện động học được viết bằng C++ tối ưu tốc độ tính toán, kết nối qua pybind11 (obot_kinematics).
- **Giao tiếp Serial**: Tương thích hoàn toàn với giao thức truyền thông STM32 / Arduino của AR4 Robot.
- **Jog Controls**: Điều khiển từng khớp (J1 - J6) mượt mà với nhiều tốc độ và bước dịch chuyển.

## 🚀 Hướng Dẫn Cài Đặt & Chạy

### Cách 1: Chạy tự động bằng file Batch (Khuyên dùng trên Windows)
Chỉ cần nhấp đúp vào file:
`
build_and_run.bat
`
Script sẽ tự động kích hoạt môi trường ảo, cài đặt các gói thư viện cần thiết, biên dịch module C++ và khởi động ứng dụng HMI.

### Cách 2: Chạy thủ công
1. Khởi tạo & kích hoạt môi trường ảo:
`ash
python -m venv venv
venv\Scripts\activate
`
2. Cài đặt các thư viện phụ thuộc:
`ash
pip install -r requirements.txt
`
3. Biên dịch module động học C++ (Tùy chọn nếu muốn tăng tốc):
`ash
python build_kinematics.py
`
4. Khởi chạy ứng dụng:
`ash
python main.py
`

## 📁 Cấu Trúc Thư Mục
`
RobotArm/
├── assets/
│   ├── icons/            # Icon giao diện
│   └── robot_model/      # File 3D STL các khớp AR4 (Link Base, Link 1 -> 6)
├── config/
│   └── defaults.json     # File cấu hình thông số DH & thông số robot
├── core/
│   ├── config_manager.py # Quản lý đọc/ghi cấu hình
│   ├── kinematics.py     # Module tính toán động học robot (C++ & Python fallback)
│   └── serial_driver.py  # Driver giao tiếp UART/Serial với mạch điều khiển
├── gui/
│   ├── main_window.py    # Giao diện chính (CustomTkinter)
│   └── vtk_viewer.py     # Cửa sổ hiển thị mô hình 3D VTK
├── kinematics_src/       # Mã nguồn C++ cho động học
├── build_and_run.bat     # File chạy nhanh
├── build_kinematics.py   # Script biên dịch C++ pybind11
├── main.py               # Điểm khởi chạy chương trình
└── requirements.txt      # Danh sách thư viện Python
`
