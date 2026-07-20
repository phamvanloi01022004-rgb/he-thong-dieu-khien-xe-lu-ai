# Hệ thống hỗ trợ điều khiển an toàn cho xe lu sử dụng AI

Repository này lưu trữ mã nguồn của hệ thống hỗ trợ điều khiển an toàn cho xe lu sử dụng Jetson Nano, ESP32 và xử lý ảnh bằng trí tuệ nhân tạo.

# Kiến trúc hệ thống

Hệ thống gồm hai khối chính:

Khối xử lý ảnh:
Camera Intel RealSense D435i
Jetson Nano
YOLOv8
TensorRT
Khối điều khiển:
ESP32
Driver BTS7960
Động cơ chuyển số
Động cơ lái

Luồng hoạt động
Camera
      │
      ▼
Jetson Nano
(Xử lý ảnh + YOLO)
      │ UART
      ▼
ESP32
      │
      ▼
Driver BTS7960
      │
      ▼
Motor DC
## Thành phần chính

- Jetson Nano: xử lý ảnh, nhận diện vật thể và xác định vùng nguy hiểm ROI.
- Camera RealSense D435i: thu nhận hình ảnh phía trước xe lu.
- ESP32: nhận lệnh từ Jetson Nano và điều khiển cơ cấu chấp hành.
- Driver BTS7960: điều khiển động cơ DC.
- Động cơ DC: điều khiển cơ cấu chuyển số và cơ cấu lái.

# Yêu cầu phần mềm

Jetson Nano

Ubuntu
Python 3
OpenCV
Ultralytics YOLOv8
TensorRT
PyTorch
PySerial

ESP32

Arduino IDE
ESP32 Board Package

## Cấu trúc mã nguồn

- `Jetson_Nano/`: chương trình xử lý ảnh AI và gửi lệnh RUN/STOP.
- `ESP32/`: chương trình điều khiển động cơ và nhận lệnh UART.
- `Images/`: hình ảnh minh họa hệ thống.

# Hướng dẫn chạy chương trình
Jetson Nano:
Kết nối camera RealSense D435i.
Cài đặt các thư viện cần thiết.
Chạy chương trình:
python detect.py
ESP32:
Mở Arduino IDE.
Chọn Board ESP32.
Nạp chương trình main.ino.
Kết nối UART với Jetson Nano.

## Nguyên lý hoạt động

Camera thu nhận hình ảnh phía trước xe.
Jetson Nano thực hiện nhận diện người và phương tiện bằng YOLOv8.
Khi đối tượng đi vào vùng ROI nguy hiểm, Jetson Nano gửi lệnh STOP đến ESP32.
ESP32 điều khiển driver BTS7960 đưa cần số về vị trí N.
Khi vùng ROI trở lại an toàn, Jetson Nano gửi lệnh RUN, ESP32 điều khiển đưa cần số sang vị trí tiến.

## Tác giả

Phạm Văn Lợi
