# Hệ thống hỗ trợ điều khiển an toàn cho xe lu sử dụng AI

Repository này lưu trữ mã nguồn của hệ thống hỗ trợ điều khiển an toàn cho xe lu sử dụng Jetson Nano, ESP32 và xử lý ảnh bằng trí tuệ nhân tạo.

## Thành phần chính

- Jetson Nano: xử lý ảnh, nhận diện vật thể và xác định vùng nguy hiểm ROI.
- Camera RealSense D435i: thu nhận hình ảnh phía trước xe lu.
- ESP32: nhận lệnh từ Jetson Nano và điều khiển cơ cấu chấp hành.
- Driver BTS7960: điều khiển động cơ DC.
- Động cơ DC: điều khiển cơ cấu chuyển số và cơ cấu lái.

## Cấu trúc mã nguồn

- `Jetson_Nano/`: chương trình xử lý ảnh AI và gửi lệnh RUN/STOP.
- `ESP32/`: chương trình điều khiển động cơ và nhận lệnh UART.
- `Images/`: hình ảnh minh họa hệ thống.

## Nguyên lý hoạt động

Camera thu nhận hình ảnh phía trước xe lu. Jetson Nano xử lý ảnh, phát hiện đối tượng nguy hiểm trong vùng ROI. Khi phát hiện vật cản nguy hiểm, Jetson gửi lệnh STOP đến ESP32. Khi vùng ROI an toàn trở lại, Jetson gửi lệnh RUN để xe tiếp tục hoạt động.

## Tác giả

Phạm Văn Lợi
