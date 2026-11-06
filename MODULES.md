# Cấu trúc module STM32

Đợt tách module này giữ nguyên chân GPIO, cấu hình timer/I2C, gain PI/PID,
chiều bánh, bộ lọc, ngưỡng hiệu chỉnh IMU, giao thức UART và timeout.

| Module | Trách nhiệm |
|---|---|
| `main.c` | Khởi tạo HAL, clock, GPIO, timer, I2C do CubeMX sinh; gọi ứng dụng |
| `robot_app.c/.h` | Ghép hai motor với phần cứng, chạy chu kỳ 10 ms, cập nhật biến debug, gửi telemetry |
| `motor_control.c/.h` | Đếm encoder có dấu và xử lý tràn, PID, chống tích phân bão hòa, PWM, khoảng nghỉ đổi chiều |
| `motor_config.h` | Chu kỳ điều khiển, count/vòng, chiều motor/encoder, kích thước bộ lọc |
| `speed_filter.c/.h` | Moving Average với bộ đệm riêng cho từng motor |
| `robot_control.c/.h` | Vận tốc v/w, giới hạn gia tốc, phản hồi gyro, trạng thái và thông số động lực học tham khảo |
| `robot_math.h` | Hình học robot, động học thuận/nghịch |
| `imu_mpu6050.c/.h` | Khởi tạo, đọc I2C, hiệu chỉnh gia tốc/gyro, lọc gyro, tích phân yaw |
| `telemetry_uart.c/.h` | USART1, RX ring buffer, CRC, lệnh vận tốc, timeout và telemetry |
| `stm32f1xx_it.c` | Điểm vào ngắt USART1, chuyển tiếp tới `Telemetry_RxIRQHandler()` |

Luồng chính: `main` → `RobotApp_Init()` một lần → `RobotApp_Process()` liên tục.
Ứng dụng phục vụ UART mỗi vòng lặp, rồi mỗi 10 ms chụp hai encoder,
gọi `Robot_Update()`, PID hai bánh và đóng gói telemetry.
`Robot_Update()` đọc IMU và dùng phản hồi RPM của chu kỳ trước, như bản gốc.

## Chỉnh thông số ở đâu

- Gain `Kp`, `KpFine`, `Ki`, `Kd`: `motor_control.c`.
- Count/vòng, dấu bánh trái/phải, cửa sổ lọc: `motor_config.h`.
- Chân motor và ghép timer: `robot_app.c`; cấu hình chân ngoại vi: `stm32f1xx_hal_msp.c`.
- Kích thước bánh, khoảng cách bánh, giới hạn v/w: `robot_math.h`.
- Bias/scale gia tốc, ngưỡng hiệu chỉnh: `imu_mpu6050.c`.
- Cấu hình TIM4/PWM vẫn nằm trong `main.c`/`.ioc`, chưa đổi tần số.
- UART PA9/PA10 do `telemetry_uart.c` khởi tạo trực tiếp; không khởi tạo thêm HAL USART1.

Các biến MCUViewer như `target_left_rpm`, `filtered_left_rpm`, `pwm_left`,
`Kp`, `imu_state`, `robot_enable` giữ nguyên tên. Sau khi build/nạp firmware,
cần import lại ELF hoặc Refresh addresses trong MCUViewer vì địa chỉ biến có thể thay đổi.

## Build và kiểm tra

Trong CubeIDE: Refresh (F5) project rồi Build Project. Project đã có source entry `Core`,
nên IDE tự nhận các file mới trong `Core/Src`. Các lời gọi ứng dụng và wrapper ngắt
nằm trong USER CODE để giữ lại khi CubeMX sinh code (bật giữ USER CODE).

Các file make đang có trong `Debug` cũng đã được cập nhật để build từ terminal;
CubeIDE có thể tạo lại chúng tự động.

Kiểm thử logic không truy cập phần cứng, dùng C++17 constexpr với HAL giả lập:

```text
python Tests/check_robot_control.py <đường-dẫn-arm-none-eabi-g++.exe>
python Tests/check_velocity_uart.py <đường-dẫn-arm-none-eabi-g++.exe>
python Tests/check_motor_control.py <đường-dẫn-arm-none-eabi-g++.exe>
```

Các kiểm thử này không thay thế chạy thử trên xe; lần tách module chưa tự nạp firmware.
