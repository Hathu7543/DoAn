# STM32 + MPU6050: điều khiển vận tốc xe hai bánh vi sai

Đã cấu hình: bánh đường kính 65 mm, khoảng cách tâm hai bánh 215 mm,
khối lượng khoảng 2 kg, encoder TI12 = 1320 count/vòng bánh.
Hai bánh phụ xoay hướng tự do. PID hai motor và bộ lọc encoder được giữ lại.
Không dùng ESP, Bluetooth hoặc UART điều khiển trong bản này.

## Nối GY-521 vào STM32F103C8

| GY-521 | STM32 | Ghi chú |
|---|---|---|
| VCC | 3V3 | Cấp module từ nguồn 3,3 V ổn định |
| GND | GND | Chung mass STM32 và driver |
| SCL | PB10 | I2C2, 100 kHz |
| SDA | PB11 | I2C2, 100 kHz |
| AD0 | GND | Địa chỉ 7 bit 0x68 |
| INT, XDA, XCL | Không nối | Code polling cờ data-ready qua I2C |

SDA/SCL cần điện trở kéo lên 3,3 V; module thường có sẵn. Nếu module không có,
dùng khoảng 4,7 kOhm mỗi đường. Không kéo SDA/SCL lên nguồn motor 12 V hoặc 5 V.
PB6/PB7 vẫn dành cho PWM L298N, không dùng làm I2C1.

Giữ dây motor: trái ENA=PB6, IN1=PA1, IN2=PA2, encoder A/B=PA6/PA7;
phải ENB=PB7, IN3=PB0, IN4=PB1, encoder A/B=PA15/PB3.
Tháo jumper ENA/ENB khi dùng PWM.

Gắn MPU nằm ngang, chắc chắn, gần giữa hai bánh chủ động; +X hướng tiến,
+Y sang trái, +Z hướng lên. Hướng chip/module phải được kiểm tra thực tế.
Xoay thân xe sang trái khi motor tắt: `gyro_w_radps` phải dương.
Nếu hướng này sai, sửa cách lắp/ánh xạ trục trước khi bật phản hồi gyro.

## Khởi động và chạy thử

Đã thêm hiệu chỉnh gia tốc từ số đo sáu tư thế do người dùng cung cấp:

| Trục | Số đo + / - (m/s²) | Bias (m/s²) | Scale |
|---|---|---:|---:|
| X | 9,8 / -9,5 | 0,15 | 1,016233 |
| Y | 9,9 / -9,7 | 0,10 | 1,000679 |
| Z | 11,6 / -8,0 | 1,80 | 1,000679 |

Công thức: corrected = (uncalibrated - bias) * scale. Hệ số áp dụng một lần,
trước kiểm tra tư thế/độ yên để hiệu chuẩn gyro; không nới các ngưỡng kiểm tra.
`accel_x/y/z_mps2` hiện là giá trị đã hiệu chỉnh, vẫn có trọng lực.
`accel_uncal_x/y/z_mps2` giữ giá trị trước hiệu chỉnh (m/s², không phải ADC counts).
Nếu đo lại sáu tư thế, dùng các biến `accel_uncal_*` để tránh bù hai lần.
Các số đo đầu vào được làm tròn, nên cần xác nhận trên module thật sau khi nạp.
Hệ số này không thay thế hiệu chuẩn bias gyro lúc khởi động.

1. Refresh project trong CubeIDE, build/nạp `Debug/Do_An.elf`.
2. Giữ xe nằm ngang và đứng yên khoảng 3–4 giây. Không chạm vào xe khi hiệu chuẩn.
   `imu_calibration_samples` cần đạt 300; xe rung/chuyển động sẽ làm đếm lại.
3. Kiểm tra `imu_state=2`, `robot_fault=0`, `robot_enable=0`.
   Đứng yên, `gyro_w_radps` gần 0 và `accel_z_mps2` gần +9,81.
4. Đặt lệnh trong CubeIDE Live Expressions/debugger có hỗ trợ ghi biến.
   Có thể theo dõi bằng MCUViewer; import/refresh địa chỉ từ ELF mới.
5. Ghi `robot_enable=1` **sau cùng** để chạy; ghi `robot_enable=0` để dừng PWM.

`robot_enable`, `imu_state`, `robot_fault`, `imu_calibration_samples` là uint32_t.
Các vận tốc và hệ số là float. Không thay đổi `target_left_rpm/target_right_rpm`
trực tiếp: chúng được tính lại từ lệnh thân xe mỗi chu kỳ.

| Thử nghiệm | command_v_mps | command_w_radps |
|---|---:|---:|
| Tiến | 0.1 | 0 |
| Lùi | -0.1 | 0 |
| Quay trái tại chỗ | 0 | 0.5 |
| Quay phải tại chỗ | 0 | -0.5 |
| Dừng | 0 | 0 |

Trước khi đổi thử nghiệm, đặt `robot_enable=0`, ghi cả v và w, rồi bật lại.
Không mặc định bật cả 0,1 m/s và 0,5 rad/s: khi bật đồng thời xe sẽ đi theo cung tròn.
Lệnh bị giới hạn ở +/-0,1 m/s và +/-0,5 rad/s cho giai đoạn thử này.
Profile tăng/giảm tốc giới hạn 0,15 m/s² và 0,8 rad/s²; riêng dừng cắt PWM ngay.
Xe vẫn có quán tính sau khi ngắt PWM, không có chức năng phanh giữ vị trí.

Lần đầu kê bánh lên, đặt `yaw_feedback_enable=0` để kiểm tra riêng dấu motor/encoder.
Sau đó đặt xe xuống sàn, xác nhận dấu gyro, bật `yaw_feedback_enable=1` để thử quay.
Khi kê bánh, thân xe không quay dù encoder có quay; không dùng trường hợp đó
để tuning phản hồi tốc độ quay của thân xe.

## Các biến cần quan sát

| Biến | Ý nghĩa |
|---|---|
| command_v_mps, command_w_radps | Lệnh của người dùng, m/s và rad/s |
| reference_v_mps, reference_w_radps | Lệnh sau giới hạn và profile gia tốc |
| target_left_rpm, target_right_rpm | RPM đặt vào PID hai bánh |
| measured_v_mps | Vận tốc tiến suy ra từ encoder, không bù trượt bánh |
| encoder_w_radps | Tốc độ quay suy ra từ encoder |
| gyro_w_radps | Tốc độ quay thân xe từ gyro Z, đã trừ bias và lọc |
| yaw_correction_radps | Bù tốc độ quay có giới hạn +/-0,15 rad/s |
| yaw_relative_rad | Góc tương đối tích phân gyro, có trôi; không phải hướng tuyệt đối |
| accel_x/y/z_mps2 | Gia tốc kế theo trục cảm biến, còn thành phần trọng lực |
| imu_state | 0 khởi tạo, 1 hiệu chuẩn, 2 sẵn sàng, 3 lỗi |
| robot_fault | 0 bình thường, 1 lỗi IMU, 2 chu kỳ >50 ms, 3 lệnh/gain không hợp lệ |

Nếu cảm biến không nối, đọc lỗi hoặc dữ liệu không cập nhật trong 100 ms,
code dừng hai bánh và khóa chạy. Sửa dây/nguồn rồi Reset STM32 để khởi tạo và
hiệu chuẩn lại; không ép `imu_state` hay xóa `robot_fault` để bỏ qua lỗi.
Dừng lâu tại breakpoint có thể gây lỗi thời gian sau Resume; reset để chạy lại.

## Mô hình và giới hạn

Với v (m/s), w (rad/s), r=0,0325 m, L=0,215 m:

    RPM_L = (v - w*L/2) * 60 / (2*pi*r)
    RPM_R = (v + w*L/2) * 60 / (2*pi*r)

Tiến 0,1 m/s: khoảng +29,38 RPM mỗi bánh.
Quay trái 0,5 rad/s: trái -15,79 RPM, phải +15,79 RPM trước bù gyro.
Giới hạn bánh 60 RPM, co đồng thời hai giá trị để giữ tỷ lệ nếu vượt giới hạn.

Vòng ngoài dùng P nhỏ `yaw_kp=0.3` hiệu chỉnh sai số tốc độ quay;
PID tốc độ bánh vẫn là vòng trong. Đây là gain khởi đầu cần thử trên xe thật.
Gia tốc kế hiện dùng để kiểm tra tư thế/độ yên khi hiệu chuẩn và để quan sát,
không tích phân thành vận tốc tiến. Chưa có bộ lọc hợp nhất trạng thái hoặc
bộ điều khiển giữ góc/đi theo quỹ đạo.

Động lực học lý tưởng:

    F = m * a
    Mz = Iz * alpha
    torque_left  = r * (F/2 - Mz/L)
    torque_right = r * (F/2 + Mz/L)

`force_reference_n` dùng khối lượng 2 kg và gia tốc đặt.
`body_inertia_kgm2=0` nghĩa là **chưa xác định**, nên `dynamics_valid=0` và các
mô-men chưa được tính. Chỉ nhập Iz sau khi đo/ước lượng từ phân bố khối lượng thực.
Khi Iz hợp lệ, code xuất mô-men lý tưởng để quan sát; không bù PWM theo mô-men.
Chưa bao gồm quán tính bánh/hộp số, ma sát, điện áp pin và đặc tính motor.
Cần nhận dạng các thông số này trước khi thêm feedforward động lực học.

MPU được cấu hình gyro +/-250 độ/s, accel +/-2 g, đọc dữ liệu mới ở 100 Hz.
Tham khảo [đặc tính MPU6050 của TDK](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf).

Đã build firmware và kiểm thử logic với I2C/thời gian mô phỏng (động học, hiệu
chuẩn, dừng, lỗi cảm biến). Chưa nạp hoặc kiểm nghiệm với cảm biến/xe thật.
Bản PID cũ: `C:\Users\hathu\Downloads\PID_Control\imu_backup_20261022`.
