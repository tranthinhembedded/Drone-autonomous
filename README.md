# Drone-autonomous  
🚁 **Phát triển mô hình UAV tự hành theo làn đường sử dụng Raspberry Pi**

## 📌 Giới thiệu
Dự án **Drone-autonomous** là đồ án tốt nghiệp ngành **Công nghệ Kỹ thuật Máy tính**, tập trung vào việc **nghiên cứu, thiết kế và triển khai một mô hình UAV (quadcopter) có khả năng tự hành và bám theo làn đường** dựa trên thị giác máy tính.

Hồi quy chiếu hệ thống được xây dựng theo **kiến trúc lai (Hybrid System)**:
- **Raspberry Pi** đảm nhiệm xử lý ảnh, phát hiện làn đường và sinh lệnh điều hướng.
- **Vi điều khiển STM32** đảm nhiệm điều khiển bay thời gian thực, ổn định tư thế và điều khiển động cơ.

Dự án hướng tới việc chứng minh **tính khả thi của nền tảng tính toán nhúng chi phí thấp** trong các bài toán UAV tự hành dựa trên thị giác máy tính.

---

## 🎯 Mục tiêu
- Xây dựng mô hình UAV quadcopter hoàn chỉnh có khả năng bay tự hành
- Phát hiện và theo dõi làn đường bằng camera trong thời gian thực
- Điều khiển UAV bám theo tâm làn đường bằng bộ điều khiển PID
- Đánh giá hiệu năng hệ thống thông qua thực nghiệm

---

## 🧠 Kiến trúc hệ thống

### 1. Tổng quan
Hệ thống được chia thành hai tầng chính:

| Tầng | Thành phần | Chức năng |
|----|----|----|
| High-level | Raspberry Pi | Xử lý ảnh, phát hiện làn, tính sai lệch |
| Low-level | STM32 | Điều khiển PID, ổn định bay, điều khiển động cơ |

---

### 2. Phần cứng chính
- **Raspberry Pi 5**
- **STM32F401CEU6 (Flight Controller)**
- **IMU MPU6050**
- **La bàn HMC5883L**
- **Cảm biến áp suất BMP280**
- **Camera USB / Pi Camera**
- **ESC SkyWalker**
- **Động cơ BLDC**
- **Pin LiPo**

---

## 👁️ Xử lý ảnh & phát hiện làn đường
Thuật toán xử lý ảnh được triển khai trên Raspberry Pi bằng **Python + OpenCV**, gồm các bước:

1. Thu nhận ảnh từ camera
2. Trích xuất vùng quan tâm (ROI)
3. Biến đổi phối cảnh (Inverse Perspective Mapping)
4. Lọc nhiễu & phân ngưỡng
5. Phát hiện biên (Canny)
6. Phát hiện đường thẳng (Hough / Sliding Window)
7. Xác định:
   - Sai lệch vị trí ngang (offset)
   - Sai lệch góc hướng (heading error)

Kết quả được gửi sang STM32 để điều khiển bay.

---

## 🎮 Điều khiển UAV
Hệ thống điều khiển bay sử dụng **PID Cascade (2 vòng)**:

### 🔹 Vòng ngoài – Angle Loop
- Điều khiển góc roll, pitch, yaw
- Chuyển sai lệch vị trí → góc mong muốn

### 🔹 Vòng trong – Rate Loop
- Điều khiển vận tốc góc
- Tạo tín hiệu PWM cho ESC

Kết hợp với:
- **Low-pass filter**
- **Complementary Filter** để giảm nhiễu IMU

---

## 🧪 Thực nghiệm & đánh giá
Thực nghiệm được thực hiện với các kịch bản:
- Làn đường thẳng
- Làn đường cong chữ S
- Điều kiện ánh sáng khác nhau

### Tiêu chí đánh giá:
- Độ lệch so với tâm làn
- Độ ổn định quỹ đạo bay
- Khả năng bám làn liên tục
- Tốc độ xử lý ảnh (FPS)

Kết quả cho thấy UAV có khả năng:
- Bám làn ổn định
- Giữ tư thế bay tốt
- Hoạt động thời gian thực trong điều kiện thử nghiệm

---

## 📂 Cấu trúc thư mục
```text
Drone-autonomous/
│
├── Flight_Controller_STM32F401CEU6/   # Firmware STM32 (PID, IMU, PWM)
├── Python_FC/                         # Code Flight Controller Python on raspberry pi 
├── README.md
