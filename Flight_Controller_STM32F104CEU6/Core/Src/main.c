
/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : FLIGHT CONTROLLER - RC RECEIVER READY
  * Updated         : iNav Style Support & Physical RC Setup
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "Filter.h"
#include "PID.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Cal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    HOVER,      // Chế độ Angle (Tự cân bằng)
    RATE_MODE,  // Chế độ Rate (Bay nhào lộn / Khóa góc)
} MPC_Status_t;

typedef enum {
    ARM,
    NOT_ARM
} ARM_Status_t;

IMU_RAW_DATA_t MPU6500_RAW_DATA;
IMU_Data_t MPU6500_DATA;

MAG_RAW_DATA_t HMC5883L_RAW_DATA;
MAG_DATA_t HMC5883L_DATA;

MagCal_Simple_t MagCal = {
    .S = 1.0f,
    .state = MAG_CAL_DONE,
    .samples_target = 5000,
	.offset = {4.7840004f, -3.45000076f, 7.36000061f},
	.scale  = {0.937098265f, 0.954184234f, 1.13012183f}

};

Complimentary_Filter_t Complimentary_Filter = {
    .alpha[0] = 0.99, .alpha[1] = 0.99, .alpha[2] = 0.96,
};

/* ===================== PID CONFIGURATION ===================== */
// LƯU Ý: Feed Forward đang để 0.05 (Mức thấp an toàn cho iNav style)
// Sau khi bay ổn, hãy tăng dần lên 0.1 -> 0.2 để bay "dính tay" hơn.

// RATE LOOP (Inner Loop)
PID_ALTIDUE_t PID_RATE_ROLL = {
    .alpha_lpf = 0.88, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 0.700993, .ki = 1.265000, .kd = 0.112501, .d_limit = 22.5,
};
PID_ALTIDUE_t PID_RATE_PITCH = {
    .alpha_lpf = 0.88, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 0.700993, .ki = 1.265000, .kd = 0.112501, .d_limit = 22.5,
};
PID_ALTIDUE_t PID_RATE_YAW = {
    .alpha_lpf = 0.88, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 1.456010, .ki = 1.401011, .kd = 0.138702, .d_limit = 22.5,
};

// ANGLE LOOP (Outer Loop)
PID_ALTIDUE_t PID_ROLL = {
    .alpha_lpf = 0.88, .feed_forward = 0, .i_limit = 65, .max_output = 150,
    .kp = 1.7, .ki = 0.007, .kd = 1.8, .d_limit = 22.5,
};
PID_ALTIDUE_t PID_PITCH = {
    .alpha_lpf = 0.88, .feed_forward = 0, .i_limit = 65, .max_output = 150,
    .kp = 1.7, .ki = 0.007, .kd = 1.8, .d_limit = 22.5,
};
PID_ALTIDUE_t PID_YAW = {
    .alpha_lpf = 0.88, .feed_forward = 0, .i_limit = 60, .max_output = 120,
    .kp = 1.5, .ki = 0.001, .kd = 0.0, .d_limit = 22.5,
};

MPC_Status_t MPC_Status = HOVER;
ARM_Status_t ARM_Status = NOT_ARM;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_ARM 1240 // Mức ga tối thiểu để motor quay
#define RX_DMA_SIZE    256
#define CMD_LINE_SIZE  128
#define HMC5883L_ADDR      (0x1E << 1)
#define HMC5883L_SCALE_GAUSS 0.00092f
#define HMC5883L_SCALE_UT    (HMC5883L_SCALE_GAUSS * 100.0f)
#define MPU6050_ADDR         (0x68 << 1)
#define GYRO_SENSITIVITY     65.5f
#define ACCEL_SENSITIVITY    4096.0f
#define GRAVITY_EARTH        9.80665f
#define MAG_LPF_ALPHA   0.08f
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int16_t raw_acc[3];
int16_t raw_gyro[3];
float32_t acc_phys[3], gyro_phys[3];
uint32_t current_time, prev_time, dt;
uint8_t enable_motor = 0;

// Variables for RC Control
float32_t Throttle = 1000.0f;
float32_t Moment[3];
float32_t angle_desired[3] = {0, 0, 0};
float32_t angle_rate_desired[3] = {0, 0, 0};

// Motor Output
float32_t PWM_MOTOR[4];
uint32_t PWM_TIMER[4];

// Telemetry
float vbat = 11.1f; // Bạn cần đọc ADC pin thực tế để cập nhật biến này
uint32_t last_telemetry_time = 0;

// Filters
float32_t gyro_final[3] = {0.0f, 0.0f, 0.0f};
float32_t acc_filtered[3] = {0.0f, 0.0f, 0.0f};
const float32_t alpha_acc_soft = 0.20f;
const float32_t alpha_gyro = 0.35f;
float32_t mag_filtered[3] = {0.0f, 0.0f, 0.0f};
uint8_t mag_lpf_inited = 0;

// Calibration
float32_t gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float32_t accel_bias[3] = {0.0f, 0.0f, 0.0f};
uint8_t is_calibrated = 0;

// UART Buffers (For PID Tuning Only)
static uint8_t rx_dma_buf[RX_DMA_SIZE];
static char cmd_work[CMD_LINE_SIZE];
static uint16_t cmd_len = 0;
static char cmd_ready[CMD_LINE_SIZE];
static volatile uint8_t line_ready = 0;

// --- BIẾN ĐỌC RC THÔ (RAW PWM) ---
// Đơn vị: micro giây (us). Giá trị chuẩn: 1000 - 2000
volatile uint32_t RC_Raw_Roll     = 0; // TIM5 CH1
volatile uint32_t RC_Raw_Pitch    = 0; // TIM5 CH2
volatile uint32_t RC_Raw_Throttle = 0; // TIM5 CH3
volatile uint32_t RC_Raw_Yaw      = 0; // TIM5 CH4
volatile uint32_t RC_Raw_SW_Arm   = 0; // TIM1 CH1
volatile uint32_t RC_Raw_SW_Mode  = 0; // TIM1 CH4

// Biến phụ trợ để tính toán thời gian
uint32_t val_start_roll = 0, val_start_pitch = 0, val_start_thr = 0, val_start_yaw = 0;
uint32_t val_start_arm = 0, val_start_mode = 0;

static MPC_Status_t last_MPC_Status = HOVER;
static uint8_t reset_pid_request = 0; // Cờ yêu cầu đồng bộ PID

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void IMU_PROCESS(void);
void COMPASS_PROCESS(void);
void MIX_THROTTLE(float32_t thr, float32_t* moment, float32_t* m);
void MPC(void);
void Control_Motor(void);
void RESET_ALL_PID(void);
void Send_Telemetry(void);
static void UART1_StartRxToIdle_DMA(void);
static void ProcessLine(char *line);
void MPU6050_Init(void);
void HMC5883L_Init(void);
void MPU6050_Calibrate(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- UART Logic --- */
static void UART1_StartRxToIdle_DMA(void) {
    HAL_UART_DMAStop(&huart1);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE) != HAL_OK) {
        HAL_UART_DeInit(&huart1);
        HAL_UART_Init(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE);
    }
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

// Xử lý lệnh Tune PID từ GUI (Không còn lệnh điều khiển bay)
// Xử lý lệnh Tune PID từ GUI
static void ProcessLine(char *line) {
    size_t n = strlen(line);
    while (n > 0 && (line[n-1] == '\r' || line[n-1] == '\n')) {
        line[n-1] = 0; n--;
    }
    if (n == 0) return;

    char *tok = strtok(line, ":");
    if (!tok) return;

    if (strcmp(tok, "PID") == 0) {
        char *axis = strtok(NULL, ":");
        char *s_kp = strtok(NULL, ":");
        char *s_ki = strtok(NULL, ":");
        char *s_kd = strtok(NULL, ":");

        if (axis && s_kp && s_ki && s_kd) {
            float p = strtof(s_kp, NULL);
            float i = strtof(s_ki, NULL);
            float d = strtof(s_kd, NULL);

            // 1. TUNE ANGLE LOOP (ROLL & PITCH)
            if (strcmp(axis, "ANG") == 0) {
                PID_ROLL.kp = p; PID_ROLL.ki = i; PID_ROLL.kd = d;
                PID_PITCH.kp = p; PID_PITCH.ki = i; PID_PITCH.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // 2. TUNE YAW ANGLE LOOP (OUTER - MỚI)
            else if (strcmp(axis, "YANG") == 0) {
                PID_YAW.kp = p; PID_YAW.ki = i; PID_YAW.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // 3. TUNE YAW RATE LOOP (INNER - CŨ)
            else if (strcmp(axis, "YAW") == 0) {
                PID_RATE_YAW.kp = p; PID_RATE_YAW.ki = i; PID_RATE_YAW.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // ACK
            if (huart1.gState == HAL_UART_STATE_READY) {
                char ack[64];
                int len = snprintf(ack, sizeof(ack), "MSG:UPDATED %s P=%.3f\n", axis, p);
                if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t*)ack, (uint16_t)len, 10);
            }
        }
    }
}

void Send_Telemetry(void) {
    if (huart1.gState != HAL_UART_STATE_READY) return;
    static char tx_buf[128];
    // Gửi dữ liệu góc để vẽ đồ thị
    int len = sprintf(tx_buf, "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"v\":%.1f}\n",
            Complimentary_Filter.Euler_Angle_Deg[0],
            Complimentary_Filter.Euler_Angle_Deg[1],
            Complimentary_Filter.Euler_Angle_Deg[2],
            vbat);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buf, len);
}

/* --- IMU Functions --- */
void MPU6050_Init(void) {
    uint8_t data;
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
    data = 0x06; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);
    data = 0x08; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
    data = 0x10; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);
    uint8_t bypass_en = 0x02; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x37, 1, &bypass_en, 1, 100);
}

void MPU6050_Calibrate(void) {
    uint8_t buffer[14];
    int32_t sum_acc[3] = {0}, sum_gyro[3] = {0};
    int16_t ra_acc[3], ra_gyro[3];
    int sample_count = 1000;

    for (int i = 0; i < sample_count; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
            ra_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
            ra_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
            ra_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
            ra_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
            ra_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
            ra_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

            sum_acc[0] += ra_acc[0]; sum_acc[1] += ra_acc[1]; sum_acc[2] += ra_acc[2];
            sum_gyro[0] += ra_gyro[0]; sum_gyro[1] += ra_gyro[1]; sum_gyro[2] += ra_gyro[2];
        }
        HAL_Delay(2);
    }
    gyro_bias[0] = (float)sum_gyro[0] / sample_count;
    gyro_bias[1] = (float)sum_gyro[1] / sample_count;
    gyro_bias[2] = (float)sum_gyro[2] / sample_count;
    accel_bias[0] = (float)sum_acc[0] / sample_count;
    accel_bias[1] = (float)sum_acc[1] / sample_count;
    accel_bias[2] = ((float)sum_acc[2] / sample_count) - 4096.0f;
    is_calibrated = 1;
}

void HMC5883L_Init(void) {
    uint8_t data;
    data = 0x70; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    data = 0x20; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    HAL_Delay(10);
}

void IMU_PROCESS(void) {
    uint8_t buffer[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
        raw_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
        raw_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
        raw_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
        raw_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

        float32_t acc_temp[3], gyro_temp[3];
        if (is_calibrated) {
            acc_temp[0] = (float32_t)raw_acc[0] - accel_bias[0];
            acc_temp[1] = (float32_t)raw_acc[1] - accel_bias[1];
            acc_temp[2] = (float32_t)raw_acc[2] - accel_bias[2];
            gyro_temp[0] = (float32_t)raw_gyro[0] - gyro_bias[0];
            gyro_temp[1] = (float32_t)raw_gyro[1] - gyro_bias[1];
            gyro_temp[2] = (float32_t)raw_gyro[2] - gyro_bias[2];
        } else {
            acc_temp[0] = raw_acc[0]; acc_temp[1] = raw_acc[1]; acc_temp[2] = raw_acc[2];
            gyro_temp[0] = raw_gyro[0]; gyro_temp[1] = raw_gyro[1]; gyro_temp[2] = raw_gyro[2];
        }

        acc_phys[0] = (acc_temp[0] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[1] = -(acc_temp[1] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[2] = -(acc_temp[2] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        gyro_phys[0] = (gyro_temp[0] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[1] = -(gyro_temp[1] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[2] = -(gyro_temp[2] / GYRO_SENSITIVITY) * DEG_TO_RAD;

        // Accel LPF
        static uint8_t acc_lpf_inited = 0;
        if (!acc_lpf_inited) {
            acc_filtered[0] = acc_phys[0]; acc_filtered[1] = acc_phys[1]; acc_filtered[2] = acc_phys[2];
            acc_lpf_inited = 1;
        } else {
            for (int i = 0; i < 3; i++) acc_filtered[i] += alpha_acc_soft * (acc_phys[i] - acc_filtered[i]);
        }

        // Gyro LPF
        static uint8_t gyro_lpf_inited = 0;
        if (!gyro_lpf_inited) {
            gyro_final[0] = gyro_phys[0]; gyro_final[1] = gyro_phys[1]; gyro_final[2] = gyro_phys[2];
            gyro_lpf_inited = 1;
        } else {
            for (int i = 0; i < 3; i++) gyro_final[i] += alpha_gyro * (gyro_phys[i] - gyro_final[i]);
        }

        MPU6500_DATA.acc[0] = acc_filtered[0]; MPU6500_DATA.acc[1] = acc_filtered[1]; MPU6500_DATA.acc[2] = acc_filtered[2];
        MPU6500_DATA.w[0] = gyro_final[0]; MPU6500_DATA.w[1] = gyro_final[1]; MPU6500_DATA.w[2] = gyro_final[2];
    }
}

void COMPASS_PROCESS(void) {
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, 100) == HAL_OK) {
        raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_z = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_y = (int16_t)(buffer[4] << 8 | buffer[5]);

        HMC5883L_RAW_DATA.mag[0] = -(float32_t)raw_x * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[1] = (float32_t)raw_y * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[2] = -(float32_t)raw_z * HMC5883L_SCALE_UT;

        MagCal_Update(&MagCal, &HMC5883L_RAW_DATA, &HMC5883L_DATA);

        if (MagCal.state == MAG_CAL_DONE) {
            if (!mag_lpf_inited) {
                mag_filtered[0] = HMC5883L_DATA.mag_uT[0];
                mag_filtered[1] = HMC5883L_DATA.mag_uT[1];
                mag_filtered[2] = HMC5883L_DATA.mag_uT[2];
                mag_lpf_inited = 1;
            } else {
                mag_filtered[0] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[0] - mag_filtered[0]);
                mag_filtered[1] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[1] - mag_filtered[1]);
                mag_filtered[2] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[2] - mag_filtered[2]);
            }
            HMC5883L_DATA.mag_uT[0] = mag_filtered[0];
            HMC5883L_DATA.mag_uT[1] = mag_filtered[1];
            HMC5883L_DATA.mag_uT[2] = mag_filtered[2];
        }
    }
}
// =============================================================================
// MPC CONTROL LOOP (MAIN FLIGHT LOGIC)
// =============================================================================
void MPC(void) {
    float32_t feedback[3];
    float32_t real_dt = MPU6500_DATA.dt;

    if (real_dt > 0.01f) real_dt = 0.01f;
    if (real_dt < 0.001f) real_dt = 0.004f;

    if (RC_Raw_Throttle < 950) {
        if (ARM_Status == ARM) {
            MPC_Status = HOVER;
            angle_desired[0] = 0.0f;
            angle_desired[1] = 0.0f;
            Throttle -= 0.2f;
            if (Throttle < 1100.0f) {
                ARM_Status = NOT_ARM; enable_motor = 0; Throttle = 1000.0f;
            }
        }
    }
    else {
        // --- NORMAL CONTROL ---
        if (RC_Raw_Throttle > 2000) Throttle = 2000.0f;
        else if (RC_Raw_Throttle < 1000) Throttle = 1000.0f;
        else Throttle = (float32_t)RC_Raw_Throttle;

        // Check Mode Change
        MPC_Status_t current_Mode = (RC_Raw_SW_Mode > 1500) ? HOVER : RATE_MODE;
        if (current_Mode != last_MPC_Status) {
            MPC_Status = current_Mode;
            last_MPC_Status = current_Mode;
            reset_pid_request = 1; // Yêu cầu đồng bộ PID
        }

        // Check Arming
        if (RC_Raw_SW_Arm > 1500) {
            if (ARM_Status == NOT_ARM && Throttle < 1150) {
                ARM_Status = ARM; enable_motor = 1;
                RESET_ALL_PID();
                // Reset góc mong muốn về góc hiện tại
                angle_desired[0] = 0;
                angle_desired[1] = 0;
                angle_desired[2] = Complimentary_Filter.Euler_Angle_Deg[2];
                reset_pid_request = 1;
            }
        } else {
            ARM_Status = NOT_ARM; enable_motor = 0;
        }

        // Stick Mapping & Deadband
        float32_t stick_roll  = (float32_t)RC_Raw_Roll - 1500.0f;
        float32_t stick_pitch = (float32_t)RC_Raw_Pitch - 1500.0f;
        float32_t stick_yaw   = (float32_t)RC_Raw_Yaw - 1500.0f;

        if (fabsf(stick_yaw) < 15.0f) stick_yaw = 0.0f;
        if (fabsf(stick_roll) < 5.0f) stick_roll = 0.0f;
        if (fabsf(stick_pitch) < 5.0f) stick_pitch = 0.0f;

        if (MPC_Status == HOVER) {
            angle_desired[0] = stick_roll * 0.06f;
            angle_desired[1] = -stick_pitch * 0.06f;

            float yaw_speed = 150.0f;
            float angle_step = (stick_yaw / 500.0f) * yaw_speed * real_dt;
            angle_desired[2] += angle_step;

            if (angle_desired[2] > 180.0f) angle_desired[2] -= 360.0f;
            if (angle_desired[2] < -180.0f) angle_desired[2] += 360.0f;
        } else {
            angle_rate_desired[0] = stick_roll * 0.20f;
            angle_rate_desired[1] = -stick_pitch * 0.20f;
            angle_rate_desired[2] = stick_yaw * 0.20f;
        }
    }

    if (ARM_Status == ARM) {
        if (Throttle < 1500.0f) {
            PID_ROLL.integral = 0; PID_PITCH.integral = 0; PID_YAW.integral = 0;
            PID_RATE_ROLL.integral = 0; PID_RATE_PITCH.integral = 0; PID_RATE_YAW.integral = 0;
        }
        switch (MPC_Status) {
            case RATE_MODE:
                // --- RATE MODE: SYNC TRƯỚC PID ---
                if (reset_pid_request) {
                    PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                    PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                    PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                    reset_pid_request = 0;
                }

                feedback[0] = MPU6500_DATA.w[0] * RAD_TO_DEG;
                feedback[1] = MPU6500_DATA.w[1] * RAD_TO_DEG;
                feedback[2] = MPU6500_DATA.w[2] * RAD_TO_DEG;

                Caculate_PID_Rate_ALTIDUE(&PID_RATE_ROLL, angle_rate_desired[0], feedback[0], real_dt);
                Caculate_PID_Rate_ALTIDUE(&PID_RATE_PITCH, angle_rate_desired[1], feedback[1], real_dt);
                Caculate_PID_Rate_ALTIDUE(&PID_RATE_YAW, angle_rate_desired[2], feedback[2], real_dt);

                Moment[0] = PID_RATE_ROLL.output;
                Moment[1] = PID_RATE_PITCH.output;
                Moment[2] = PID_RATE_YAW.output;

                MIX_THROTTLE(Throttle, Moment, PWM_MOTOR);
                Control_Motor();
                break;

            case HOVER:

                // 1. Angle PID
                feedback[0] = Complimentary_Filter.Euler_Angle_Deg[0];
                feedback[1] = Complimentary_Filter.Euler_Angle_Deg[1];
                feedback[2] = Complimentary_Filter.Euler_Angle_Deg[2];

                Caculate_PID_ALTIDUE(&PID_ROLL, angle_desired[0], feedback[0], real_dt);
                Caculate_PID_ALTIDUE(&PID_PITCH, angle_desired[1], feedback[1], real_dt);
                Caculate_PID_ALTIDUE(&PID_YAW, angle_desired[2], feedback[2], real_dt);

                angle_rate_desired[0] = PID_ROLL.output;
                angle_rate_desired[1] = PID_PITCH.output;
                angle_rate_desired[2] = PID_YAW.output;

                if (reset_pid_request) {
                    PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                    PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                    PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                    reset_pid_request = 0;
                }

                // 3. Rate PID
                feedback[0] = MPU6500_DATA.w[0] * RAD_TO_DEG;
                feedback[1] = MPU6500_DATA.w[1] * RAD_TO_DEG;
                feedback[2] = MPU6500_DATA.w[2] * RAD_TO_DEG;

                Caculate_PID_Rate_ALTIDUE(&PID_RATE_ROLL, angle_rate_desired[0], feedback[0], real_dt);
                Caculate_PID_Rate_ALTIDUE(&PID_RATE_PITCH, angle_rate_desired[1], feedback[1], real_dt);
                Caculate_PID_Rate_ALTIDUE(&PID_RATE_YAW, angle_rate_desired[2], feedback[2], real_dt);

                Moment[0] = PID_RATE_ROLL.output;
                Moment[1] = PID_RATE_PITCH.output;
                Moment[2] = PID_RATE_YAW.output;

                MIX_THROTTLE(Throttle, Moment, PWM_MOTOR);
                Control_Motor();
                break;
        }
    } else {
        for (int i = 0; i < 4; i++) { PWM_MOTOR[i] = 1000; PWM_TIMER[i] = 1000; }
        Control_Motor();
        RESET_ALL_PID();

        angle_rate_desired[0] = 0; angle_rate_desired[1] = 0; angle_rate_desired[2] = 0;
        angle_desired[0] = 0;
        angle_desired[1] = 0;
        angle_desired[2] = Complimentary_Filter.Euler_Angle_Deg[2];
    }
}
void RESET_ALL_PID(void) {
    Reset_PID_ALTIDUE(&PID_RATE_ROLL); Reset_PID_ALTIDUE(&PID_RATE_PITCH); Reset_PID_ALTIDUE(&PID_RATE_YAW);
    Reset_PID_ALTIDUE(&PID_ROLL); Reset_PID_ALTIDUE(&PID_PITCH); Reset_PID_ALTIDUE(&PID_YAW);
}

void MIX_THROTTLE(float32_t thr, float32_t* moment, float32_t* m) {
    m[0] = thr - moment[0] + moment[1] + moment[2]; // Motor 1
    m[1] = thr - moment[0] - moment[1] - moment[2]; // Motor 2
    m[2] = thr + moment[0] - moment[1] + moment[2]; // Motor 3
    m[3] = thr + moment[0] + moment[1] - moment[2]; // Motor 4

    for (int i = 0; i < 4; i++) {
        if (m[i] > 1850) m[i] = 1850;
        if (m[i] < MIN_ARM) m[i] = MIN_ARM;
    }
}
void Control_Motor(void) {
    if (enable_motor) {
        for (int i = 0; i < 4; i++) PWM_TIMER[i] = (uint32_t)PWM_MOTOR[i];
        TIM3->CCR1 = PWM_TIMER[0]; TIM3->CCR2 = PWM_TIMER[1];
        TIM4->CCR1 = PWM_TIMER[2]; TIM4->CCR2 = PWM_TIMER[3];
    } else {
        TIM3->CCR1 = 1000; TIM3->CCR2 = 1000; TIM4->CCR1 = 1000; TIM4->CCR2 = 1000;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); // Roll
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // Pitch
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); // Throttle
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); // Yaw

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // SW Arm
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); // SW Mode

  TIM3->CCR1 = 1000; TIM3->CCR2 = 1000; TIM4->CCR1 = 1000; TIM4->CCR2 = 1000;

  MPU6050_Init(); HAL_Delay(50);
  HMC5883L_Init(); HAL_Delay(500);
  MPU6050_Calibrate();

  RESET_ALL_PID();
  current_time = TIM2->CNT;
  enable_motor = 0;
  ARM_Status = NOT_ARM;
  Throttle = 1000.0f;

  UART1_StartRxToIdle_DMA();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    // 1. Capture Cycle Time (Loop Pacing)
    current_time = TIM2->CNT;
    dt = current_time - prev_time;
    prev_time = current_time;

    if (dt > 10000) dt = 4000;

    // 2. UART Command (PID Tuning ONLY)
    if (line_ready) {
        char local[CMD_LINE_SIZE];
        __disable_irq();
        strncpy(local, cmd_ready, CMD_LINE_SIZE);
        local[CMD_LINE_SIZE-1] = 0;
        line_ready = 0;
        __enable_irq();
        ProcessLine(local);
    }

    // 3. Sensor Update
    IMU_PROCESS();
    COMPASS_PROCESS();

    // 4. Update Filter & PID with REAL dt
    MPU6500_DATA.dt = (float32_t)dt * 1.0e-6f;
    Complimentary_Filter_Predict(&Complimentary_Filter, &MPU6500_DATA);
    if (MagCal.state == MAG_CAL_DONE) {
        Complimentary_Filter_Update(&Complimentary_Filter, &HMC5883L_DATA);
    }

    // 5. Control Loop (MPC)
    MPC();

    // 6. Telemetry (Gửi dữ liệu vẽ đồ thị)
    if (HAL_GetTick() - last_telemetry_time > 100) { // 10Hz
        Send_Telemetry();
        last_telemetry_time = HAL_GetTick();
    }

    // 7. Precise Loop Pacing (Đảm bảo Loop 250Hz - 4000us)
    while ((TIM2->CNT - current_time) < 4000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void UART_ParseByte_ISR(uint8_t b) {
    if (b == '\r') return;
    if (b == '\n') {
        cmd_work[cmd_len] = 0;
        if (!line_ready) {
            strncpy(cmd_ready, cmd_work, CMD_LINE_SIZE);
            cmd_ready[CMD_LINE_SIZE-1] = 0;
            line_ready = 1;
        }
        cmd_len = 0;
        return;
    }
    if (b >= 32 && b <= 126) {
        if (cmd_len < (CMD_LINE_SIZE - 1)) cmd_work[cmd_len++] = (char)b;
        else cmd_len = 0;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        for (uint16_t i = 0; i < Size; i++) UART_ParseByte_ISR(rx_dma_buf[i]);
        UART1_StartRxToIdle_DMA();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_DMAStop(&huart1);
        volatile uint32_t temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;
        UART1_StartRxToIdle_DMA();
    }
}
/* USER CODE BEGIN 4 */

// HÀM ĐỌC ĐỘ RỘNG XUNG PWM (Input Capture)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // --- XỬ LÝ TIM5 (4 Kênh chính: A, E, T, R) ---
    if (htim->Instance == TIM5)
    {
        // 1. ROLL (PA0 - CH1)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                val_start_roll = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Sườn lên
            } else {
                RC_Raw_Roll = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - val_start_roll; // Sườn xuống
            }
        }
        // 2. PITCH (PA1 - CH2)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
                val_start_pitch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            } else {
                RC_Raw_Pitch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - val_start_pitch;
            }
        }
        // 3. THROTTLE (PA2 - CH3)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
                val_start_thr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            } else {
                RC_Raw_Throttle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - val_start_thr;
            }
        }
        // 4. YAW (PA3 - CH4)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
                val_start_yaw = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            } else {
                RC_Raw_Yaw = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - val_start_yaw;
            }
        }
    }

    // --- XỬ LÝ TIM1 (2 Công tắc: SW_Arm, SW_Mode) ---
    // Lưu ý: TIM1 dùng PA8 (CH1) và PA11 (CH4)
    if (htim->Instance == TIM1)
    {
        // 5. SW ARM (PA8 - CH1)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
                val_start_arm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            } else {
                // Xử lý tràn Timer 16-bit (nếu có)
                uint32_t val_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                if (val_end >= val_start_arm)
                    RC_Raw_SW_Arm = val_end - val_start_arm;
                else
                    RC_Raw_SW_Arm = (0xFFFF - val_start_arm) + val_end;
            }
        }
        // 6. SW MODE (PA11 - CH4)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) {
                val_start_mode = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            } else {
                uint32_t val_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                if (val_end >= val_start_mode)
                    RC_Raw_SW_Mode = val_end - val_start_mode;
                else
                    RC_Raw_SW_Mode = (0xFFFF - val_start_mode) + val_end;
            }
        }
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
