///*
// * PID.h
// *
// *  Created on: Dec 14, 2025
// *      Author: THINH
// */
//
//#ifndef INC_PID_H_
//#define INC_PID_H_
//#include "arm_math.h"
//typedef struct {
//    float32_t kp;
//    float32_t ki;
//    float32_t kd;
//
//    float32_t feed_forward;   // angular rate feedforward
//
//    float32_t error;
//    float32_t prev_error;
//
//    float32_t integral;
//    float32_t derivative;
//
//    float32_t alpha_lpf;      // derivative low-pass filter factor (0-1)
//
//    float32_t max_output;     // giới hạn PID output (torque / rate)
//    float32_t i_limit,d_limit;        // anti-windup
//    float32_t output;
//} PID_ALTIDUE_t;
//void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid);
//void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback , float32_t dt); //ROLL PITCH YAW
//void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt); //WX WY WZ
//
//
//#endif /* INC_PID_H_ */
/*
 * PID.h
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 * Updated: iNAV Style Support (D-on-Measurement & Dynamic FF)
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "arm_math.h"

typedef struct {
    // --- Tunings ---
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t feed_forward;   // iNav: Dynamic FF Gain (Rate Loop)

    // --- State Variables ---
    float32_t error;
    float32_t prev_error;     // Dùng cho Angle Loop (hoặc legacy D-term)

    float32_t integral;
    float32_t derivative;

    // --- NEW: iNAV / Betaflight Style State Variables ---
    float32_t prev_measure;   // Lưu giá trị đo cũ (Dùng cho D-on-Measurement)
    float32_t prev_setpoint;  // Lưu setpoint cũ (Dùng cho Dynamic Feed Forward)

    // --- Configs ---
    float32_t alpha_lpf;      // derivative low-pass filter factor (0-1)
    float32_t max_output;     // giới hạn PID output (torque / rate)
    float32_t i_limit;        // anti-windup
    float32_t d_limit;        // (Có thể không dùng trong thuật toán mới nhưng giữ để tương thích)
    float32_t output;

} PID_ALTIDUE_t;

void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid);

// Angle Loop (Outer Loop) - Vẫn dùng thuật toán cơ bản
void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback , float32_t dt);

// Rate Loop (Inner Loop) - Dùng thuật toán iNav (D-on-Measurement & Dynamic FF)
void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt);

#endif /* INC_PID_H_ */
