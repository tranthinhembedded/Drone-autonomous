///*
// * PID.c
// *
// *  Created on: Dec 14, 2025
// *      Author: THINH
// */
//#include "PID.h"
//
//void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid){
//     pid->derivative = 0;
//     pid->error = 0;
//     pid->prev_error = 0;
//     pid->integral = 0;
//     pid->output = 0;
//}
//void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
//     pid->error = setpoint - feedback;
//
//     if(pid->error > 180){
//    	 pid->error = pid->error - 360.0;
//     }
//     if(pid->error < -180){
//    	 pid->error = pid->error + 360.0;
//     }
//     float32_t deta_error = pid->error - pid->prev_error;
//
//     if (deta_error > 180) deta_error -= 360;
//     if (deta_error < -180) deta_error += 360;
//
//     if (fabsf(pid->error) < 20.0f){
//        pid->integral += 0.5 * (pid->error + pid->prev_error) * dt;
//     }
//     else{
//    	pid->integral = 0.0;
//     }
//     if(pid->integral > pid->i_limit) pid->integral = pid->i_limit;
//     if(pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
//
//     pid->derivative = pid->alpha_lpf * pid->derivative + (1.0 - pid->alpha_lpf) * (deta_error/dt);
//
//     pid->prev_error = pid->error;
//     pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
//	if (pid->output > pid->max_output) pid->output = pid->max_output;
//	else if (pid->output < -pid->max_output) pid->output = -pid->max_output;
//
//}
//void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
//     pid->error = setpoint - feedback;
//
//     pid->integral += 0.5 * (pid->error + pid->prev_error) * dt;
//
//     if(pid->integral > pid->i_limit) pid->integral = pid->i_limit;
//     if(pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
//
//     pid->derivative = pid->alpha_lpf * pid->derivative + (1.0 - pid->alpha_lpf) * ((pid->error - pid->prev_error)/dt);
//
//     pid->prev_error = pid->error;
//     pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
//     if(pid->output > 0) pid->output +=  pid->feed_forward;
//     if(pid->output < 0) pid->output -=  pid->feed_forward;
//	if (pid->output > pid->max_output) pid->output = pid->max_output;
//	else if (pid->output < -pid->max_output) pid->output = -pid->max_output;
//
//}
/*
 * PID.c
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 */
/*
 * PID.c
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 * Final Stable Version
 */

//#include "PID.h"
//#include <math.h>
//
///* Helper function to constrain values */
//static float32_t clamp_value(float32_t value, float32_t min, float32_t max) {
//    if (value > max) return max;
//    if (value < min) return min;
//    return value;
//}
//
///* Reset PID structure */
//void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid){
//     pid->derivative = 0;
//     pid->error = 0;
//     pid->prev_error = 0;
//     pid->integral = 0;
//     pid->output = 0;
//}
//
///* * Calculate PID for Angle/Attitude (Outer Loop)
// * Handles -180 to 180 degree wrapping
// */
//void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
//     // 1. Calculate Error
//     pid->error = setpoint - feedback;
//
//     // 2. Wrap Error (-180 to 180) for shortest path rotation
//     if(pid->error > 180.0f) pid->error -= 360.0f;
//     else if(pid->error < -180.0f) pid->error += 360.0f;
//
//     // 3. Calculate Delta Error
//     float32_t deta_error = pid->error - pid->prev_error;
//
//     // Wrap Delta Error
//     if (deta_error > 180.0f) deta_error -= 360.0f;
//     else if (deta_error < -180.0f) deta_error += 360.0f;
//
//     // 4. Integral Term (Smart Accumulation)
//     // Only accumulate I when error is small (< 20 deg) to prevent windup
//     if (fabsf(pid->error) < 20.0f){
//        pid->integral += 0.5f * (pid->error + pid->prev_error) * dt;
//        pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);
//     }
//     // Note: If error >= 20, we simply FREEZE integral (do nothing), NOT reset to 0.
//
//     // 5. Derivative Term with LPF
//     // Divide by dt to get true rate of change
//     float32_t derivative_raw = deta_error / dt;
//     pid->derivative = pid->alpha_lpf * pid->derivative + (1.0f - pid->alpha_lpf) * derivative_raw;
//
//     // 6. Final Output
//     pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
//     pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);
//
//     pid->prev_error = pid->error;
//}
//
///* * Calculate PID for Rate/Angular Velocity (Inner Loop)
// * Includes Feed Forward logic
// */
//void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
//     pid->error = setpoint - feedback;
//
//     // 1. Integral Term
//     // Rate loop needs integral almost always to hold attitude
//     pid->integral += 0.5f * (pid->error + pid->prev_error) * dt;
//     pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);
//
//     // 2. Derivative Term with LPF
//     float32_t derivative_raw = (pid->error - pid->prev_error) / dt;
//     pid->derivative = pid->alpha_lpf * pid->derivative + (1.0f - pid->alpha_lpf) * derivative_raw;
//
//     // 3. Feed Forward (Dynamic)
//     // Adds output proportional to the requested speed (Setpoint)
//     float32_t ff_term = pid->feed_forward * setpoint;
//
//     // 4. Final Output
//     pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * pid->derivative) + ff_term;
//     pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);
//
//     pid->prev_error = pid->error;
//}
/*
 * PID.c
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 * Final Stable Version with D-Term Limit
 */

#include "PID.h"
#include <math.h>

static float32_t clamp_value(float32_t value, float32_t min, float32_t max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* Reset PID structure */
void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid){
     pid->derivative = 0;
     pid->error = 0;
     pid->prev_error = 0;
     pid->integral = 0;
     pid->output = 0;
     pid->prev_measure = 0;
     pid->prev_setpoint = 0;
}

/* ANGLE LOOP (Outer Loop) */
void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
     if (dt < 0.0001f) dt = 0.0001f;
     pid->error = setpoint - feedback;

     if(pid->error > 180.0f) pid->error -= 360.0f;
     else if(pid->error < -180.0f) pid->error += 360.0f;

     float32_t P_term = pid->kp * pid->error;

     if (fabsf(pid->error) < 20.0f){
         pid->integral += pid->ki * pid->error * dt;
         pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);
     }

     pid->output = P_term + pid->integral;
     pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);

     pid->prev_error = pid->error;
     pid->prev_measure = feedback;
     pid->prev_setpoint = setpoint;
}

/* RATE LOOP (Inner Loop) */
void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt){
     if (dt < 0.0001f) dt = 0.0001f;
     pid->error = setpoint - feedback;

     float32_t P_term = pid->kp * pid->error;

     pid->integral += pid->ki * pid->error * dt;
     pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);

     float32_t delta_measure = feedback - pid->prev_measure;
     float32_t D_raw = -(delta_measure / dt) * pid->kd;

     // Lọc LPF cho D-term
     pid->derivative = pid->alpha_lpf * pid->derivative + (1.0f - pid->alpha_lpf) * D_raw;
     pid->derivative = clamp_value(pid->derivative, -pid->d_limit, pid->d_limit);

     // 4. Feed Forward (Dynamic)
     float32_t delta_setpoint = setpoint - pid->prev_setpoint;
     float32_t FF_term = (delta_setpoint / dt) * pid->feed_forward;

     float32_t ff_limit = pid->max_output * 0.2f;
     FF_term = clamp_value(FF_term, -ff_limit, ff_limit);

     // 5. Final Output
     pid->output = P_term + pid->integral + pid->derivative + FF_term;
     pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);

     pid->prev_error = pid->error;
     pid->prev_measure = feedback;
     pid->prev_setpoint = setpoint;
}
