/*
 * Cal.h
 *
 *  Created on: Dec 27, 2025
 *      Author: THINH
 */

#ifndef INC_CAL_H_
#define INC_CAL_H_

#include "main.h"
#include "STATES_SENSOR.h"
#include "arm_math.h"
#include "Filter.h"

typedef enum {
    MAG_CAL_IDLE = 0,       // chưa làm gì
    MAG_CAL_START,          // reset min/max
    MAG_CAL_COLLECTING,     // đang thu thập mẫu
    MAG_CAL_COMPUTE,        // tính toán hiệu chỉnh
    MAG_CAL_DONE            // xong
} MagCal_State_t;
typedef struct {
    MagCal_State_t state;
    float32_t S;               // LSB/G
    float32_t min[3];
    float32_t max[3];
    float32_t offset[3];       //
    float32_t scale[3];        // diag scale
    uint32_t samples;      // đếm số mẫu đã thu
    uint32_t samples_target; // số mẫu cần (ví dụ 5000)
} MagCal_Simple_t;

//5000 mẫu * 0.04 -> thời gian lấy mẫu để hiệu chỉnh
void Mag_ApplyCalibration(MagCal_Simple_t* c,MAG_RAW_DATA_t* raw,MAG_DATA_t* out);
void MagCal_Update(MagCal_Simple_t* c, MAG_RAW_DATA_t* raw,MAG_DATA_t* out);


#endif /* INC_CAL_H_ */
