/*
 * Cal.c
 *
 *  Created on: Dec 27, 2025
 *      Author: THINH
 */
#include "Cal.h"

void MagCal_Update(MagCal_Simple_t* c, MAG_RAW_DATA_t* raw , MAG_DATA_t* out){
    switch (c->state){
    case MAG_CAL_IDLE:
        // use default offset
    	c->offset[0] = 4.7840004;
    	c->offset[1] = -3.45000076;
    	c->offset[2] = 7.36000061;
    	c->scale[0] = 0.937098265;
    	c->scale[1] = 0.954184234;
    	c->scale[2] = 1.13012183;
    	c->state = MAG_CAL_DONE;
        break;
    case MAG_CAL_START:
        // reset min/max
        for (int i = 0; i < 3; i++) {
            c->min[i] =  1e9f;
            c->max[i] = -1e9f;
        }
        c->samples = 0;
        c->state = MAG_CAL_COLLECTING;
        break;

    case MAG_CAL_COLLECTING:
        // cập nhật min/max
        for (int i = 0; i < 3; i++) {
            if (raw->mag[i] < c->min[i]) c->min[i] = raw->mag[i];
            if (raw->mag[i] > c->max[i]) c->max[i] = raw->mag[i];
        }
        c->samples++;
        // đủ mẫu thì tính hiệu chuẩn
        if (c->samples >= c->samples_target) {
            c->state = MAG_CAL_COMPUTE;
        }
        break;

    case MAG_CAL_COMPUTE:
    {
        float radius[3];
        // Tính offset và bán kính
        for (int i = 0; i < 3; i++) {
            c->offset[i] = (c->max[i] + c->min[i]) * 0.5f;
            radius[i]    = (c->max[i] - c->min[i]) * 0.5f;
        }
        float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
        // scale
        for (int i = 0; i < 3; i++) {
            c->scale[i] = avg_radius / radius[i];
        }
        c->state = MAG_CAL_DONE;
    }
    break;
    case MAG_CAL_DONE:
        // hiệu chuẩn xong, dùng offset/scale
    	Mag_ApplyCalibration(c, raw, out);
        break;
    }
    if(c->state != MAG_CAL_DONE){
    	for(int i = 0 ; i < 3 ;i ++){
    	  out->mag_uT[i] = 0;
    	}
    }
}
//COMPASS
void Mag_ApplyCalibration(MagCal_Simple_t* c,MAG_RAW_DATA_t* raw,MAG_DATA_t* out){
    for (int i = 0; i < 3; i++)
    {
        float x = raw->mag[i] - c->offset[i];
        x *= c->scale[i];

        // đổi sang µT
        out->mag_uT[i] = x * (1.0f / c->S);
    }
    //out->timestamp = raw->timestamp;
}
