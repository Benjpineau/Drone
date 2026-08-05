#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "ICM42688P.h"

typedef struct {
    float w, x, y, z;
} Quaternion_t;

typedef struct {
    float roll, pitch, yaw;
} Euler_t;

typedef struct {
    float ki, kp, integralFBx, integralFBy, integralFBz;
} MahonyFilter_t;

void Update_Orientation_Quaternion(ICM42688P_Data_t *imu, Quaternion_t *q_actuel, float dt);
void Quaternion_To_Euler(Quaternion_t *q, Euler_t *euler);
void Mahony_Init(MahonyFilter_t *filter, float kp, float ki);
void Mahony_Update(MahonyFilter_t *filter, ICM42688P_Data_t *imu, Quaternion_t *q_actuel, float dt);


#endif //ATTITUDE_H_