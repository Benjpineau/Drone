#include "attitude.h"
#include "filter.h"
#include <math.h>


void Update_Orientation_Quaternion(ICM42688P_Data_t *imu, Quaternion_t *q_actuel, float dt) {
    // 1. Convertir les vitesses du gyro de deg/s en rad/s
    float gx = imu->gyro_x * (M_PI / 180.0f);
    float gy = imu->gyro_y * (M_PI / 180.0f);
    float gz = imu->gyro_z * (M_PI / 180.0f);

    // 2. Sauvegarde des états actuels pour le calcul 
    float qw = q_actuel->w;
    float qx = q_actuel->x;
    float qy = q_actuel->y;
    float qz = q_actuel->z;

    // 3. Intégration de l'équation différentielle 
    q_actuel->w += 0.5f * (-qx * gx - qy * gy - qz * gz) * dt;
    q_actuel->x += 0.5f * ( qw * gx + qy * gz - qz * gy) * dt;
    q_actuel->y += 0.5f * ( qw * gy - qx * gz + qz * gx) * dt;
    q_actuel->z += 0.5f * ( qw * gz + qx * gy - qy * gx) * dt;

    // 4. Normalisation
    float norme = sqrtf(q_actuel->w * q_actuel->w + 
                        q_actuel->x * q_actuel->x + 
                        q_actuel->y * q_actuel->y + 
                        q_actuel->z * q_actuel->z);
                        
    if (norme > 0.0f) {
        q_actuel->w /= norme;
        q_actuel->x /= norme;
        q_actuel->y /= norme;
        q_actuel->z /= norme;
    }
}

void Quaternion_To_Euler(Quaternion_t *q, Euler_t *euler) {
    // 1. Calcul du Roll (Roulis - inclinaison autour de l'axe X)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    // 2. Calcul du Pitch (Tangage - inclinaison autour de l'axe Y)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    // Sécurité contre les singularités (Gimbal Lock à +/- 90 degrés)
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI);
    } else {
        euler->pitch = asinf(sinp) * (180.0f / M_PI);
    }

    // 3. Calcul du Yaw (Lacet - cap autour de l'axe Z)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}

// ============================================================================
// Mahony_Init
// ============================================================================
void Mahony_Init(MahonyFilter_t *filter, float kp, float ki)
{
    filter->kp = kp;
    filter->ki = ki;
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;
}

// ============================================================================
// Mahony_Update
// ----------------------------------------------------------------------------
// Corrige gx,gy,gz (vitesses gyro) à partir de l'accéléromètre AVANT de faire
// exactement la même intégration/normalisation que Update_Orientation_Quaternion
// -- seule la source des gx,gy,gz change, pas la mécanique d'intégration.
// ============================================================================
void Mahony_Update(MahonyFilter_t *filter, ICM42688P_Data_t *imu, Quaternion_t *q_actuel, float dt)
{
    float gx = imu->gyro_x * (M_PI / 180.0f);
    float gy = imu->gyro_y * (M_PI / 180.0f);
    float gz = imu->gyro_z * (M_PI / 180.0f);

    float ax = imu->accel_x;
    float ay = imu->accel_y;
    float az = imu->accel_z;

    float norme_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norme_a > 0.0f) {
        ax /= norme_a;
        ay /= norme_a;
        az /= norme_a;

        float qw = q_actuel->w, qx = q_actuel->x, qy = q_actuel->y, qz = q_actuel->z;


        float vx = 2.0f * (qx*qz - qw*qy);
        float vy = 2.0f * (qw*qx + qy*qz);
        float vz = qw*qw - qx*qx - qy*qy + qz*qz;

        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        // Terme intégral : estime lentement le biais du gyro
        if (filter->ki > 0.0f) {
            filter->integralFBx += filter->ki * ex * dt;
            filter->integralFBy += filter->ki * ey * dt;
            filter->integralFBz += filter->ki * ez * dt;
            gx += filter->integralFBx;
            gy += filter->integralFBy;
            gz += filter->integralFBz;
        } else {
            filter->integralFBx = 0.0f;
            filter->integralFBy = 0.0f;
            filter->integralFBz = 0.0f;
        }

        // Terme proportionnel : correction immédiate
        gx += filter->kp * ex;
        gy += filter->kp * ey;
        gz += filter->kp * ez;
    }

    // ---- Intégration : identique à Update_Orientation_Quaternion, mais sur
    // les gx,gy,gz CORRIGÉS plutôt que les vitesses gyro brutes ----
    float qw = q_actuel->w;
    float qx = q_actuel->x;
    float qy = q_actuel->y;
    float qz = q_actuel->z;

    q_actuel->w += 0.5f * (-qx * gx - qy * gy - qz * gz) * dt;
    q_actuel->x += 0.5f * ( qw * gx + qy * gz - qz * gy) * dt;
    q_actuel->y += 0.5f * ( qw * gy - qx * gz + qz * gx) * dt;
    q_actuel->z += 0.5f * ( qw * gz + qx * gy - qy * gx) * dt;

    float norme = sqrtf(q_actuel->w * q_actuel->w +
                        q_actuel->x * q_actuel->x +
                        q_actuel->y * q_actuel->y +
                        q_actuel->z * q_actuel->z);

    if (norme > 0.0f) {
        q_actuel->w /= norme;
        q_actuel->x /= norme;
        q_actuel->y /= norme;
        q_actuel->z /= norme;
    }
}