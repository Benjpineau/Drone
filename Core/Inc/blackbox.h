#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>


#pragma pack(push, 1)
typedef struct {
    uint32_t tick;       // Temps en ms (4 octets)
    float gyro_x;        // Gyro X filtré (4 octets)
    float gyro_y;        // Gyro Y filtré (4 octets)
    float gyro_z;        // Gyro Z filtré (4 octets)
    float accel_x;       // Accel X filtré (4 octets)
    float accel_y;       // Accel Y filtré (4 octets)
    float accel_z;       // Accel Z filtré (4 octets)
    float altitude;      // Altitude barométrique ou estimée (4 octets)
    float consigne_forward;
    float consigne_lateral;
    
} Blackbox_Frame_t;
#pragma pack(pop)

#endif //BLACKBOX_H_