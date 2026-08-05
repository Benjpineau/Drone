#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "attitude.h"  // pour Quaternion_t (erreur d'attitude géométrique)

typedef float Vec3_t[3];
typedef float Mat3_t[3][3];

typedef struct {
    float thrust;
    Quaternion_t attitude;
} Attitude_Command_t;


typedef struct {
    Vec3_t accel_d;
    Vec3_t accel_s;
    Vec3_t ws;
    Quaternion_t qs;
    float YawRate;
    float dt;
    float mass;
    float g;
} Controller_Input_t;

typedef struct {
    Vec3_t tau;
    float thrust;
} Controller_Output_t;

typedef struct {
    Vec3_t Fd;
    Vec3_t integral_accel;
    Vec3_t integral_rate;
    Vec3_t wd;
    Vec3_t previous_wd;
    float Yawd;
    uint32_t last_tick_outer;
    uint32_t last_tick_attitude;
    Attitude_Command_t att_c;
} Controller_State_t;

void InitState(Controller_State_t *state);

void MapInput(Controller_Input_t *in, const Vec3_t in_accel_d, const Vec3_t in_accel_s, const Vec3_t in_ws, const Quaternion_t *in_qs, float in_YawRate, float in_dt);

void Global_Controller(
    const Controller_Input_t *in,
    Controller_State_t *state,
    Controller_Output_t *out,
    uint32_t tick_actuel);
#endif // PID_H_
