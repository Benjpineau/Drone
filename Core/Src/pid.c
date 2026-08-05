#include "pid.h"
#include <math.h>


#define PI 3.14159265358979323846f
#define OUTER_LOOP_PERIOD_MS     20   // ~50 Hz : boucle acceleration-> Fd, Yawd
#define ATTITUDE_LOOP_PERIOD_MS   4   // ~250 Hz : boucle d'attitude -> wd

// ============================================================================
//Utilitaires Vecteurs [3]
// ============================================================================
static float Vec3_Dot(Vec3_t a, Vec3_t b)
{
    return a[0]*b[0]
         + a[1]*b[1]
         + a[2]*b[2];
}

static void Vec3_Cross(Vec3_t result, Vec3_t a, Vec3_t b)
{
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

static float Vec3_Norm(const Vec3_t a){
    float norm;
    norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    return norm;

}

static void Vec3_Normalize(Vec3_t result, const Vec3_t a)
{
    float norm = Vec3_Norm(a);

    if (norm > 1e-6f)
    {
        result[0] = a[0] / norm;
        result[1] = a[1] / norm;
        result[2] = a[2] / norm;
    }
    else
    {
        result[0] = 0.0f;
        result[1] = 0.0f;
        result[2] = 0.0f;
    }
}

void Vec3_Cross_Normalized(Vec3_t result, const Vec3_t a, const Vec3_t b){
    Vec3_Cross(result, a,b);
    Vec3_Normalize(result, result);

}

// Constructeur de Matrice 
static void Mat3_FromColumns(Mat3_t R,
    const Vec3_t c1,
    const Vec3_t c2,
    const Vec3_t c3)
    {
    for (int i = 0; i < 3; i++)
        {
        R[i][0] = c1[i];
        R[i][1] = c2[i];
        R[i][2] = c3[i];
        }
    }

//Utilitaires Quaternions
static Quaternion_t Quat_Conjugate(const Quaternion_t *q) {
    Quaternion_t r = {
        q->w,
        -q->x,
        -q->y,
        -q->z
    };
    return r;
}

static Quaternion_t Quat_Multiply(const Quaternion_t *a, const Quaternion_t *b) {
    Quaternion_t r;
    r.w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    r.x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    r.y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    r.z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
    return r;
}

static Quaternion_t RotationMatrix_To_Quaternion(const Mat3_t R)
{
    Quaternion_t q;

    float trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f) * 2.0f;

        q.w = 0.25f * s;
        q.x = (R[2][1] - R[1][2]) / s;
        q.y = (R[0][2] - R[2][0]) / s;
        q.z = (R[1][0] - R[0][1]) / s;
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        // Qxx est le plus grand terme diagonal -> on résout autour de qx
        float s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f; // s = 4*qx
        q.w = (R[2][1] - R[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (R[0][1] + R[1][0]) / s;
        q.z = (R[0][2] + R[2][0]) / s;
    }
    else if (R[1][1] > R[2][2])
    {
        // Qyy est le plus grand terme diagonal -> on résout autour de qy
        float s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f; // s = 4*qy
        q.w = (R[0][2] - R[2][0]) / s;
        q.x = (R[0][1] + R[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (R[1][2] + R[2][1]) / s;
    }
    else
    {
        // Qzz est le plus grand terme diagonal -> on résout autour de qz
        float s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f; // s = 4*qz
        q.w = (R[1][0] - R[0][1]) / s;
        q.x = (R[0][2] + R[2][0]) / s;
        q.y = (R[1][2] + R[2][1]) / s;
        q.z = 0.25f * s;
    }

    return q;
}

static float Sign_NonZero(float v) {
    return (v < 0.0f) ? -1.0f : 1.0f;
}


static void IntegrateYaw(float yawRate, float *yawd, float dt)
{
    *yawd += yawRate * dt;

    while (*yawd > PI)
        *yawd -= 2.0f * PI;

    while (*yawd <= -PI)
        *yawd += 2.0f * PI;
}
static void Acceleration_Controller(
    Vec3_t Fd,
    const Vec3_t acceleration_desired,
    const Vec3_t acceleration_sensor,
    Vec3_t integral_term,
    float dt,
    float mass,
    float g)
{
    Vec3_t error_a;

    static const Vec3_t Kp = {1.0f, 1.0f, 1.0f};//to tune
    static const Vec3_t Ki = {1.0f, 1.0f, 1.0f};//to tune

    for (int i = 0; i < 3; i++)
    {
        error_a[i] = acceleration_desired[i] - acceleration_sensor[i];

        integral_term[i] += error_a[i] * dt;

        Fd[i] = mass * (
            acceleration_desired[i]
            + Kp[i] * error_a[i]
            + Ki[i] * integral_term[i]
        );
    }

    Fd[2] += mass * g;
}

static void DesiredDirection(Vec3_t xd,const float Yawd){
    xd[0] = cosf(Yawd); 
    xd[1] = sinf(Yawd);
    xd[2] = 0.f;
}

static Attitude_Command_t Force_n_Yaw_2_Attitude(
    const Vec3_t Fd,
    float Yawd)
{  
    Attitude_Command_t cmd;

    cmd.thrust = Vec3_Norm(Fd);

    Vec3_t b3d, b1c, b2d, b1d;

    Vec3_Normalize(b3d, Fd);

    DesiredDirection(b1c, Yawd);

    Vec3_Cross_Normalized(b2d, b3d, b1c);
    Vec3_Cross(b1d, b2d, b3d);

    Vec3_Normalize(b1d, b1d); //On renormalise juste pour éviter le drift de cet axe

    Mat3_t Rd;
    Mat3_FromColumns(Rd, b1d, b2d, b3d);

    cmd.attitude = RotationMatrix_To_Quaternion(Rd);

    return cmd;
}
static void Attitude_Controller(Vec3_t wd, const Quaternion_t *qd, const Quaternion_t *qs, const Vec3_t wff)
{
    //Les quaternions sont unitaires (normalisés fréquement), donc l'inverse est seulement le conjugué


    Quaternion_t qd_inv = Quat_Conjugate(qd);
    Quaternion_t qe = Quat_Multiply(&qd_inv, qs);

    float s = Sign_NonZero(qe.w);
    static const Vec3_t Kq = {1.0f, 1.0f, 1.0f};//to tune

    wd[0] = Kq[0]*2.0f * s * qe.x + wff[0];
    wd[1] = Kq[1]*2.0f * s * qe.y + wff[1];
    wd[2] = Kq[2]*2.0f * s * qe.z + wff[2];

}
static void Rate_Controller(Vec3_t tau, const Vec3_t wd, const Vec3_t ws, Vec3_t integral_term, Vec3_t previous_wd, float dt)
{
    //Calcul de l'erreur
    Vec3_t ew;
    Vec3_t d_wd;
    static const Vec3_t J = {0.00262f, 0.003f, 0.0065f};//Ixx,Iyy,Izz


    static const Vec3_t Kp = {1.0f, 1.0f, 1.0f};//to tune
    static const Vec3_t Ki = {1.0f, 1.0f, 1.0f};//to tune

    static const float Imax = 1.0f;

    for (int i = 0; i < 3; i++)
    {
        ew[i] = wd[i] - ws[i];

        //Bornage du terme intégrale
        integral_term[i] += ew[i] * dt;
        if (integral_term[i] > Imax)
            integral_term[i] = Imax;

        if (integral_term[i] < -Imax)
            integral_term[i] = -Imax;
        //On évite les dt trop petits
        if (dt > 1e-6f)
            {
                d_wd[i] = (wd[i] - previous_wd[i]) / dt;
            }
            else
            {
                d_wd[i] = 0.0f;
            }

        tau[i] = Kp[i]*ew[i] + Ki[i]*integral_term[i] + J[i]*d_wd[i];
        previous_wd[i] = wd[i];
    }



}

static inline uint8_t Task_Due(uint32_t tick_actuel, uint32_t *last_tick,
    uint32_t period_ms, float *dt_out)
    {
    if ((tick_actuel - *last_tick) >= period_ms) {
        *dt_out = (tick_actuel - *last_tick) / 1000.0f;
        *last_tick = tick_actuel;
        return 1;
        }
    return 0;
    }

void MapInput(Controller_Input_t *in, const Vec3_t in_accel_d, const Vec3_t in_accel_s, const Vec3_t in_ws, const Quaternion_t *in_qs, float in_YawRate, float in_dt){
    for (int i = 0; i < 3; i++) {
        in->accel_d[i] = in_accel_d[i];
        in->accel_s[i] = in_accel_s[i];
        in->ws[i]      = in_ws[i];
    }
    in->qs = *in_qs;
    in->YawRate = in_YawRate;
    in->dt = in_dt;
}
void InitState(Controller_State_t *state){
    for (int i = 0; i < 3; i++) {
        state->integral_accel[i] = 0.f;
        state->integral_rate[i] = 0.f;
        state->previous_wd[i] = 0.f;
    }
    state->Yawd = 0.f;
}

void Global_Controller(
    const Controller_Input_t *in,
    Controller_State_t *state,
    Controller_Output_t *out,
    uint32_t tick_actuel)
{
    float dt_outer, dt_attitude;

    if (Task_Due(tick_actuel, &state->last_tick_outer, OUTER_LOOP_PERIOD_MS, &dt_outer)) {
        Acceleration_Controller(
            state->Fd,
            in->accel_d,
            in->accel_s,
            state->integral_accel,
            dt_outer,        
            in->mass,
            in->g
        );

        IntegrateYaw(in->YawRate, &state->Yawd, dt_outer);

        state->att_c = Force_n_Yaw_2_Attitude(state->Fd, state->Yawd);
    }

    if (Task_Due(tick_actuel, &state->last_tick_attitude, ATTITUDE_LOOP_PERIOD_MS, &dt_attitude)) {
        Vec3_t wff = {0.f, 0.f, 0.f}; // pas encore implémenté

        Attitude_Controller(
            state->wd,
            &state->att_c.attitude,
            &in->qs,
            wff);
    }

    Rate_Controller(
        out->tau,
        state->wd,
        in->ws,
        state->integral_rate,
        state->previous_wd,
        in->dt);   

    out->thrust = state->att_c.thrust;
}

