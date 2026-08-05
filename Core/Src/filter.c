#include "filter.h"
#include <math.h>

void Filtre_PT1_3Axes_Init(Filtre_PT1_Axe_t filtres[3], float f_coupure, float dt) {
    float rc = 1.0f / (2.0f * M_PI * f_coupure);
    float alpha = dt / (dt + rc);
    
    for(int i = 0; i < 3; i++) {
        filtres[i].alpha = alpha;
        filtres[i].valeur_precedente = 0.0f;
    }
}


void Filtre_PT2_3Axes_Init(Filtre_PT2_Axe_t filtres[3], float f_coupure, float f_echantillonnage) {
    // Calcul des coefficients de Butterworth du 2e ordre
    float wc = 2.0f * M_PI * f_coupure;
    float angle = 1.0f / tanf(wc / (2.0f * f_echantillonnage));
    float angle_sq = angle * angle;
    
    float b0 = 1.0f / (1.0f + M_SQRT2 * angle + angle_sq);
    float b1 = 2.0f * b0;
    float b2 = b0;
    float a1 = 2.0f * b0 * (1.0f - angle_sq);
    float a2 = b0 * (1.0f - M_SQRT2 * angle + angle_sq);

    for(int i = 0; i < 3; i++) {
        filtres[i].b0 = b0; filtres[i].b1 = b1; filtres[i].b2 = b2;
        filtres[i].a1 = a1; filtres[i].a2 = a2;
        filtres[i].x1 = 0.0f; filtres[i].x2 = 0.0f;
        filtres[i].y1 = 0.0f; filtres[i].y2 = 0.0f;
    }
}

void LPF_Gyro_PT1(ICM42688P_Data_t *data, Filtre_PT1_Axe_t filtres[3]) {
    float *gyro[3] = { &data->gyro_x, &data->gyro_y, &data->gyro_z };

    for(int i = 0; i < 3; i++) {
        filtres[i].valeur_precedente = filtres[i].valeur_precedente + filtres[i].alpha * (*gyro[i] - filtres[i].valeur_precedente);
        *gyro[i] = filtres[i].valeur_precedente;
    }
}


void LPF_Gyro_PT2(ICM42688P_Data_t *data, Filtre_PT2_Axe_t filtres[3]) {
    float x[3] = {data->gyro_x, data->gyro_y, data->gyro_z};
    float y[3];

    for(int i = 0; i < 3; i++) {
        // Équation de différence de la forme Biquad Direct Form 1
        y[i] = filtres[i].b0 * x[i] + filtres[i].b1 * filtres[i].x1 + filtres[i].b2 * filtres[i].x2
                                    - filtres[i].a1 * filtres[i].y1 - filtres[i].a2 * filtres[i].y2;
        
        // Mise à jour de la mémoire du filtre
        filtres[i].x2 = filtres[i].x1;
        filtres[i].x1 = x[i];
        filtres[i].y2 = filtres[i].y1;
        filtres[i].y1 = y[i];
    }

    // Réinjection des données filtrées dans la structure IMU
    data->gyro_x = y[0];
    data->gyro_y = y[1];
    data->gyro_z = y[2];
}

void LPF_Accel_PT1(ICM42688P_Data_t *data, Filtre_PT1_Axe_t f[3]) {
    float *accel[3] = { &data->accel_x, &data->accel_y, &data->accel_z };

    for(int i = 0; i < 3; i++) {
        // Formule PT1 : Y(n) = Y(n-1) + alpha * (X(n) - Y(n-1))
        f[i].valeur_precedente = f[i].valeur_precedente + f[i].alpha * (*accel[i] - f[i].valeur_precedente);
        *accel[i] = f[i].valeur_precedente;
    }
}




void Filtre_Notch_3Axes_Config(Filtre_Notch_3Axes_t *f, float f_cible, float f_echantillonnage, float Q) {
    f->Q = Q;
    
    // Sécurité si la fréquence cible est hors limites
    if (f_cible <= 0.0f || f_cible >= (f_echantillonnage / 2.0f)) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }

    float omega = 2.0f * M_PI * f_cible / f_echantillonnage;
    float alpha = sinf(omega) / (2.0f * Q);
    float cos_omega = cosf(omega);

    float a0 = 1.0f + alpha;
    
    // Normalisation par a0 pour économiser des divisions au CPU
    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cos_omega) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cos_omega) / a0;
    f->a2 = (1.0f - alpha) / a0;
}

// Fonction d'initialisation à zéro des mémoires
void Filtre_Notch_3Axes_Init(Filtre_Notch_3Axes_t *f) {
    for(int i = 0; i < 3; i++) {
        f->x1[i] = 0.0f; f->x2[i] = 0.0f;
        f->y1[i] = 0.0f; f->y2[i] = 0.0f;
    }
}

void Notch_Gyro(ICM42688P_Data_t *data, Filtre_Notch_3Axes_t *f) {
    float *gyro[3] = { &data->gyro_x, &data->gyro_y, &data->gyro_z };

    for (int i = 0; i < 3; i++) {
        // Équation de différence Biquad
        float y = f->b0 * (*gyro[i]) + f->b1 * f->x1[i] + f->b2 * f->x2[i]
                                      - f->a1 * f->y1[i] - f->a2 * f->y2[i];
        
        // Update de la mémoire
        f->x2[i] = f->x1[i];
        f->x1[i] = *gyro[i];
        f->y2[i] = f->y1[i];
        f->y1[i] = y;

        // Réécriture
        *gyro[i] = y;
    }
}