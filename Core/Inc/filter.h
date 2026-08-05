#ifndef FILTER_H    // 1. Si "FILTER_H" n'est pas encore défini
#define FILTER_H    // 2. Alors on le définit immédiatement

#include <stdint.h>
#include "ICM42688P.h"

// Structure pour un axe en PT1
typedef struct {
    float alpha;
    float valeur_precedente;
} Filtre_PT1_Axe_t;

// Structure pour un axe en PT2 (Forme Biquad Passe-Bas standard)
typedef struct {
    float b0, b1, b2; // Coefficients du numérateur
    float a1, a2;     // Coefficients du dénominateur
    float x1, x2;     // États précédents de l'entrée brute
    float y1, y2;     // États précédents de la sortie filtrée
} Filtre_PT2_Axe_t;

// Structure compacte pour le Notch (gère les 3 axes)
typedef struct {
    float b0, b1, b2; // Coefficients (Identiques pour X,Y,Z)
    float a1, a2;     // Coefficients (Identiques pour X,Y,Z)
    
    // Historique des états (Séparé pour X=0, Y=1, Z=2)
    float x1[3], x2[3];
    float y1[3], y2[3];
    
    float Q; // Facteur de qualité (mémorisé pour les updates dynamiques)
} Filtre_Notch_3Axes_t;


void Filtre_PT1_3Axes_Init(Filtre_PT1_Axe_t filtres[3], float f_coupure, float dt);

void Filtre_PT2_3Axes_Init(Filtre_PT2_Axe_t filtres[3], float f_coupure, float f_echantillonnage);

void LPF_Gyro_PT1(ICM42688P_Data_t *data, Filtre_PT1_Axe_t filtres[3]);

void LPF_Gyro_PT2(ICM42688P_Data_t *data, Filtre_PT2_Axe_t filtres[3]); 


void LPF_Accel_PT1(ICM42688P_Data_t *data, Filtre_PT1_Axe_t filtres[3]);

void Filtre_Notch_3Axes_Config(Filtre_Notch_3Axes_t *f, float f_cible, float f_echantillonnage, float Q);

void Notch_Gyro(ICM42688P_Data_t *data, Filtre_Notch_3Axes_t *f);

void Filtre_Notch_3Axes_Init(Filtre_Notch_3Axes_t *f);

#endif // FILTER_H _ // 