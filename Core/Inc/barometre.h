#ifndef BAROMETRE_H_
#define BAROMETRE_H_

#include "bmp280.h"

typedef struct {
    float temperature; // En °C
    float pressure;    // En Pa
} baro_data_t;


bool initBarometre(void);
void calibrerPressionAuSol(void);
float obtenirAltitudeRelative(void);
baro_data_t obtenirDonneesCompensees(void);

#endif //BAROMETRE_H_