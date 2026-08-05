#include "barometre.h"
#include "bmp280.h"
#include <string.h>
#include <math.h>

//In main.c
extern I2C_HandleTypeDef hi2c2;


static BMP280_HandleTypedef monBMP280;
static float pression_au_sol = 101325.0f;


typedef struct {
    int32_t temperature;
    uint32_t pressure;
} uncompensated_data_t;

bool initBarometre(void) {
    bmp280_params_t params_t;

    memset(&monBMP280, 0, sizeof(BMP280_HandleTypedef));
    memset(&params_t, 0, sizeof(bmp280_params_t));

    params_t.mode = BMP280_MODE_NORMAL;
    params_t.filter = BMP280_FILTER_8;
    params_t.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
    params_t.oversampling_temperature = BMP280_LOW_POWER;
    params_t.standby = BMP280_STANDBY_05;

    monBMP280.addr = BMP280_I2C_ADDRESS_0; // SDO à la masse donc 0x76    
    monBMP280.params = params_t; 
    monBMP280.i2c = &hi2c2; 

    return bmp280_init(&monBMP280, &params_t);
}

uncompensated_data_t readBarometre(void) {
    int32_t temp_brute = 0;
    uint32_t press_brute = 0;
    

    bmp280_read_fixed(&monBMP280, &temp_brute, &press_brute, NULL);
    
    uncompensated_data_t uncompensated_data = {0, 0};
    uncompensated_data.pressure = press_brute;
    uncompensated_data.temperature = temp_brute;
    
    return uncompensated_data;
}

baro_data_t obtenirDonneesCompensees(void) {
    uncompensated_data_t raw = readBarometre();
    int32_t fine_temp = 0;
    int32_t temp_calculee = 0;
    uint32_t press_calculee = 0;
    
    temp_calculee = compensate_temperature(&monBMP280, raw.temperature, &fine_temp);
    press_calculee = compensate_pressure(&monBMP280, raw.pressure, fine_temp);
    
    baro_data_t data;
    data.temperature = (float)temp_calculee / 100.0f;
    data.pressure = (float)press_calculee / 256.0f;
    
    return data;
}

static float calculerAltitude(float pression_actuelle, float pression_reference) {
    if (pression_actuelle <= 0.0f) return 0.0f;

    return 44330.0f * (1.0f - powf((pression_actuelle / pression_reference), 0.1902949f));
}

void calibrerPressionAuSol(void) {
    float somme_pression = 0.0f;
    
    for(int i = 0; i < 20; i++) {
        baro_data_t donnees = obtenirDonneesCompensees();
        somme_pression += donnees.pressure;
        HAL_Delay(10); 
    }
    
    pression_au_sol = somme_pression / 20.0f;
}

float obtenirAltitudeRelative(void) {
    baro_data_t donnees = obtenirDonneesCompensees();
    return calculerAltitude(donnees.pressure, pression_au_sol);
}