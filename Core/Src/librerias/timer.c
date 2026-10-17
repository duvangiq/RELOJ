/*
 * timer.c
 *
 *  Created on: May 1, 2025
 *      Author: ALCIDES_RAMOS
 */
#include "timer.h"

uint32_t pasos1;
uint32_t tmicros;
float tmilis;

// Callback de periodo de timer
// NOTA: Para muestreo de guitarra a 10 kHz, el procesamiento I2C del IMU
// se realiza en el while(1) principal para no colgar el microcontrolador.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
}

void TIMER_PROCEDIMIENTOS()
{
    // Lectura y filtros del MPU-6050
    MPU6050_lee_datos();
    MPU6050_calcula_flotantes();
    MPU6050_Ace_angulos();
    MPU6050_Giro_angulos();
    Filtro_Complementario(0.98f);
    filtro_kalman();
}

void TIMER_INIT()
{
    // Si se requiere iniciar el timer en modo base (sin interrupción de periodo)
    HAL_TIM_Base_Start(&htim_INT);
}

// Medir tiempo de ejecución
void TIMER_MIDE()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    pasos1 = (HAL_RCC_GetSysClockFreq() / 1000000);

    DWT->CYCCNT = 0;
    TIMER_PROCEDIMIENTOS();
    tmicros = (DWT->CYCCNT / pasos1);
    tmilis = tmicros / 1000.0f;
}
