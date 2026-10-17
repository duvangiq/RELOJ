/*
 * dsp_effects.h
 *
 * Motor de Efectos Digitales para Guitarra (DSP)
 * Compatible con STM32F4 (BlackPill) y STM32H5 (250 MHz)
 */

#ifndef INC_DSP_EFFECTS_H_
#define INC_DSP_EFFECTS_H_

#include "main.h"
#include <stdint.h>
#include <math.h>

// Modos de efectos disponibles
typedef enum {
    EFFECT_CLEAN = 0,
    EFFECT_OVERDRIVE,
    EFFECT_DISTORTION,
    EFFECT_TREMOLO
} GuitarEffect_t;

// Estructura de parámetros del motor DSP
typedef struct {
    GuitarEffect_t active_effect; // Efecto actualmente seleccionado
    float gain;                   // Ganancia de entrada (1.0 a 10.0)
    float volume;                 // Volumen maestro de salida (0.1 a 1.0)
    float tremolo_rate_hz;        // Velocidad del trémolo (1.0 Hz a 15.0 Hz)
    float tremolo_depth;          // Profundidad del trémolo (0.0 a 1.0)
    float tremolo_phase;          // Fase acumulada del LFO
} DSP_Engine_t;

extern DSP_Engine_t dsp_engine;

// Prototipos de funciones
void DSP_Init(void);
void DSP_SetEffect(GuitarEffect_t effect);
void DSP_UpdateFromMPU(float roll_deg, float pitch_deg);
void DSP_ProcessBlock(const uint16_t *input_samples, uint16_t *output_samples, uint16_t length);

#endif /* INC_DSP_EFFECTS_H_ */
