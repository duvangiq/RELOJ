/*
 * dsp_effects.c
 *
 * Algoritmos DSP de audio en tiempo real:
 * - Clean: Ganancia lineal
 * - Overdrive: Saturación suave (soft-clipping cúbico estilo válvulas)
 * - Distorsión: Recorte rígido simétrico (hard-clipping)
 * - Trémolo: Modulación LFO senoidal controlada dinámicamente por el MPU-6050
 */

#include "dsp_effects.h"

#define SAMPLE_RATE_HZ   10000.0f
#define DC_OFFSET        2048.0f
#define TWO_PI           6.28318530718f

DSP_Engine_t dsp_engine;

void DSP_Init(void)
{
    dsp_engine.active_effect = EFFECT_OVERDRIVE; // Efecto por defecto
    dsp_engine.gain = 2.5f;
    dsp_engine.volume = 0.9f;
    dsp_engine.tremolo_rate_hz = 5.0f;
    dsp_engine.tremolo_depth = 0.75f;
    dsp_engine.tremolo_phase = 0.0f;
}

void DSP_SetEffect(GuitarEffect_t effect)
{
    dsp_engine.active_effect = effect;
}

#define PEDAL_MAX_ANGLE_DEG  22.0f

// Mapeo del sensor MPU-6050 al motor DSP (Pedal de Expresión Físico / Balancín)
// Eje único: Pitch (cabeceo hacia arriba / abajo de talón a punta)
void DSP_UpdateFromMPU(float roll_deg, float pitch_deg)
{
    (void)roll_deg; // Ignorado intencionalmente: pedal físico de 1 solo grado de libertad

    float abs_pitch = fabsf(pitch_deg);
    if (abs_pitch > PEDAL_MAX_ANGLE_DEG) abs_pitch = PEDAL_MAX_ANGLE_DEG;

    // Normalizado de 0.0 (reposo/talón) a 1.0 (a fondo/punta pisada)
    float pedal_pos = abs_pitch / PEDAL_MAX_ANGLE_DEG;

    switch (dsp_engine.active_effect)
    {
        case EFFECT_CLEAN:
            // Pedal de Volumen limpio (35% a 100%)
            dsp_engine.volume = 0.35f + (pedal_pos * 0.65f);
            break;

        case EFFECT_OVERDRIVE:
            // Pedal de Ganancia/Drive (1.5x a 7.5x)
            dsp_engine.gain = 1.5f + (pedal_pos * 6.0f);
            dsp_engine.volume = 0.90f;
            break;

        case EFFECT_DISTORTION:
            // Pedal de Distorsión pesada (2.0x a 9.0x)
            dsp_engine.gain = 2.0f + (pedal_pos * 7.0f);
            dsp_engine.volume = 0.85f;
            break;

        case EFFECT_TREMOLO:
            // Pedal de Velocidad de Trémolo (1.5 Hz a 13.0 Hz)
            dsp_engine.tremolo_rate_hz = 1.5f + (pedal_pos * 11.5f);
            dsp_engine.volume = 0.90f;
            break;

        default:
            break;
    }
}

#define DC_IN_OFFSET     2540.0f
#define DC_OUT_OFFSET    2048.0f

// Procesa un bloque de muestras de audio (256 muestras a 10 kHz)
void DSP_ProcessBlock(const uint16_t *input_samples, uint16_t *output_samples, uint16_t length)
{
    float phase_step = (TWO_PI * dsp_engine.tremolo_rate_hz) / SAMPLE_RATE_HZ;

    for (uint16_t i = 0; i < length; i++)
    {
        // 1. Quitar el nivel DC fijo de la baquelita para obtener la señal AC pura (-1.0 a +1.0)
        float x = ((float)input_samples[i] - DC_IN_OFFSET) / 2048.0f;
        float y = x;

        // 2. Aplicar el algoritmo según el efecto activo
        switch (dsp_engine.active_effect)
        {
            case EFFECT_CLEAN:
                y = x * dsp_engine.volume;
                break;

            case EFFECT_OVERDRIVE:
            {
                // Soft-clipping polinómico cálido (estilo amplificador a tubos)
                float driven = x * dsp_engine.gain;
                if (driven > 1.0f) {
                    y = 2.0f / 3.0f;
                } else if (driven < -1.0f) {
                    y = -2.0f / 3.0f;
                } else {
                    y = driven - (driven * driven * driven / 3.0f);
                }
                y *= dsp_engine.volume * 1.3f;
                break;
            }

            case EFFECT_DISTORTION:
            {
                // Hard-clipping simétrico agresivo
                float boosted = x * dsp_engine.gain * 2.0f;
                const float threshold = 0.45f;
                if (boosted > threshold) {
                    y = threshold;
                } else if (boosted < -threshold) {
                    y = -threshold;
                } else {
                    y = boosted;
                }
                y = (y / threshold) * dsp_engine.volume * 0.8f;
                break;
            }

            case EFFECT_TREMOLO:
            {
                // Modulación senoidal periódica (LFO)
                float lfo = 0.5f * (1.0f + sinf(dsp_engine.tremolo_phase));
                float mod = (1.0f - dsp_engine.tremolo_depth) + (dsp_engine.tremolo_depth * lfo);
                y = x * mod * dsp_engine.volume;

                dsp_engine.tremolo_phase += phase_step;
                if (dsp_engine.tremolo_phase >= TWO_PI) {
                    dsp_engine.tremolo_phase -= TWO_PI;
                }
                break;
            }

            default:
                y = x;
                break;
        }

        // 3. Re-polarizar exactamente al centro de 12 bits (2048)
        float out_val = (y * 2048.0f) + 2048.0f;
        if (out_val < 0.0f) out_val = 0.0f;
        if (out_val > 4095.0f) out_val = 4095.0f;

        output_samples[i] = (uint16_t)out_val;
    }
}
