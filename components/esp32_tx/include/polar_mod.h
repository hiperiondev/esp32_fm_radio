/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/esp32_fm_radio *
 *
 * This is based on other projects:
 *    ESP32 as FM radio transmitter: https://github.com/Alexxdal/ESP32FMRadio
 *    SSB/CW/FM signal generator 35 - 4400MHz: https://gitlab.com/dg6rs/polar
 *
 *    please contact their authors for more information.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef POLAR_MOD_H_
#define POLAR_MOD_H_
#include <stdint.h>

// Microphone AGC (automated gain control)
#define HIGH_VOL_THRES (65000)              // threshold for a volume to be loud
#define LOW_VOL_THRES  (HIGH_VOL_THRES / 2) // threshold for a volume to be not loud
#define NO_VOL_THRES   (4096)               // threshold for a volume to be almost silent (1/16 of max amplitude, so ca. -24dB)
#define STEP_DOWN_SIZE (5)                  // every step size is 2^^-4  (=6,25%)  -> higher number means lower step size !!
#define STEP_UP_SIZE   (5)                  // every step size is 2^^-6  (=1,6%)

enum polar_status_e {
    PTT_ACTIVE = 0x00000001,
    AGC_TRAINING = 0x00000002,   // AGC is active without the PTT
    AGC_FROZEN = 0x00000004,     // AGC is frozen even if PTT is active (TODO: AGC active or frozen? Which one has priority?)
    AUDIO_SILENCE = 0x00000008,  // audio input is so low that the output power is deactivated
    AUDIO_LOW = 0x00000010,      // audio level is low (warning for the user)
    AUDIO_MIDLEVEL = 0x00000020, // audio peaks at a medium level (-6dB?!)
    AUDIO_OVF = 0x00000040,      // audio ADC get too high level of signal
};

typedef enum modulation_mode_e {
    MOD_FMN,  // FM-Narrow 2,5kHz
    MOD_LSB,  // LSB
    MOD_USB,  // USB
    MOD_CW,   // CW
    MOD_FM,   // FM 5kHz max. frequency deviation
    MOD_AM,   // AM
    MOD_FSK,  // FSK
    MOD_CWR,  // according to TS-480, not used here
    MOD_FMW,  // FM-Wide 75kHz (FM Radio 87-108MHz)
    MOD_TEST, // FSK-reverse
} modulation_mode_t;

// SPECIAL_MODULATION
typedef enum SPECIAL_MODULATION_E {
    SPECIAL_MODULATION_NORMAL,          //
    SPECIAL_MODULATION_ATT_FIX_0,       //
    SPECIAL_MODULATION_ATT_FIX_10,      //
    SPECIAL_MODULATION_FM_FIX_0,        //
    SPECIAL_MODULATION_FM_FRAC1STEP,    //
    SPECIAL_MODULATION_2_TONE_SIG,      //
    SPECIAL_MODULATION_1_TONE_SIG_SW,   //
    SPECIAL_MODULATION_1_TONE_SIG_500,  //
    SPECIAL_MODULATION_2_TONE_SIG_IQ,   //
    SPECIAL_MODULATION_3_TONE_SIG_IQ,   //
    SPECIAL_MODULATION_RECT_FM_10,      //
    SPECIAL_MODULATION_RECT_FM_100,     //
    SPECIAL_MODULATION_RECT_FM_1000,    //
    SPECIAL_MODULATION_FM_DIRECT,       //
    SPECIAL_MODULATION_AM_DIRECT,       //
    SPECIAL_MODULATION_AM_SIG500,       // AM with 500Hz sinus, with 100% modulation level
    SPECIAL_MODULATION_AM_SAWTOOTH,     // TODO: doesn't make a sawtooth at all?
    SPECIAL_MODULATION_AM_RECT_100,     //
    SPECIAL_MODULATION_AM_RECT_1000,    //
    SPECIAL_MODULATION_AM_3STEP,        // 3 steps for dig att directly
    SPECIAL_MODULATION_PLL_WR_FREEZE,   // after a freeze, send a "SPECIAL_MODULATION_PLL_WR_RESTART" to restart the PLL access via SPI  !!!
    SPECIAL_MODULATION_PLL_WR_RESTART,  //
    SPECIAL_MODULATION_FM_SIG500,       // FM with 500Hz sinus
    SPECIAL_MODULATION_AM_SIG500_MOD50, // AM with 500Hz sinus, with 50% modulation level
} special_modulation_t;

// Audio filter
typedef enum FILTERS_PRE_LP_E {
    FILTER_NONE,         //
    FILTER_LP_3000_2pol, //
    FILTER_LP_3400_2pol, //
    FILTER_LP_3000_4pol, //
    FILTER_LP_3400_4pol, // default
} filter_pre_lp_t;

typedef enum FILTERS_PRE_HP_E {
    FILTER_HP_NONE,     //
    FILTER_HP_200_4pol, //
    FILTER_HP_300_4pol, //
    FILTER_HP_300_2pol, // default
} filter_pre_hp_t;

typedef enum FILTERS_PRE_PB_E {
    FILTER_PB_NONE, //
    FILTER_PB_500,  //
    FILTER_PB_1k,   //
    FILTER_PB_2k,   //
} filter_pre_pb_t;

// Low pass after soft limiter
typedef enum FILTER_POST_LP_E {
    FILTER_POST_LP_NONE,      //
    FILTER_POST_LP_3000_2pol, //
    FILTER_POST_LP_3400_2pol, // default
    FILTER_POST_LP_3000_4pol, //
    FILTER_POST_LP_3400_4pol, //
} filter_post_lp_t;

typedef enum AGC_TYPE_E {
    AGC_NORMAL,
    AGC_PLUS_10_DB,
    AGC_GAIN_FIX,    // 108 -> gain=0.5  / 110 -> gain=1 / 128 -> gain=2.5  (usual range: 2..5)
    AGC_GAIN_CHANGE, // 208 -> gain=0.5  / 110 -> gain=1 / 128 -> gain=2.5  compared to the usual AGC gain
} agc_type_t;

typedef struct {
    // From mic_agc_fast
    int gain_value;
    int max_ampl;
    int n;
    int cnt_high_volume_peaks;
#ifdef DEBUG_PC2_AGC
    int cnt_high_volume_event;
#endif
    int cnt_low_volume_event;
    int cnt_no_volume_event;

    // From high-pass filters (internal delays)
    int delay_hp500[2];    // For filter_1pol_highpass_500hz
    int delay_hp1000[2];   // For filter_1pol_highpass_1000hz
    int delay_hp2000[2];   // For filter_1pol_highpass_2000hz
    int delay_hp200_s1[2]; // For filter_4pol_highpass_200hz stage 1
    int delay_hp200_s2[2]; // For filter_4pol_highpass_200hz stage 2
    int delay_hp300_s1[2]; // For filter_4pol_highpass_300hz stage 1
    int delay_hp300_s2[2]; // For filter_4pol_highpass_300hz stage 2
    int delay_hp300_2p[2]; // For filter_2pol_highpass_300hz

    // From hilbert
    int delay_i0;
    int delay_i1[2];
    int delay_i2[2];
    int delay_i3[2];
    int delay_i4[2];
    int delay_q1[2];
    int delay_q2[2];
    int delay_q3[2];
    int delay_q4[2];
    int delay_s1[2];
    int delay_s2[2];

    // From iq_signal_generator
    unsigned int last_mode;
    unsigned int counter;

    // From modulation_am_pm
    int delay_lp_adc[4];
    int delay_lp_2[4];
    int delay_lp_x[4];
    int delay_lp_y[4];
    int agc_gain;
    int last_angle;
} polar_mod_ctx_t;

typedef struct {
    modulation_mode_t modulation_mode;       //
    filter_pre_hp_t filter_pre_hp;           //
    filter_pre_lp_t filter_pre_lp;           //
    filter_pre_pb_t filter_pre_pb;           //
    filter_post_lp_t filter_post_lp;         //
    agc_type_t agc_type;                     //
    special_modulation_t special_modulation; //
    uint32_t polar_status;                   //
} modulation_t;

void polar_mod_init(polar_mod_ctx_t *ctx);
int modulation_am_pm(polar_mod_ctx_t *ctx, modulation_t modulation, int data, int *ampl_out, int *phase_diff_out);

#endif /* POLAR_MOD_H_ */
