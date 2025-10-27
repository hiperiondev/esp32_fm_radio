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

/**
 * \def HIGH_VOL_THRES
 * Threshold value for detecting high volume levels.
 */
#define HIGH_VOL_THRES (65000)

/**
 * \def LOW_VOL_THRES
 * Threshold value for detecting low volume levels, set to half of HIGH_VOL_THRES.
 */
#define LOW_VOL_THRES (HIGH_VOL_THRES / 2)

/**
 * \def NO_VOL_THRES
 * Threshold value for detecting near-silent audio levels.
 */
#define NO_VOL_THRES (4096)

/**
 * \def STEP_DOWN_SIZE
 * Step size for decreasing gain in AGC, where each step is 2^(-STEP_DOWN_SIZE).
 */
#define STEP_DOWN_SIZE (5)

/**
 * \def STEP_UP_SIZE
 * Step size for increasing gain in AGC, where each step is 2^(-STEP_UP_SIZE).
 */
#define STEP_UP_SIZE (5)

/**
 * \enum polar_status_e
 * Enumeration defining various status flags for the polar modulator, used to indicate operational states like PTT activity or audio conditions.
 */
enum polar_status_e {
    PTT_ACTIVE = 0x00000001,     /**< Flag indicating PTT (Push-To-Talk) is active. */
    AGC_TRAINING = 0x00000002,   /**< Flag indicating AGC is in training mode without PTT. */
    AGC_FROZEN = 0x00000004,     /**< Flag indicating AGC is frozen, preventing adjustments. */
    AUDIO_SILENCE = 0x00000008,  /**< Flag indicating audio input is nearly silent. */
    AUDIO_LOW = 0x00000010,      /**< Flag indicating low audio level warning. */
    AUDIO_MIDLEVEL = 0x00000020, /**< Flag indicating medium audio peak levels. */
    AUDIO_OVF = 0x00000040,      /**< Flag indicating audio ADC overflow due to high signal. */
};

/**
 * \enum modulation_mode_e
 * Enumeration defining supported modulation modes for the polar modulator, including various FM, SSB, AM, and test modes.
 */
typedef enum modulation_mode_e {
    MOD_FMN,  /**< FM Narrow mode with 2.5 kHz deviation. */
    MOD_LSB,  /**< Lower Sideband SSB mode. */
    MOD_USB,  /**< Upper Sideband SSB mode. */
    MOD_CW,   /**< Continuous Wave mode. */
    MOD_FM,   /**< FM mode with 5 kHz deviation. */
    MOD_AM,   /**< Amplitude Modulation mode. */
    MOD_FSK,  /**< Frequency Shift Keying mode. */
    MOD_CWR,  /**< Continuous Wave Reverse mode (not used). */
    MOD_FMW,  /**< FM Wide mode with 75 kHz deviation for broadcast. */
    MOD_TEST, /**< Test mode for FSK reverse. */
} modulation_mode_t;

/**
 * \enum SPECIAL_MODULATION_E
 * Enumeration defining special modulation types for testing and specific signal generations, extending beyond standard modes.
 */
typedef enum SPECIAL_MODULATION_E {
    SPECIAL_MODULATION_NORMAL,          /**< Normal modulation operation. */
    SPECIAL_MODULATION_ATT_FIX_0,       /**< Fixed attenuation at 0 dB. */
    SPECIAL_MODULATION_ATT_FIX_10,      /**< Fixed attenuation at 10 dB. */
    SPECIAL_MODULATION_FM_FIX_0,        /**< Fixed FM at 0 deviation. */
    SPECIAL_MODULATION_FM_FRAC1STEP,    /**< FM with fractional 1-step deviation. */
    SPECIAL_MODULATION_2_TONE_SIG,      /**< 2-tone test signal. */
    SPECIAL_MODULATION_1_TONE_SIG_SW,   /**< 1-tone switched signal. */
    SPECIAL_MODULATION_1_TONE_SIG_500,  /**< 1-tone 500 Hz signal. */
    SPECIAL_MODULATION_2_TONE_SIG_IQ,   /**< 2-tone IQ signal. */
    SPECIAL_MODULATION_3_TONE_SIG_IQ,   /**< 3-tone IQ signal. */
    SPECIAL_MODULATION_RECT_FM_10,      /**< Rectangular FM at 10 Hz. */
    SPECIAL_MODULATION_RECT_FM_100,     /**< Rectangular FM at 100 Hz. */
    SPECIAL_MODULATION_RECT_FM_1000,    /**< Rectangular FM at 1000 Hz. */
    SPECIAL_MODULATION_FM_DIRECT,       /**< Direct FM modulation. */
    SPECIAL_MODULATION_AM_DIRECT,       /**< Direct AM modulation. */
    SPECIAL_MODULATION_AM_SIG500,       /**< AM with 500 Hz sine at 100% modulation. */
    SPECIAL_MODULATION_AM_SAWTOOTH,     /**< AM with sawtooth waveform. */
    SPECIAL_MODULATION_AM_RECT_100,     /**< AM rectangular at 100 Hz. */
    SPECIAL_MODULATION_AM_RECT_1000,    /**< AM rectangular at 1000 Hz. */
    SPECIAL_MODULATION_AM_3STEP,        /**< AM with 3-step levels. */
    SPECIAL_MODULATION_PLL_WR_FREEZE,   /**< Freeze PLL write operations. */
    SPECIAL_MODULATION_PLL_WR_RESTART,  /**< Restart PLL write operations. */
    SPECIAL_MODULATION_FM_SIG500,       /**< FM with 500 Hz sine. */
    SPECIAL_MODULATION_AM_SIG500_MOD50, /**< AM with 500 Hz sine at 50% modulation. */
} special_modulation_t;

/**
 * \enum FILTERS_PRE_LP_E
 * Enumeration defining pre-processing low-pass filter options for audio input.
 */
typedef enum FILTERS_PRE_LP_E {
    FILTER_NONE,         /**< No low-pass filter. */
    FILTER_LP_3000_2pol, /**< 2-pole low-pass at 3000 Hz. */
    FILTER_LP_3400_2pol, /**< 2-pole low-pass at 3400 Hz. */
    FILTER_LP_3000_4pol, /**< 4-pole low-pass at 3000 Hz. */
    FILTER_LP_3400_4pol, /**< 4-pole low-pass at 3400 Hz. */
} filter_pre_lp_t;

/**
 * \enum FILTERS_PRE_HP_E
 * Enumeration defining pre-processing high-pass filter options for audio input.
 */
typedef enum FILTERS_PRE_HP_E {
    FILTER_HP_NONE,     /**< No high-pass filter. */
    FILTER_HP_200_4pol, /**< 4-pole high-pass at 200 Hz. */
    FILTER_HP_300_4pol, /**< 4-pole high-pass at 300 Hz. */
    FILTER_HP_300_2pol, /**< 2-pole high-pass at 300 Hz. */
} filter_pre_hp_t;

/**
 * \enum FILTERS_PRE_PB_E
 * Enumeration defining pre-processing passband shaping filters.
 */
typedef enum FILTERS_PRE_PB_E {
    FILTER_PB_NONE, /**< No passband filter. */
    FILTER_PB_500,  /**< Passband starting at 500 Hz. */
    FILTER_PB_1k,   /**< Passband starting at 1000 Hz. */
    FILTER_PB_2k,   /**< Passband starting at 2000 Hz. */
} filter_pre_pb_t;

/**
 * \enum FILTER_POST_LP_E
 * Enumeration defining post-limiter low-pass filter options.
 */
typedef enum FILTER_POST_LP_E {
    FILTER_POST_LP_NONE,      /**< No post low-pass filter. */
    FILTER_POST_LP_3000_2pol, /**< 2-pole post low-pass at 3000 Hz. */
    FILTER_POST_LP_3400_2pol, /**< 2-pole post low-pass at 3400 Hz. */
    FILTER_POST_LP_3000_4pol, /**< 4-pole post low-pass at 3000 Hz. */
    FILTER_POST_LP_3400_4pol, /**< 4-pole post low-pass at 3400 Hz. */
} filter_post_lp_t;

/**
 * \enum AGC_TYPE_E
 * Enumeration defining AGC (Automatic Gain Control) types.
 */
typedef enum AGC_TYPE_E {
    AGC_NORMAL,      /**< Standard AGC operation. */
    AGC_PLUS_10_DB,  /**< AGC with +10 dB boost. */
    AGC_GAIN_FIX,    /**< Fixed gain AGC. */
    AGC_GAIN_CHANGE, /**< AGC with dynamic gain changes. */
} agc_type_t;

/**
 * \struct polar_mod_ctx_t
 * Structure holding the internal state and delay lines for the polar modulator, including AGC parameters and filter delays.
 */
typedef struct {
    int gain_value;            /**< Current AGC gain value (fixed-point). */
    int max_ampl;              /**< Maximum amplitude tracked for AGC. */
    int n;                     /**< Sample counter for AGC updates. */
    int cnt_high_volume_peaks; /**< Count of high volume peaks. */
#ifdef DEBUG_PC2_AGC
    int cnt_high_volume_event; /**< Count of high volume events (debug). */
#endif
    int cnt_low_volume_event; /**< Count of low volume events. */
    int cnt_no_volume_event;  /**< Count of no volume events. */
    int delay_hp500[2];       /**< Delay for 500 Hz high-pass filter. */
    int delay_hp1000[2];      /**< Delay for 1000 Hz high-pass filter. */
    int delay_hp2000[2];      /**< Delay for 2000 Hz high-pass filter. */
    int delay_hp200_s1[2];    /**< Stage 1 delay for 200 Hz high-pass. */
    int delay_hp200_s2[2];    /**< Stage 2 delay for 200 Hz high-pass. */
    int delay_hp300_s1[2];    /**< Stage 1 delay for 300 Hz high-pass. */
    int delay_hp300_s2[2];    /**< Stage 2 delay for 300 Hz high-pass. */
    int delay_hp300_2p[2];    /**< Delay for 2-pole 300 Hz high-pass. */
    int delay_i0;             /**< In-phase delay for Hilbert transform. */
    int delay_i1[2];          /**< In-phase stage 1 delay for Hilbert. */
    int delay_i2[2];          /**< In-phase stage 2 delay for Hilbert. */
    int delay_i3[2];          /**< In-phase stage 3 delay for Hilbert. */
    int delay_i4[2];          /**< In-phase stage 4 delay for Hilbert. */
    int delay_q1[2];          /**< Quadrature stage 1 delay for Hilbert. */
    int delay_q2[2];          /**< Quadrature stage 2 delay for Hilbert. */
    int delay_q3[2];          /**< Quadrature stage 3 delay for Hilbert. */
    int delay_q4[2];          /**< Quadrature stage 4 delay for Hilbert. */
    int delay_s1[2];          /**< General stage 1 delay. */
    int delay_s2[2];          /**< General stage 2 delay. */
    unsigned int last_mode;   /**< Last modulation mode for signal generator. */
    unsigned int counter;     /**< Counter for lookup tables in signal generator. */
    int delay_lp_adc[4];      /**< Delay for ADC low-pass filter. */
    int delay_lp_2[4];        /**< Delay for secondary low-pass filter. */
    int delay_lp_x[4];        /**< Delay for X (I) low-pass in post-Hilbert. */
    int delay_lp_y[4];        /**< Delay for Y (Q) low-pass in post-Hilbert. */
    int agc_gain;             /**< AGC gain factor. */
    int last_angle;           /**< Last computed angle for phase difference. */
} polar_mod_ctx_t;

/**
 * \struct modulation_t
 * Structure defining the configuration for modulation, including mode, filters, AGC type, and status.
 */
typedef struct {
    modulation_mode_t modulation_mode;       /**< Selected modulation mode. */
    filter_pre_hp_t filter_pre_hp;           /**< Pre-high-pass filter type. */
    filter_pre_lp_t filter_pre_lp;           /**< Pre-low-pass filter type. */
    filter_pre_pb_t filter_pre_pb;           /**< Pre-passband filter type. */
    filter_post_lp_t filter_post_lp;         /**< Post-low-pass filter type. */
    agc_type_t agc_type;                     /**< AGC type. */
    special_modulation_t special_modulation; /**< Special modulation type. */
    uint32_t polar_status;                   /**< Current polar status flags. */
} modulation_t;

/**
 * \brief Initializes the polar modulator context.
 *
 * Sets all members to zero and initializes specific values like gain and mode.
 *
 * \param[in,out] ctx Pointer to the polar_mod_ctx_t structure to initialize.
 */
void polar_mod_init(polar_mod_ctx_t *ctx);

/**
 * \brief Performs amplitude and phase modulation on a single audio sample.
 *
 * Processes the input data through filters, AGC, limiter, Hilbert transform, and CORDIC to produce amplitude and phase difference outputs based on the
 * modulation configuration.
 *
 * \param[in,out] ctx Pointer to the polar_mod_ctx_t context.
 * \param[in] modulation Modulation configuration.
 * \param[in] data Input audio sample.
 * \param[out] ampl_out Pointer to store the output amplitude.
 * \param[out] phase_diff_out Pointer to store the phase difference.
 * \return 0 on success, -1 on error (e.g., null pointers or invalid mode).
 */
int modulation_am_pm(polar_mod_ctx_t *ctx, modulation_t modulation, int data, int *ampl_out, int *phase_diff_out);

#endif /* POLAR_MOD_H_ */
