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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "polar_mod.h"

/**
 * \def CORDIC_ITERATIONS
 * Number of iterations for CORDIC algorithm.
 */
#define CORDIC_ITERATIONS 8

/**
 * \def LOOKUP_SIZE
 * Size of lookup tables for tone signals.
 */
#define LOOKUP_SIZE 32

/**
 * \brief 3-tone X (I) lookup table for test signals.
 */
static int table_3_tone_x[LOOKUP_SIZE] = {
    0,      //
    14700,  //
    25483,  //
    29586,  //
    26182,  //
    16573,  //
    3713,   //
    -8750,  //
    -17600, //
    -20996, //
    -18915, //
    -12991, //
    -5818,  //
    22,     //
    2856,   //
    2454,   //
    0,      //
    -2454,  //
    -2856,  //
    -22,    //
    5818,   //
    12991,  //
    18915,  //
    20996,  //
    17600,  //
    8750,   //
    -3713,  //
    -16573, //
    -26182, //
    -29586, //
    -25483, //
    -14700  //
};

/**
 * \brief 3-tone Y (Q) lookup table for test signals.
 */
static int table_3_tone_y[LOOKUP_SIZE] = {
    30400,  //
    26516,  //
    15958,  //
    1671,   //
    -12445, //
    -22704, //
    -26708, //
    -23983, //
    -16000, //
    -5581,  //
    4081,   //
    10459,  //
    12445,  //
    10575,  //
    6669,   //
    3048,   //
    1600,   //
    3048,   //
    6669,   //
    10575,  //
    12445,  //
    10459,  //
    4081,   //
    -5581,  //
    -16000, //
    -23983, //
    -26708, //
    -22704, //
    -12445, //
    1671,   //
    15958,  //
    26516   //
};

/**
 * \brief 2-tone X (I) lookup table for test signals.
 */
static int table_2_tone_x[LOOKUP_SIZE] = {
    0,      //
    15012,  //
    26096,  //
    30475,  //
    27314,  //
    17904,  //
    5191,   //
    -7181,  //
    -16000, //
    -19426, //
    -17437, //
    -11661, //
    -4686,  //
    910,    //
    3468,   //
    2766,   //
    0,      //
    -2766,  //
    -3468,  //
    -910,   //
    4686,   //
    11661,  //
    17437,  //
    19426,  //
    16000,  //
    7181,   //
    -5191,  //
    -17904, //
    -27314, //
    -30475, //
    -26096, //
    -15012  //
};

/**
 * \brief 2-tone Y (Q) lookup table for test signals.
 */
static int table_2_tone_y[LOOKUP_SIZE] = {
    32000,  //
    28086,  //
    17437,  //
    3001,   //
    -11314, //
    -21815, //
    -26096, //
    -23671, //
    -16000, //
    -5893,  //
    3468,   //
    9570,   //
    11314,  //
    9244,   //
    5191,   //
    1479,   //
    0,      //
    1479,   //
    5191,   //
    9244,   //
    11314,  //
    9570,   //
    3468,   //
    -5893,  //
    -16000, //
    -23671, //
    -26096, //
    -21815, //
    -11314, //
    3001,   //
    17437,  //
    28086   //
};

/**
 * \brief Fast microphone AGC computation.
 *
 * Adjusts gain based on amplitude peaks to normalize audio levels, updating every 400 samples (25 ms at 16 kHz).
 *
 * \param[in,out] ctx Pointer to the polar_mod_ctx_t context.
 * \param[in] ampl Current amplitude value.
 * \param[in] polar_status Current polar status flags.
 * \return Updated gain value (fixed-point, 256 = 1.0).
 */
static int mic_agc_fast(polar_mod_ctx_t *ctx, int ampl, uint32_t polar_status) {
    int flag_agc_active;

    flag_agc_active = ((polar_status & PTT_ACTIVE) || (polar_status & AGC_TRAINING)) && (!(polar_status & AGC_FROZEN)); // self explanatory, obviously

    if ((ampl > ctx->max_ampl) && flag_agc_active)
        ctx->max_ampl = ampl; // find max peak amplitude
    if ((ampl > HIGH_VOL_THRES) && flag_agc_active)
        ctx->cnt_high_volume_peaks++; // how often the max volume is exceeded?

    ctx->n++;

    // every x samples (=25ms at 16kSps)
    if (ctx->n > 400) {
        ctx->n = 0; // reset counter

        if (ctx->cnt_high_volume_peaks > 3)                                          // more than 3 peaks happened ?
            ctx->gain_value = ctx->gain_value - (ctx->gain_value >> STEP_DOWN_SIZE); // reduce gain

        if ((ctx->max_ampl < NO_VOL_THRES) && flag_agc_active)
            ctx->cnt_no_volume_event++;
        else
            ctx->cnt_no_volume_event = 0;

        if ((ctx->max_ampl < LOW_VOL_THRES) && (ctx->max_ampl > NO_VOL_THRES) && flag_agc_active)
            ctx->cnt_low_volume_event++;
        if ((ctx->max_ampl > LOW_VOL_THRES) && flag_agc_active)
            ctx->cnt_low_volume_event = 0; // only a loud event resets counter // a "no sound" event does not

        if (ctx->cnt_no_volume_event > 10)
            ctx->cnt_low_volume_event = 10; // if there was quite some "no volume" in a row, then put back the low-volume counter to half the way, so it would
                                            // not happen that "almost no sound" for a while pushes up the gain too much

        // more than x times in a row , use quite some delay so a word not that loud does not immediately pushes up the gain
        if (ctx->cnt_low_volume_event > 20) {
            ctx->gain_value = ctx->gain_value + (ctx->gain_value >> STEP_UP_SIZE); // increase gain
            polar_status |= AUDIO_LOW;
        } else {
            polar_status &= ~AUDIO_LOW; // Hmm... it's still quiet, even if the gain isn't increased!?
        }

        // more than 5 times in a row
        if (ctx->cnt_no_volume_event > 5) {
            polar_status |= AUDIO_SILENCE; // what else to do?
        } else {
            polar_status &= ~AUDIO_SILENCE; // what else to do?
        }

        if (ctx->gain_value < 64) {
            ctx->gain_value = 64; // smallest gain value: 0.25 (64/256) // what else to do?
        }

        if (ctx->gain_value > (1 << 15)) {
            ctx->gain_value = 1 << 15; // highest gain value: 1<<5 (32) i.e. needs ca. 1/32 /ca. -30dB) of full swing for full scale // what else ?
        }

        ctx->max_ampl = 0; // reset max value search
        ctx->cnt_high_volume_peaks = 0;

        // ADC gain (overdrive, underdrive too?): extra function? or converting AGC gain back to the ADC range. What about audio midlevel?
        // How can I display it properly?
    }

    return ctx->gain_value;
}

/**
 * \brief Applies a soft limiter to the input signal.
 *
 * Limits the signal to prevent hard clipping, using a cubic approximation for smooth compression.
 *
 * \param[in] x Input signal value.
 * \return Limited output value.
 */
static int soft_limiter(int x) {

    int x_sq; // x squared
    int x_tr; // x to the power of three
    int data_out;

    if (x > 53500)
        return 35676;
    if (x < -53500)
        return -35672; // saturation. Exact values were try&error

    x_sq = ((x >> 2) * (x >> 2)) >> 12;    // x_sq = x*x // TODO: still generates 2LSB (=4steps) discontinuous steps
    x_tr = ((x >> 2) * (x_sq >> 2)) >> 12; // x_tr = x*x_sq
    data_out = x - (x_tr >> 1);            // x - 1/2*x_tr

    return data_out;
}

/**
 * \brief Computes a biquad filter stage (transposed direct form II).
 *
 * General biquad filter implementation for IIR filters with fixed coefficients.
 *
 * \param[in] x Input sample.
 * \param[in] b1 b1 coefficient (fixed-point).
 * \param[in] a1 a1 coefficient (fixed-point).
 * \param[in] a2 a2 coefficient (fixed-point).
 * \param[in,out] delay Pointer to delay array (2 elements).
 * \return Filtered output.
 */
static int biquad(int x, int b1, int a1, int a2, int *delay) {
    int y;
    static const int64_t b0 = 1 << 16;
    static const int64_t b2 = 1 << 16; // GNU Octave butterworth lowass filter: b0,b1 and a0 always = 1

    y = (((int64_t)b0 * x) >> 16) + delay[1];
    delay[1] = (((int64_t)b1 * x - (int64_t)a1 * y) >> 16) + delay[0]; // here is "minus", so it fits to the polartiy of the coefficients from GNU octave
    delay[0] = (((int64_t)b2 * x - (int64_t)a2 * y) >> 16);

    return y;
}

/**
 * \brief Computes a biquad filter with b2=0 (for 1-pole filters).
 *
 * Specialized biquad for high-pass filters where b2 is zero.
 *
 * \param[in] x Input sample.
 * \param[in] b1 b1 coefficient (fixed-point).
 * \param[in] a1 a1 coefficient (fixed-point).
 * \param[in] a2 a2 coefficient (fixed-point).
 * \param[in,out] delay Pointer to delay array (2 elements).
 * \return Filtered output.
 */
static int biquad_b2zero(int x, int b1, int a1, int a2, int *delay) {
    int y;
    static const int64_t b0 = 1 << 16;

    y = (((int64_t)b0 * x) >> 16) + delay[1];
    delay[1] = (((int64_t)b1 * x - (int64_t)a1 * y) >> 16) + delay[0]; // here is "minus", so it fits to the polarity of the coefficients from GNU octave
    delay[0] = ((-(int64_t)a2 * y) >> 16);

    return y;
}

/**
 * \brief 2-pole Bessel low-pass filter at 3000 Hz (16 kHz sample rate).
 *
 * Implements a 2-pole low-pass filter using biquad with Bessel coefficients.
 *
 * \param[in] x Input sample.
 * \param[in,out] delay Pointer to delay array (2 elements).
 * \return Filtered output.
 */
static int filter_2pol_lowpass_3000hz_bessel(int x, int *delay) {
    int stage1;
    // int stage2;

    stage1 = biquad(x, 2 << 16, -27865, 7278, &delay[0]);
    stage1 = stage1 >> 3; // reduce the gain to approx. 0.73  ( = 1/(g*8) )

    return stage1;
}

/**
 * \brief 4-pole Bessel low-pass filter at 3000 Hz (16 kHz sample rate).
 *
 * Implements a 4-pole low-pass filter using two cascaded biquads with Bessel coefficients.
 *
 * \param[in] x Input sample.
 * \param[in,out] delay Pointer to delay array (4 elements).
 * \return Filtered output.
 */
static int filter_4pol_lowpass_3000hz_bessel(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -27521, 17143, &delay[0]);
    stage2 = biquad(stage1, 2 << 16, -30251, 4755, &delay[2]);
    stage2 = stage2 >> 5; // reduce the gain to approx. 0.97  ( = 1/(g*32) ). TODO: distribute the gain reduction in between the stages?

    return stage2;
}

/**
 * \brief 4-pole Butterworth low-pass filter at 3000 Hz (16 kHz sample rate).
 *
 * Implements a 4-pole low-pass filter using two cascaded biquads with Butterworth coefficients.
 *
 * \param[in] x Input sample.
 * \param[in,out] delay Pointer to delay array (4 elements).
 * \return Filtered output.
 */
static int filter_4pol_lowpass_3000hz(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -27061, 5178, &delay[0]);
    stage2 = biquad(stage1, 2 << 16, -37057, 31299, &delay[2]);
    stage2 = stage2 >> 4; // reduce the gain to approx. 1.65  ( = 1/(g*16) ). TODO: distribute the gain reduction in between the stages?

    return stage2;
}

/**
 * \brief 4-pole Butterworth low-pass filter at 3400 Hz (16 kHz sample rate).
 *
 * Implements a 4-pole low-pass filter using two cascaded biquads with Butterworth coefficients.
 *
 * \param[in] x Input sample.
 * \param[in,out] delay Pointer to delay array (4 elements).
 * \return Filtered output.
 */
static int filter_4pol_lowpass_3400hz(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -16118, 3509, &delay[0]);       // a1 = -0.245945*65536 = -16118  /  a2 = 0.053545*65536 =  3509
    stage2 = biquad(stage1, 2 << 16, -22300, 29900, &delay[2]); // a1 = -0.340272*65536 = -22300  /  a2 = 0.457609*65536 = 29990
    stage2 = stage2 >> 4;                                       // reduce the gain to approx. 1.12  ( = 1/(g*16) )
                                                                // TODO: distribute the gain reduction in between the stages?!??!?!?

    return stage2;
}

/**
 * \brief 2-pole Butterworth low-pass filter at 3400 Hz (16 kHz sample rate).
 *
 * Implements a 2-pole low-pass filter using biquad with Butterworth coefficients.
 *
 * \param[in] x Input sample.
 * \param[in,out] delay Pointer to delay array (2 elements).
 * \return Filtered output.
 */
static int filter_2pol_lowpass_3400hz(int x, int *delay) {
    int stage1;

    stage1 = biquad(x, 2 << 16, -18131, 12133, &delay[0]);
    stage1 = stage1 >> 2; //  reduce the gain to approx. 1.10  ( = 1/(g*4) )

    return stage1;
}

/**
 * \brief 1-pole Butterworth high-pass filter at 500 Hz (16 kHz sample rate).
 *
 * Implements a 1-pole high-pass filter using specialized biquad.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_1pol_highpass_500hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;

    stage1 = biquad_b2zero(x, -(1 << 16), -53784, 0, ctx->delay_s1); // b1,a1,a2

    // ignore g here, as it is close to one
    stage1 = stage1 + 0; // Manually calculate the DC offset of the high pass -> leave it as it is?
    return stage1;
}

/**
 * \brief 1-pole Butterworth high-pass filter at 1000 Hz (16 kHz sample rate).
 *
 * Implements a 1-pole high-pass filter using specialized biquad.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_1pol_highpass_1000hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;
    // static int delay_s1[2] = { 0, 0 };

    stage1 = biquad_b2zero(x, -(1 << 16), -43790, 0, ctx->delay_s1); // b1,a1,a2
    // ignore g here, as it is close to one
    stage1 = stage1 + 0; // manually calculate the DC offset of the high pass -> leave it like that?
    return stage1;
}

/**
 * \brief 1-pole Butterworth high-pass filter at 2000 Hz (16 kHz sample rate).
 *
 * Implements a 1-pole high-pass filter using specialized biquad.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_1pol_highpass_2000hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;

    stage1 = biquad_b2zero(x, -(1 << 16), -27146, 0, ctx->delay_s1); // b1,a1,a2
    // ignore g here, as it is close to one
    // stage1=stage1+0; // manually calculate the DC offset of the high pass -> it is virtually zero
    return stage1;
}

/**
 * \brief 4-pole Butterworth high-pass filter at 200 Hz (16 kHz sample rate).
 *
 * Implements a 4-pole high-pass filter using two cascaded biquads, with DC offset correction.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_4pol_highpass_200hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;
    int stage2;

    stage1 = biquad(x, -(2 << 16), -121837, 56677, ctx->delay_s1);      // b1,a1,a2
    stage2 = biquad(stage1, -(2 << 16), -126859, 61715, ctx->delay_s2); // b1,a1,a2
    // ignore g here, as it is close to one
    stage2 = stage2 + 168; // TODO: manually calculate the DC offset of the high pass -> leave it like that?
    return stage2;
}

/**
 * \brief 4-pole Butterworth high-pass filter at 300 Hz (16 kHz sample rate).
 *
 * Implements a 4-pole high-pass filter using two cascaded biquads, with DC offset correction.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_4pol_highpass_300hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;
    int stage2;

    stage1 = biquad(x, -(2 << 16), -117414, 52697, ctx->delay_s1);      // b1,a1,a2
    stage2 = biquad(stage1, -(2 << 16), -124561, 59894, ctx->delay_s2); // b1,a1,a2
    // ignore g here, as it is close to one
    stage2 = stage2 + 75; // manually calculate the DC offset of the high pass -> leave it like that?
    return stage2;
}

/**
 * \brief 2-pole Butterworth high-pass filter at 300 Hz (16 kHz sample rate).
 *
 * Implements a 2-pole high-pass filter using biquad, with DC offset correction.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] x Input sample.
 * \return Filtered output.
 */
static int filter_2pol_highpass_300hz(polar_mod_ctx_t *ctx, int x) {
    int stage1;

    stage1 = biquad(x, -(2 << 16), -120175, 55478, ctx->delay_s1); // b1,a1,a2
    // ignore g here, as it is close to one
    stage1 = stage1 + 78; // manually calculate the DC offset of the high pass -> leave it like that?
    return stage1;
}

/**
 * \brief Computes an all-pass filter stage for Hilbert transform.
 *
 * All-pass filter used in the Hilbert transform chain.
 *
 * \param[in] x Input sample.
 * \param[in] coeff Fixed-point coefficient.
 * \param[in,out] delay Pointer to delay array (2 elements).
 * \return Filtered output.
 */
static int allpass(int x, int coeff, int *delay) {
    int signal_top_right;
    int y;

    signal_top_right = ((int64_t)coeff * (delay[0] - x)) >> 16;
    y = signal_top_right + delay[0];
    delay[0] = delay[1];
    delay[1] = signal_top_right + x;

    return y;
}

/**
 * \brief Performs Hilbert transform to generate I and Q components.
 *
 * Uses cascaded all-pass filters to create in-phase (I) and quadrature (Q) signals from input.
 *
 * \param[in,out] ctx Pointer to context for delays.
 * \param[in] sample_in Input sample.
 * \param[out] i_out Pointer to in-phase output.
 * \param[out] q_out Pointer to quadrature output.
 */
static void hilbert(polar_mod_ctx_t *ctx, int sample_in, int *i_out, int *q_out) {
    int i1, i2, i3, q1, q2, q3;

    // inphase 1. to 4. Block
    i1 = allpass(ctx->delay_i0, 31418, ctx->delay_i1); // coeff = ( ( 0.6923877778065 ) ^ 2 ) << 16
    i2 = allpass(i1, 57434, ctx->delay_i2);            // coeff = ( ( 0.9360654322959 ) ^ 2 ) << 16
    i3 = allpass(i2, 64002, ctx->delay_i3);            // coeff = ( ( 0.9882295226860 ) ^ 2 ) << 16
    *i_out = allpass(i3, 65372, ctx->delay_i4);        // coeff = ( ( 0.9987488452737 ) ^ 2 ) << 16
    // inphase Z^-1 block
    ctx->delay_i0 = sample_in;

    // quadrature 1. to 4. Block
    q1 = allpass(sample_in, 10601, ctx->delay_q1); // coeff = ( ( 0.4021921162426 ) ^ 2 ) << 16
    q2 = allpass(q1, 48040, ctx->delay_q2);        // coeff = ( ( 0.8561710882420 ) ^ 2 ) << 16
    q3 = allpass(q2, 61954, ctx->delay_q3);        // coeff = ( ( 0.9722909545651 ) ^ 2 ) << 16
    *q_out = allpass(q3, 64920, ctx->delay_q4);    // coeff = ( ( 0.9952884791278 ) ^ 2 ) << 16
}

/**
 * \brief Computes magnitude and angle using CORDIC algorithm.
 *
 * Rotates the vector to compute absolute value and phase angle in fixed-point.
 *
 * \param[in] x X coordinate (I).
 * \param[in] y Y coordinate (Q).
 * \param[out] out_abs Pointer to magnitude output.
 * \param[out] out_angle Pointer to angle output (scaled: 2^24 = 360 degrees).
 */
static void cordic(int x, int y, int *out_abs, int *out_angle) {
    // AngTable in degrees = 45, 26.565,14.036, 7.125, 3.576,1.790,0.895,0.448
    static const int angtable[CORDIC_ITERATIONS] = {
        2097152, //
        1238019, //
        654125,  //
        332049,  //
        166654,  //
        83420,   //
        41710,   //
        20878    //
    }; // 2^24=16777216 corresponds to 360° / 46603 -> 1°  / 20878 -> 0.448° (last step)
    int loopnum;
    int x_new, y_new, sumangle;
    int x_shift;
    int flag_neg_x;

    if (x < 0) // is the point in the 2. or 3. quadrant ?
    {
        x = -x;         // shift point to 1. or 4. quadrant (positive x value)
        flag_neg_x = 1; // remember in flag to shift back later
    } else {
        flag_neg_x = 0;
    }

    sumangle = 0;
    for (loopnum = 0; loopnum < CORDIC_ITERATIONS; loopnum++) {
        if (x >= 0)
            x_shift = x >> loopnum;
        else
            x_shift = -(
                (-x) >>
                loopnum); // it is necessary to shift a positive number, otherwise result can be off by one / -1 would be the end result if you do this: (-7)>>6

        if (y >= 0) {
            x_new = x + (y >> loopnum);
            y_new = y - x_shift;
            sumangle = sumangle + angtable[loopnum];
        } else {
            x_new = x + ((-y) >> loopnum); // the "plus" is a double minus
            y_new = y + x_shift;
            sumangle = sumangle - angtable[loopnum];
        }

        x = x_new;
        y = y_new;
    }

    if (flag_neg_x == 1) // if x was negative: angle = 180° - angle
    {
        sumangle = 8388608 - sumangle; // 16777216 = 360°
    }

    *out_angle = sumangle; // value of 2^24 = 16777216 corresponds to 360°  // output range: -180...+180°
    *out_abs = x_new;      // proportional to absolute value, with factor 0.60726
    return;
}

/**
 * \brief Generates IQ test signals using lookup tables.
 *
 * Cycles through precomputed tables for 2-tone or 3-tone signals based on mode.
 *
 * \param[in,out] ctx Pointer to context for state.
 * \param[in] mode Special modulation mode for signal type.
 * \param[out] x Pointer to X (I) output.
 * \param[out] y Pointer to Y (Q) output.
 */
static void iq_signal_generator(polar_mod_ctx_t *ctx, int mode, int *x, int *y) {
    // reset counter when mode changes
    if (ctx->last_mode != mode) {
        ctx->last_mode = mode;
        ctx->counter = 0;
    }

    ctx->counter++;
    if (ctx->counter >= LOOKUP_SIZE)
        ctx->counter = 0;

    if (mode == SPECIAL_MODULATION_2_TONE_SIG_IQ) {
        *x = table_2_tone_x[ctx->counter];
        *y = table_2_tone_y[ctx->counter];
    } else if (mode == SPECIAL_MODULATION_3_TONE_SIG_IQ) {
        *x = table_3_tone_x[ctx->counter];
        *y = table_3_tone_y[ctx->counter];
    } else {
        *x = 0;
        *y = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int modulation_am_pm(polar_mod_ctx_t *ctx, modulation_t modulation, int data, int *ampl_out, int *phase_diff_out) {
    if (!ampl_out || !phase_diff_out)
        return -1;

    int data_2, data_3, data_4, data_5;
    int x, y;
    int ampl;
    int angle;
    int angle_diff = 0;

    // high pass, also removes DC bias
    switch (modulation.filter_pre_hp) {
        case FILTER_HP_200_4pol:
            data_2 = filter_4pol_highpass_200hz(ctx, data); // highpass is only called once, so the storage of the state can be internal
            break;
        case FILTER_HP_300_4pol:
            data_2 = filter_4pol_highpass_300hz(ctx, data);
            break;
        case FILTER_HP_300_2pol:
        default:
            data_2 = filter_2pol_highpass_300hz(ctx, data);
    }
    data_2 = data_2 << 1;

    // general lowpass filter for the mic audio
    switch (modulation.filter_pre_lp) {
        case FILTER_LP_3400_2pol:
            data_3 = filter_2pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3400_4pol:
            data_3 = filter_4pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3000_4pol:
            data_3 = filter_4pol_lowpass_3000hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3000_2pol:
            data_3 = filter_2pol_lowpass_3000hz_bessel(data_2, ctx->delay_lp_adc);
            break;
        default:
            data_3 = filter_4pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
    }
    data_3 = data_3 << 1;

    // shape passband 300-3000Hz
    switch (modulation.filter_pre_pb) {
        case FILTER_PB_500:
            data_3 = filter_1pol_highpass_500hz(ctx, data_3); // highpass is only called once, so the storage of the state can be internal
            break;
        case FILTER_PB_2k:
            data_3 = filter_1pol_highpass_2000hz(ctx, data_3);
            break;
        case FILTER_PB_NONE:
            break; // no filter
        case FILTER_PB_1k:
            data_3 = filter_1pol_highpass_1000hz(ctx, data_3);
            break;
        default:
            data_3 = filter_1pol_highpass_2000hz(ctx, data_3);
            break;
    }
    data_3 = data_3 << 1;

    // AGC: 3rd/4th digit from the right  (0xNNxx)
    switch (modulation.agc_type) {
        case AGC_GAIN_FIX:
            //  AGC set to a fixed value. Corresponds to gain==2
            data_4 = (data_3 * (0x100 & 0xff)) >> 4;
            break;
        case AGC_GAIN_CHANGE:
            // use AGC, but change gain / scaling. Corresponds to gain is equal / 0x220 -> gain is doubled
            data_4 = (data_3 * ctx->agc_gain * ((0x200 & 0xff))) >> 4;
            break;
        case 0: // normal AGC usage
        default:
            // calculate the AGC at a point that no longer has a DC offset (after a high pass). Scaling: agc_gain==256 corresponds to gain==1
            data_4 = (data_3 * ctx->agc_gain) >> 8;
            break;
    }

    // calc new agc_gain value always according to the signal value the usual AGC usage would have had
    ctx->agc_gain = mic_agc_fast(ctx, abs((data_3 * ctx->agc_gain) >> 8), modulation.polar_status);
    data_5 = soft_limiter(data_4);

    // low pass filter after soft limiter to avoid increase in high frequency spectrum
    switch (modulation.filter_post_lp) {
        case FILTER_POST_LP_3400_2pol:
            data_5 = filter_2pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_3400_4pol:
            data_5 = filter_4pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_3000_2pol:
            data_5 = filter_2pol_lowpass_3000hz_bessel(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_NONE:
            break; // no filter
        case FILTER_POST_LP_3000_4pol:
        default:
            data_5 = filter_2pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
    }

    hilbert(ctx, data_5, &x, &y); // produces an FFT peak at fs/2 due to limit cycles?

    x = filter_2pol_lowpass_3400hz(x, ctx->delay_lp_x);
    y = filter_2pol_lowpass_3400hz(y, ctx->delay_lp_y); // Quick fix gegen den Peak bei fs/2

    x = x + 451; // manually calculate the DC offset
    y = y + 98;  // manually calculate the DC offset (new values ​​Feb 2023) -> reduces the DC peak in the FFt only minimally

    if ((modulation.special_modulation == SPECIAL_MODULATION_2_TONE_SIG_IQ) || (modulation.special_modulation == SPECIAL_MODULATION_3_TONE_SIG_IQ)) {
        iq_signal_generator(ctx, modulation.special_modulation, &x, &y);
    }

    cordic(x, y, &ampl, &angle); // x and y (I and Q) -> amplitude and phase

    // TODO: assumes that the soft limiter limits the value of data_5 to +/- 35600 (for data_5, measured at approx. 37000) -> with the
    // amplification in Hilbert and Cordic, the measured "ampl" is approx. 70000 peak
    switch (modulation.modulation_mode) {
        case MOD_AM:
            angle_diff = 0;
            *ampl_out = 35800 + data_5; // with the AGC : +/-35600 is the nominal peak amplitude -> makes about 100% Mod grad and a little bit of overload
            break;
        case MOD_FM:
            angle_diff = data_5 * 148; // +/-35600 is the nominal peak amplitude ;; +/-5kHz freq deviation calculates to +/- 112,5° diff phase at 16kSps
            *ampl_out = 65535;
            break;
        case MOD_FMN:
            angle_diff = data_5 * 74; // +/-35600 is the nominal peak amplitude ;; +/-2,5kHz freq deviation calculates to +/- 56,25° diff phase at 16kSps
            *ampl_out = 65535;
            break;
        case MOD_FMW:
            angle_diff = data_5 * 2220; // +/-35600 is the nominal peak amplitude ;; +/-75kHz freq deviation calculates to +/- 8437,5° diff phase at 16kSps
                                        // (8400° = ca. 23 Umdrehungen)
            *ampl_out = 65535;
            break;
        case MOD_CW:
            angle_diff = 0;
            *ampl_out = 65535;
            break;
        case MOD_LSB:
        case MOD_USB:
            angle_diff = (angle - ctx->last_angle); // value of 2^24 = 16777216 corresponds to 360°  // range: -180...+180°
            if (angle_diff > 0x800000)              // > 180°?
                angle_diff -= 0x1000000;            // 180°...360° -> -180..0°  (unwrap)
            if (angle_diff < -0x800000)             // < -180°?
                angle_diff += 0x1000000;            // -360°...-180° -> 0..180°  (unwrap)
                                                    // if (angle_diff >  0x400000) > 90°?
                                                    //      angle_diff =  0x400000; // set to 90°. Limit Delta-f to 4kHz (at sample rate of 16KHz)
                                                    //      if (angle_diff < -0x400000) < -90°?
                                                    //          angle_diff = -0x400000; // set to -90°. Limit Delta-f to 4kHz (at sample rate of 16KHz)

            if (angle_diff > 0x600000)  // > 135°?
                angle_diff = 0x600000;  // set to 135°. Limit Delta-f to 6kHz (at sample rate of 16KHz)
            if (angle_diff < -0x600000) // < -135°?
                angle_diff = -0x600000; // set to -135°. Limit Delta-f to 6kHz (at sample rate of 16KHz)
            if (modulation.modulation_mode == MOD_USB)
                angle_diff = -angle_diff; // invert for USB
            *ampl_out = ampl;
            break;
        default:
            return -1; // undefined value
    }

    *phase_diff_out = angle_diff;
    ctx->last_angle = angle;

    return 0; // no error
}

void polar_mod_init(polar_mod_ctx_t *ctx) {
    memset(ctx, 0, sizeof(polar_mod_ctx_t));
    ctx->gain_value = 1000; // AGC start value
    ctx->agc_gain = 256;    // Default gain (1.0)
    ctx->last_mode = 555;   // Dummy for signal generator
}
