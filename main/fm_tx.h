#ifndef FM_TX_H
#define FM_TX_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Audio PLL (APLL) configuration used for FM transmission.
 *
 * Fields correspond to the APLL hardware registers / configuration used
 * to produce the requested output frequency. The fractional field
 * base_frac16 represents the 16-bit fractional part (sdm1:sdm0) and
 * dev_frac16 stores the number of fractional LSB steps corresponding
 * to the maximum frequency deviation requested.
 */
typedef struct {
    uint8_t o_div;        /**< Output divider (o_div) used by APLL */
    uint8_t sdm2;         /**< Integer part of APLL fractional multiplier (sdm2) */
    uint16_t base_frac16; /**< 16-bit fractional part (sdm1:sdm0) used as center */
    uint16_t dev_frac16;  /**< ±deviation expressed in same fractional units */
    bool is_rev0;         /**< true if chip revision is rev0 (affects APLL configuration) */
} fm_apll_cfg_t;

/**
 * @brief WAV to transmit 
*/
typedef struct {
    const unsigned char *audio;
    const unsigned int audio_len;
} wav_t;

/**
 * @brief Initialize and configure the I2S peripheral to use APLL as clock source.
 *
 * This function sets up an I2S TX channel in master mode. The function configures
 * the clock source to I2S_CLK_SRC_APLL and requests the sample rate defined in the
 * implementation. The MCLK output is not assigned to a GPIO in this function (see
 * fm_route_to_pin()). After calling this, the I2S channel will be enabled and ready
 * to provide MCLK derived from APLL.
 *
 * @param wav_sr_hz Sample rate
 */
void fm_i2s_init(uint32_t wav_sr_hz);

/**
 * @brief Calculate and initialize the global APLL configuration and enable APLL.
 *
 * This function computes sdm2, sdm1:sdm0 and o_div values that will generate the
 * requested carrier frequency taking into account the XTAL frequency and ensuring
 * the internal VCO stays within the valid lock range. It also programs the APLL
 * registers (via rtc_clk_apll_coeff_set) and enables the APLL.
 *
 * @param fm_carrier_hz Desired APLL output (carrier) in Hz
 * @param max_dev_hz Maximum desired frequency deviation in Hz (absolute)
 */
bool fm_apll_init(uint32_t fm_carrier_hz, uint32_t max_dev_hz);

/**
 * @brief Route the I2S MCLK (APLL derived) to a physical GPIO pin.
 *
 * This maps the MCLK (CLK_OUT1) to GPIO0 and sets that GPIO as an output.
 * Note: GPIO0 is a strapping pin on many ESP32 modules; care must be taken
 * when using it as an RF output that the board can still boot normally.
 */
void fm_route_to_pin(void);

/**
 * @brief Start the periodic audio timer that performs real-time FM modulation.
 *
 * The timer callback reads 8-bit PCM samples from the embedded audio array,
 * converts them to signed values, scales them by the precomputed deviation in
 * fractional LSB units and calls fm_set_deviation to update the APLL.
 *
 * @param wav_sr_hz Sample rate
 * @param wav_file Wav data
 */
void fm_start_audio(uint32_t wav_sr_hz, wav_t *wav_file);

#endif // FM_TX_H