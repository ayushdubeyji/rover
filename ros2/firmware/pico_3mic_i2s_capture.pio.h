// ---------------------------------------------------------------- //
// Pico 3-Microphone Synchronous I2S Capture PIO Program & C Helper
// Target: RP2040-Zero (Waveshare RP2040-Zero Rover Main Controller)
// Microphones: 3x INMP441 MEMS I2S (Equilateral Triangle d = 5.5 cm)
// Default Pinout (Option B - Direct RP2040-Zero):
//   - GP4:  SCK (Continuous Bit Clock, PIO side_set base pin 0)
//   - GP5:  WS  (Word Select / Frame Sync, PIO side_set base pin 1)
//   - GP28: SD0 (Data Line 0: Mic 1 Left + Mic 2 Right, PIO in base pin 0)
//   - GP29: SD1 (Data Line 1: Mic 3 Left, PIO in base pin 1)
// ---------------------------------------------------------------- //

#ifndef PICO_3MIC_I2S_CAPTURE_PIO_H
#define PICO_3MIC_I2S_CAPTURE_PIO_H

// RP2040-Zero Default Pin Constants
#define PIN_I2S_SCK             4   // GP4 (PIO side_set base pin 0: SCK bit clock)
#define PIN_I2S_WS              5   // GP5 (PIO side_set base pin 1: WS frame sync)
#define PIN_I2S_SD0             28  // GP28 (PIO in base pin 0: SD0 = Mic 1 Front 0° L + Mic 2 Rear-Right -120° R)
#define PIN_I2S_SD1             29  // GP29 (PIO in base pin 1: SD1 = Mic 3 Rear-Left +120° L)

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef PICO_SDK_VERSION_MAJOR
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#endif

// Raw PIO instructions for dual-data line 3-mic I2S sampling
// Uses sideset on SCK (1 bit) and sets WS
static const uint16_t pico_3mic_i2s_program_instructions[] = {
    // 0: set x, 30        side 0b0    ; 31 data bits loop counter
    0xe03e,
    // 1: set pins, 0      side 0b0    ; Assert Left Channel (WS = 0)
    0xe000,
    // 2: in pins, 2       side 0b1    ; Sample SD0 and SD1 on SCK rising edge
    0x4802,
    // 3: jmp x--, 2       side 0b0    ; Loop through Left Channel 31 bits
    0x0042,
    // 4: set x, 30        side 0b0    ; Reset counter for Right Channel
    0xe03e,
    // 5: set pins, 1      side 0b0    ; Assert Right Channel (WS = 1)
    0xe001,
    // 6: in pins, 2       side 0b1    ; Sample SD0 and SD1 on SCK rising edge
    0x4802,
    // 7: jmp x--, 6       side 0b0    ; Loop through Right Channel 31 bits
    0x0046,
};

#ifdef PICO_SDK_VERSION_MAJOR
static const struct pio_program pico_3mic_i2s_program = {
    .instructions = pico_3mic_i2s_program_instructions,
    .length = 8,
    .origin = -1,
};

static inline pio_sm_config pico_3mic_i2s_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 7);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

/**
 * Initializes the RP2040 PIO state machine for 3-channel synchronous I2S.
 * @param pio PIO instance (pio0 or pio1)
 * @param sm State machine index (0-3)
 * @param offset Loaded program instruction memory offset
 * @param sck_pin Clock pin (GP2)
 * @param ws_pin Word select pin (GP3)
 * @param sd_base_pin Base data pin (GP4 for SD0, GP5 for SD1)
 * @param sample_rate Sample rate in Hz (typically 16000)
 */
static inline void pico_3mic_i2s_program_init(
    PIO pio,
    uint sm,
    uint offset,
    uint sck_pin,
    uint ws_pin,
    uint sd_base_pin,
    uint32_t sample_rate
) {
    pio_sm_config c = pico_3mic_i2s_program_get_default_config(offset);

    // Setup Side-set on SCK (GP2)
    sm_config_set_sideset_pins(&c, sck_pin);
    pio_gpio_init(pio, sck_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, sck_pin, 1, true);

    // Setup SET pin on WS (GP3)
    sm_config_set_set_pins(&c, ws_pin, 1);
    pio_gpio_init(pio, ws_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, ws_pin, 1, true);

    // Setup IN pins on SD0 (GP4) and SD1 (GP5)
    sm_config_set_in_pins(&c, sd_base_pin);
    pio_gpio_init(pio, sd_base_pin);
    pio_gpio_init(pio, sd_base_pin + 1);
    gpio_pull_down(sd_base_pin);
    gpio_pull_down(sd_base_pin + 1);
    pio_sm_set_consecutive_pindirs(pio, sm, sd_base_pin, 2, false);

    // Shift in 2 bits per clock into ISR, auto-push after 32 bits (16 pairs)
    sm_config_set_in_shift(&c, false, true, 32);

    // Clock divider: 16 kHz * 64 clock cycles * 2 PIO cycles = 2.048 MHz
    float clock_div = (float)clock_get_hz(clk_sys) / (float)(sample_rate * 64 * 2);
    sm_config_set_clkdiv(&c, clock_div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
#endif

/**
 * C helper to demultiplex raw 2-bit sampled DMA words into 3 distinct audio channels.
 * @param raw_dma_words Interleaved 32-bit words straight from PIO FIFO
 * @param num_words Number of 32-bit words
 * @param mic1_out Left Front channel buffer
 * @param mic2_out Right Rear channel buffer
 * @param mic3_out Left Rear channel buffer
 */
static inline void pico_3mic_demux_samples(
    const uint32_t* raw_dma_words,
    uint32_t num_words,
    int16_t* mic1_out,
    int16_t* mic2_out,
    int16_t* mic3_out
) {
    // Each 32-bit DMA word contains 16 clocks x 2 bits (bit 0 = SD0, bit 1 = SD1)
    // Front mic (Mic 1): SD0 during WS=0
    // Rear-Right mic (Mic 2): SD0 during WS=1
    // Rear-Left mic (Mic 3): SD1 during WS=0
    uint32_t out_idx = 0;
    for (uint32_t w = 0; w + 3 < num_words; w += 4) {
        // Words 0 and 1 represent WS=0 (Left: 32 bits)
        // Words 2 and 3 represent WS=1 (Right: 32 bits)
        uint32_t s0_left = 0, s1_left = 0;
        uint32_t s0_right = 0;

        // Word w: first 16 clocks of Left channel (clocks 0..15, MSB first)
        for (int b = 15; b >= 0; b--) {
            uint32_t pair = (raw_dma_words[w] >> (b * 2)) & 0x03;
            s0_left = (s0_left << 1) | (pair & 0x01);
            s1_left = (s1_left << 1) | ((pair >> 1) & 0x01);
        }
        // Word w + 1: next 16 clocks of Left channel (clocks 16..31, MSB first)
        for (int b = 15; b >= 0; b--) {
            uint32_t pair = (raw_dma_words[w + 1] >> (b * 2)) & 0x03;
            s0_left = (s0_left << 1) | (pair & 0x01);
            s1_left = (s1_left << 1) | ((pair >> 1) & 0x01);
        }

        // Word w + 2: first 16 clocks of Right channel (clocks 0..15, MSB first)
        for (int b = 15; b >= 0; b--) {
            uint32_t pair = (raw_dma_words[w + 2] >> (b * 2)) & 0x03;
            s0_right = (s0_right << 1) | (pair & 0x01);
        }
        // Word w + 3: next 16 clocks of Right channel (clocks 16..31, MSB first)
        for (int b = 15; b >= 0; b--) {
            uint32_t pair = (raw_dma_words[w + 3] >> (b * 2)) & 0x03;
            s0_right = (s0_right << 1) | (pair & 0x01);
        }

        // Take upper 16 bits of 24-bit MSB-justified INMP441 word
        mic1_out[out_idx] = (int16_t)(s0_left >> 16);
        mic2_out[out_idx] = (int16_t)(s0_right >> 16);
        mic3_out[out_idx] = (int16_t)(s1_left >> 16);
        out_idx++;
    }
}

#ifdef __cplusplus
}
#endif

#endif // PICO_3MIC_I2S_CAPTURE_PIO_H
