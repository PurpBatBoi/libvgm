// MIT License

// Copyright (c) 2026 Purpbatboi

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BEENES_APU_H
#define BEENES_APU_H

#include <stddef.h>

#include "../../common_def.h"

#ifdef __cplusplus
extern "C" {
#endif

// APU Hardware Regions / Types
typedef enum beenes_apu_type_e {
    BEENES_AT_NTSC                = 0,
    BEENES_AT_PAL                 = 1,
    BEENES_AT_DENDY               = 2,
    BEENES_AT_PALM                = 3,
    BEENES_AT_PALN                = 4,
    BEENES_AT_TOTAL               = 5
} beenes_apu_type_t;

// Frame counter clock event flags
typedef enum beenes_frame_clocks_e {
    BEENES_FC_QUARTER             = (1 << 0), // Envelopes and linear counter
    BEENES_FC_HALF                = (1 << 1)  // Length counters and sweepers
} beenes_frame_clocks_t;

// Channel indices
typedef enum beenes_channel_e {
    BEENES_C_PULSE_0              = 0,
    BEENES_C_PULSE_1              = 1,
    BEENES_C_TRIANGLE             = 2,
    BEENES_C_NOISE                = 3,
    BEENES_C_DMC                  = 4,
    BEENES_C_TOTAL                = 5
} beenes_channel_t;

// --------------------------------------------------------------------------
// Sub-unit structures
// --------------------------------------------------------------------------

// Envelope Generator
typedef struct beenes_envelope_s {
    UINT8                       output;
    UINT8                       decay_counter;
    UINT8                       divider_counter;
    UINT8                       volume;
    bool                        restart;
} beenes_envelope_t;

// Length Counter
typedef struct beenes_length_counter_s {
    UINT8                       counter;
} beenes_length_counter_t;

// Linear Counter (used by Triangle)
typedef struct beenes_linear_counter_s {
    UINT8                       linear_counter;
    UINT8                       linear_reload;
    bool                        reload_linear;
} beenes_linear_counter_t;

// Sequencer (base unit for timer countdown and step sequences)
typedef struct beenes_sequencer_s {
    UINT32                      sequence;
    UINT16                      timer;
    UINT16                      reload;
    UINT8                       out;
    UINT8                       seq_off;
    bool                        restart_seq;
} beenes_sequencer_t;

// Sweeper (frequency sweep unit for Pulse channels)
typedef struct beenes_sweeper_s {
    UINT8                       shift;
    UINT8                       timer;
    UINT8                       period;
    bool                        enabled;
    bool                        muted;
    bool                        negated;
    bool                        need_reload;
} beenes_sweeper_t;

// Pulse Channel (Pulse 1 or Pulse 2)
typedef struct beenes_pulse_s {
    beenes_length_counter_t        length;
    beenes_envelope_t              envelope;
    beenes_sequencer_t             sequencer;
    beenes_sweeper_t               sweeper;
    UINT8                       ones_complement; // 1 for Pulse 1, 0 for Pulse 2
} beenes_pulse_t;

// Triangle Channel
typedef struct beenes_triangle_s {
    beenes_length_counter_t        length;
    beenes_linear_counter_t        linear;
    beenes_sequencer_t             sequencer;
} beenes_triangle_t;

// Noise Channel
typedef struct beenes_noise_s {
    beenes_length_counter_t        length;
    beenes_envelope_t              envelope;
    beenes_sequencer_t             sequencer;
    bool                        mode;            // false: 32767-step LFSR, true: 93-step LFSR
} beenes_noise_t;

// Delta Modulation Channel (DMC)
typedef struct beenes_dmc_s {
    UINT16                      sample_address;
    UINT16                      sample_length;
    UINT16                      current_address;
    UINT16                      bytes_remaining;
    UINT8                       output_level;
    UINT8                       sample_buffer;
    UINT8                       shift_register;
    UINT8                       bits_remaining;
    UINT8                       start_delay;
    UINT8                       enable_delay;
    UINT64                      fetch_cycle;
    UINT16                      timer;
    UINT16                      timer_period;
    bool                        irq_enabled;
    bool                        loop;
    bool                        irq_asserted;
    bool                        buffer_empty;
    bool                        silent;
    bool                        enabled;
    bool                        enable_written;
    bool                        set_implicit_abort;
    bool                        implicit_abort;
    bool                        clocked;
} beenes_dmc_t;

// Pipeline Delayed Value Structure (models BeesNES CDelayedValue)
typedef struct beenes_delayed_val_s {
    UINT8                       buffer[8];
    UINT8                       is_write[8];
    size_t                      delay;
    size_t                      dirty;
    void                        (*callback)(void *user_data, UINT8 new_val, UINT8 old_val);
    void                        *user_data;
} beenes_delayed_val_t;

// --------------------------------------------------------------------------
// Main APU 2A03 structure
// --------------------------------------------------------------------------
typedef struct beenes_apu_s {
    // Memory mapped registers $4000-$4017
    UINT8                       registers[0x18];

    // Sound channels
    beenes_pulse_t                 pulse1;
    beenes_pulse_t                 pulse2;
    beenes_triangle_t              triangle;
    beenes_noise_t                 noise;
    beenes_dmc_t                   dmc;

    // Pipeline delayed values
    beenes_delayed_val_t           dv_4017;
    beenes_delayed_val_t           dv_pulse1_length;
    beenes_delayed_val_t           dv_pulse2_length;
    beenes_delayed_val_t           dv_triangle_length;
    beenes_delayed_val_t           dv_noise_length;
    beenes_delayed_val_t           dv_pulse1_halt;
    beenes_delayed_val_t           dv_pulse2_halt;
    beenes_delayed_val_t           dv_triangle_halt;
    beenes_delayed_val_t           dv_noise_halt;

    // Cycle and timing counters
    UINT64                      cycles;
    UINT64                      step_cycles;
    INT64                       ticks_to_len_cntr;
    UINT64                      frame_irq_clear_cycle;

    // Frame sequencer state
    UINT8                       frame_mode;     // 0: 4-step mode, 1: 5-step mode
    UINT8                       frame_step;     // Current step 0..4
    UINT8                       frame_clocks;   // Events clocked on this cycle
    UINT8                       frame_clock_wait;// Delay ticks for $4017 bit 7 clock
    UINT8                       last_4017;
    bool                        mode_switch;
    bool                        frame_irq_flag;

    // Hardware configuration
    beenes_apu_type_t              type;
    bool                        swap_duty;      // True for PAL-M / PAL-N
    bool                        letterless;     // True for earliest RP2A03 revision

    // DMC sample fetch from the CPU bus
    void                        *cb_user_data;
    UINT8                       (*cb_dma_read)(void *user_data, UINT16 addr);

} beenes_apu_t;

// --------------------------------------------------------------------------
// Public API Functions
// --------------------------------------------------------------------------

// Initialization & Reset
void        beenes_apu_init(beenes_apu_t *apu, beenes_apu_type_t type);
void        beenes_apu_reset(beenes_apu_t *apu);
void        beenes_apu_reset_analog(beenes_apu_t *apu);

// CPU Bus Interface
void        beenes_apu_write(beenes_apu_t *apu, UINT16 addr, UINT8 val);
UINT8       beenes_apu_read(beenes_apu_t *apu, UINT16 addr);

// Cycle-by-cycle Execution
void        beenes_apu_tick(beenes_apu_t *apu);

// Output of each channel as the mixer sees it, indexed by beenes_channel_t: pulses, triangle and
// noise 0-15, DMC 0-127. A silenced channel reads 0.
void        beenes_apu_get_channel_levels(const beenes_apu_t *apu, UINT8 levels[BEENES_C_TOTAL]);

// Configuration
// The DMC fetches each sample byte through dma_read; without it the byte is never delivered.
void        beenes_apu_set_dma_read(
                beenes_apu_t *apu,
                void *user_data,
                UINT8 (*dma_read)(void *user_data, UINT16 addr)
            );
void        beenes_apu_set_letterless(beenes_apu_t *apu, bool letterless);

#ifdef __cplusplus
}
#endif

#endif /* BEENES_APU_H */
