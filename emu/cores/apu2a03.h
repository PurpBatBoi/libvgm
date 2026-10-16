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

#ifndef LSN_APU2A03_H
#define LSN_APU2A03_H

#include <stddef.h>

#include "../../common_def.h"

#ifdef __cplusplus
extern "C" {
#endif

// APU Hardware Regions / Types
typedef enum lsn_apu_type_e {
    LSN_AT_NTSC                = 0,
    LSN_AT_PAL                 = 1,
    LSN_AT_DENDY               = 2,
    LSN_AT_PALM                = 3,
    LSN_AT_PALN                = 4,
    LSN_AT_TOTAL               = 5
} lsn_apu_type_t;

// Frame counter clock event flags
typedef enum lsn_frame_clocks_e {
    LSN_FC_QUARTER             = (1 << 0), // Envelopes and linear counter
    LSN_FC_HALF                = (1 << 1)  // Length counters and sweepers
} lsn_frame_clocks_t;

// Channel indices
typedef enum lsn_channel_e {
    LSN_C_PULSE_0              = 0,
    LSN_C_PULSE_1              = 1,
    LSN_C_TRIANGLE             = 2,
    LSN_C_NOISE                = 3,
    LSN_C_DMC                  = 4,
    LSN_C_TOTAL                = 5
} lsn_channel_t;

// --------------------------------------------------------------------------
// Sub-unit structures
// --------------------------------------------------------------------------

// Envelope Generator
typedef struct lsn_envelope_s {
    UINT8                       output;
    UINT8                       decay_counter;
    UINT8                       divider_counter;
    UINT8                       volume;
    bool                        restart;
} lsn_envelope_t;

// Length Counter
typedef struct lsn_length_counter_s {
    UINT8                       counter;
} lsn_length_counter_t;

// Linear Counter (used by Triangle)
typedef struct lsn_linear_counter_s {
    UINT8                       linear_counter;
    UINT8                       linear_reload;
    bool                        reload_linear;
} lsn_linear_counter_t;

// Sequencer (base unit for timer countdown and step sequences)
typedef struct lsn_sequencer_s {
    UINT32                      sequence;
    UINT16                      timer;
    UINT16                      reload;
    UINT8                       out;
    UINT8                       seq_off;
    bool                        restart_seq;
} lsn_sequencer_t;

// Sweeper (frequency sweep unit for Pulse channels)
typedef struct lsn_sweeper_s {
    UINT8                       shift;
    UINT8                       timer;
    UINT8                       period;
    bool                        enabled;
    bool                        muted;
    bool                        negated;
    bool                        need_reload;
} lsn_sweeper_t;

// Pulse Channel (Pulse 1 or Pulse 2)
typedef struct lsn_pulse_s {
    lsn_length_counter_t        length;
    lsn_envelope_t              envelope;
    lsn_sequencer_t             sequencer;
    lsn_sweeper_t               sweeper;
    UINT8                       ones_complement; // 1 for Pulse 1, 0 for Pulse 2
} lsn_pulse_t;

// Triangle Channel
typedef struct lsn_triangle_s {
    lsn_length_counter_t        length;
    lsn_linear_counter_t        linear;
    lsn_sequencer_t             sequencer;
} lsn_triangle_t;

// Noise Channel
typedef struct lsn_noise_s {
    lsn_length_counter_t        length;
    lsn_envelope_t              envelope;
    lsn_sequencer_t             sequencer;
    bool                        mode;            // false: 32767-step LFSR, true: 93-step LFSR
} lsn_noise_t;

// Delta Modulation Channel (DMC)
typedef struct lsn_dmc_s {
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
} lsn_dmc_t;

// Pipeline Delayed Value Structure (models BeesNES CDelayedValue)
typedef struct lsn_delayed_val_s {
    UINT8                       buffer[8];
    UINT8                       is_write[8];
    size_t                      delay;
    size_t                      dirty;
    void                        (*callback)(void *user_data, UINT8 new_val, UINT8 old_val);
    void                        *user_data;
} lsn_delayed_val_t;

// --------------------------------------------------------------------------
// Main APU 2A03 structure
// --------------------------------------------------------------------------
typedef struct lsn_apu2a03_s {
    // Memory mapped registers $4000-$4017
    UINT8                       registers[0x18];

    // Sound channels
    lsn_pulse_t                 pulse1;
    lsn_pulse_t                 pulse2;
    lsn_triangle_t              triangle;
    lsn_noise_t                 noise;
    lsn_dmc_t                   dmc;

    // Pipeline delayed values
    lsn_delayed_val_t           dv_4017;
    lsn_delayed_val_t           dv_pulse1_length;
    lsn_delayed_val_t           dv_pulse2_length;
    lsn_delayed_val_t           dv_triangle_length;
    lsn_delayed_val_t           dv_noise_length;
    lsn_delayed_val_t           dv_pulse1_halt;
    lsn_delayed_val_t           dv_pulse2_halt;
    lsn_delayed_val_t           dv_triangle_halt;
    lsn_delayed_val_t           dv_noise_halt;

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
    lsn_apu_type_t              type;
    bool                        swap_duty;      // True for PAL-M / PAL-N
    bool                        letterless;     // True for earliest RP2A03 revision

    // DMC sample fetch from the CPU bus
    void                        *cb_user_data;
    UINT8                       (*cb_dma_read)(void *user_data, UINT16 addr);

} lsn_apu2a03_t;

// --------------------------------------------------------------------------
// Public API Functions
// --------------------------------------------------------------------------

// Initialization & Reset
void        lsn_apu2a03_init(lsn_apu2a03_t *apu, lsn_apu_type_t type);
void        lsn_apu2a03_reset(lsn_apu2a03_t *apu);
void        lsn_apu2a03_reset_analog(lsn_apu2a03_t *apu);

// CPU Bus Interface
void        lsn_apu2a03_write(lsn_apu2a03_t *apu, UINT16 addr, UINT8 val);
UINT8       lsn_apu2a03_read(lsn_apu2a03_t *apu, UINT16 addr);

// Cycle-by-cycle Execution
void        lsn_apu2a03_tick(lsn_apu2a03_t *apu);

// Output of each channel as the mixer sees it, indexed by lsn_channel_t: pulses, triangle and
// noise 0-15, DMC 0-127. A silenced channel reads 0.
void        lsn_apu2a03_get_channel_levels(const lsn_apu2a03_t *apu, UINT8 levels[LSN_C_TOTAL]);

// Configuration
// The DMC fetches each sample byte through dma_read; without it the byte is never delivered.
void        lsn_apu2a03_set_dma_read(
                lsn_apu2a03_t *apu,
                void *user_data,
                UINT8 (*dma_read)(void *user_data, UINT16 addr)
            );
void        lsn_apu2a03_set_letterless(lsn_apu2a03_t *apu, bool letterless);

#ifdef __cplusplus
}
#endif

#endif /* LSN_APU2A03_H */
