// MIT License

// Copyright (c) 2020 Shawn (L. Spiro) Wilcoxen
// Copyright (c) 2026 Purpbatboi (C port)

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

#ifndef BEESNES_APU_H
#define BEESNES_APU_H

#include <stddef.h>

#include "../../common_def.h"

#ifdef __cplusplus
extern "C" {
#endif

// APU Hardware Regions / Types
typedef enum beesnes_apu_type_e {
    BEESNES_AT_NTSC                = 0,
    BEESNES_AT_PAL                 = 1,
    BEESNES_AT_DENDY               = 2,
    BEESNES_AT_PALM                = 3,
    BEESNES_AT_PALN                = 4,
    BEESNES_AT_TOTAL               = 5
} beesnes_apu_type_t;

// Frame counter clock event flags
typedef enum beesnes_frame_clocks_e {
    BEESNES_FC_QUARTER             = (1 << 0), // Envelopes and linear counter
    BEESNES_FC_HALF                = (1 << 1)  // Length counters and sweepers
} beesnes_frame_clocks_t;

// Channel indices
typedef enum beesnes_channel_e {
    BEESNES_C_PULSE_0              = 0,
    BEESNES_C_PULSE_1              = 1,
    BEESNES_C_TRIANGLE             = 2,
    BEESNES_C_NOISE                = 3,
    BEESNES_C_DMC                  = 4,
    BEESNES_C_TOTAL                = 5
} beesnes_channel_t;

// --------------------------------------------------------------------------
// Sub-unit structures
// --------------------------------------------------------------------------

// Envelope Generator
typedef struct beesnes_envelope_s {
    UINT8                       output;
    UINT8                       decay_counter;
    UINT8                       divider_counter;
    UINT8                       volume;
    bool                        restart;
} beesnes_envelope_t;

// Length Counter
typedef struct beesnes_length_counter_s {
    UINT8                       counter;
} beesnes_length_counter_t;

// Linear Counter (used by Triangle)
typedef struct beesnes_linear_counter_s {
    UINT8                       linear_counter;
    UINT8                       linear_reload;
    bool                        reload_linear;
} beesnes_linear_counter_t;

// Sequencer (base unit for timer countdown and step sequences)
typedef struct beesnes_sequencer_s {
    UINT32                      sequence;
    UINT16                      timer;
    UINT16                      reload;
    UINT8                       out;
    UINT8                       seq_off;
    bool                        restart_seq;
} beesnes_sequencer_t;

// Sweeper (frequency sweep unit for Pulse channels)
typedef struct beesnes_sweeper_s {
    UINT8                       shift;
    UINT8                       timer;
    UINT8                       period;
    bool                        enabled;
    bool                        muted;
    bool                        negated;
    bool                        need_reload;
} beesnes_sweeper_t;

// Pulse Channel (Pulse 1 or Pulse 2)
typedef struct beesnes_pulse_s {
    beesnes_length_counter_t        length;
    beesnes_envelope_t              envelope;
    beesnes_sequencer_t             sequencer;
    beesnes_sweeper_t               sweeper;
    UINT8                       ones_complement; // 1 for Pulse 1, 0 for Pulse 2
} beesnes_pulse_t;

// Triangle Channel
typedef struct beesnes_triangle_s {
    beesnes_length_counter_t        length;
    beesnes_linear_counter_t        linear;
    beesnes_sequencer_t             sequencer;
} beesnes_triangle_t;

// Noise Channel
typedef struct beesnes_noise_s {
    beesnes_length_counter_t        length;
    beesnes_envelope_t              envelope;
    beesnes_sequencer_t             sequencer;
    bool                        mode;            // false: 32767-step LFSR, true: 93-step LFSR
} beesnes_noise_t;

// Delta Modulation Channel (DMC)
typedef struct beesnes_dmc_s {
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
} beesnes_dmc_t;

// Pipeline Delayed Value Structure (models BeesNES CDelayedValue)
typedef struct beesnes_delayed_val_s {
    UINT8                       buffer[8];
    UINT8                       is_write[8];
    size_t                      delay;
    size_t                      dirty;
    void                        (*callback)(void *user_data, UINT8 new_val, UINT8 old_val);
    void                        *user_data;
} beesnes_delayed_val_t;

// --------------------------------------------------------------------------
// Main APU 2A03 structure
// --------------------------------------------------------------------------
typedef struct beesnes_apu_s {
    // Memory mapped registers $4000-$4017
    UINT8                       registers[0x18];

    // Sound channels
    beesnes_pulse_t                 pulse1;
    beesnes_pulse_t                 pulse2;
    beesnes_triangle_t              triangle;
    beesnes_noise_t                 noise;
    beesnes_dmc_t                   dmc;

    // Pipeline delayed values
    beesnes_delayed_val_t           dv_4017;
    beesnes_delayed_val_t           dv_pulse1_length;
    beesnes_delayed_val_t           dv_pulse2_length;
    beesnes_delayed_val_t           dv_triangle_length;
    beesnes_delayed_val_t           dv_noise_length;
    beesnes_delayed_val_t           dv_pulse1_halt;
    beesnes_delayed_val_t           dv_pulse2_halt;
    beesnes_delayed_val_t           dv_triangle_halt;
    beesnes_delayed_val_t           dv_noise_halt;

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
    beesnes_apu_type_t              type;
    bool                        swap_duty;      // True for PAL-M / PAL-N
    bool                        letterless;     // True for earliest RP2A03 revision

    // DMC sample fetch from the CPU bus
    void                        *cb_user_data;
    UINT8                       (*cb_dma_read)(void *user_data, UINT16 addr);

} beesnes_apu_t;

// --------------------------------------------------------------------------
// Public API Functions
// --------------------------------------------------------------------------

// Initialization & Reset
void        beesnes_apu_init(beesnes_apu_t *apu, beesnes_apu_type_t type);
void        beesnes_apu_reset(beesnes_apu_t *apu);
void        beesnes_apu_reset_analog(beesnes_apu_t *apu);

// CPU Bus Interface
void        beesnes_apu_write(beesnes_apu_t *apu, UINT16 addr, UINT8 val);
UINT8       beesnes_apu_read(beesnes_apu_t *apu, UINT16 addr);

// Cycle-by-cycle Execution
void        beesnes_apu_tick(beesnes_apu_t *apu);
// Quiet cycles: cycles in which beesnes_apu_tick() does not change the output (only sequencers of channels
// that cannot be heard, or whose beesnes_channel_t bit is in `ignore`, run out). beesnes_apu_skip() runs up to
// beesnes_apu_quiet_cycles() of them at once; tick once with beesnes_apu_tick() after it before reading or
// writing the APU.
UINT32      beesnes_apu_quiet_cycles(const beesnes_apu_t *apu, UINT8 ignore);
// A register write's delayed effect has not landed yet (it does within a few beesnes_apu_tick() calls).
bool        beesnes_apu_delays_pending(const beesnes_apu_t *apu);
void        beesnes_apu_skip(beesnes_apu_t *apu, UINT32 cycles);

// Output of each channel as the mixer sees it, indexed by beesnes_channel_t: pulses, triangle and
// noise 0-15, DMC 0-127. A silenced channel reads 0.
void        beesnes_apu_get_channel_levels(const beesnes_apu_t *apu, UINT8 levels[BEESNES_C_TOTAL]);

// Configuration
// The DMC fetches each sample byte through dma_read; without it the byte is never delivered.
void        beesnes_apu_set_dma_read(
                beesnes_apu_t *apu,
                void *user_data,
                UINT8 (*dma_read)(void *user_data, UINT16 addr)
            );
void        beesnes_apu_set_letterless(beesnes_apu_t *apu, bool letterless);

#ifdef __cplusplus
}
#endif

#endif /* BEESNES_APU_H */
