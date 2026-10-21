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

// The APU (2A03 / 2A07 sound) of BeesNES, L-Spiro's sub-cycle-accurate NES emulator
// (https://github.com/L-Spiro/BeesNES), ported to C for libvgm.

#include "beesnes_apu.h"
#include <string.h>

// --------------------------------------------------------------------------
// Hardware tables
// --------------------------------------------------------------------------

// Length table (32 entries)
static const UINT8 beesnes_length_table[32] = {
    10, 254, 20,   2, 40,   4, 80,   6, 160,   8, 60,  10, 14,  12, 26,  14,
    12,  16, 24,  18, 48,  20, 96,  22, 192,  24, 72,  26, 16,  28, 32,  30
};

// Noise period tables: NTSC (0) and PAL (1)
static const UINT16 beesnes_noise_table[2][16] = {
    { 2, 4, 8, 16, 32, 48, 64, 80, 101, 127, 190, 254, 381, 508, 1017, 2034 },
    { 2, 4, 7, 15, 30, 44, 59, 74,  94, 118, 177, 236, 354, 472,  945, 1889 }
};

// Old noise table for earliest RP2A03 (letterless)
static const UINT16 beesnes_old_noise_table[16] = {
    2, 4, 8, 16, 32, 48, 64, 80, 101, 127, 190, 254, 381, 508, 1017, 1023
};

// 32-step triangle sequence table
static const UINT8 beesnes_triangle_table[32] = {
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

// Duty sequence waveforms
static const UINT8 beesnes_duty_normal[4] = {
    0x80, // 12.5%
    0xC0, // 25.0%
    0xF0, // 50.0%
    0x3F  // 75.0%
};

static const UINT8 beesnes_duty_swapped[4] = {
    0x80, // 12.5%
    0xF0, // 50.0%
    0xC0, // 25.0%
    0x3F  // 75.0%
};

// DMC rate countdown periods (CPU cycles)
static const UINT16 beesnes_dmc_periods[5][16] = {
    // NTSC
    { 428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54 },
    // PAL
    { 398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118,  98, 78, 66, 50 },
    // DENDY
    { 428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54 },
    // PAL-M
    { 428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54 },
    // PAL-N
    { 398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118,  98, 78, 66, 50 }
};

// Frame Counter Step Cycle Timings
typedef struct beesnes_frame_timings_s {
    UINT32 m0_s0;
    UINT32 m0_s1;
    UINT32 m0_s2;
    UINT32 m0_s3_0;
    UINT32 m0_s3_1;
    UINT32 m0_s3_2;

    UINT32 m1_s0;
    UINT32 m1_s1;
    UINT32 m1_s2;
    UINT32 m1_s3;
    UINT32 m1_s4_0;
    UINT32 m1_s4_1;
} beesnes_frame_timings_t;

static const beesnes_frame_timings_t beesnes_timings_ntsc = {
    7457, 14913, 22371, 29828, 29829, 29830,
    7457, 14913, 22371, 29829, 37281, 37282
};

static const beesnes_frame_timings_t beesnes_timings_pal = {
    8313, 16627, 24939, 33252, 33253, 33254,
    8313, 16627, 24939, 33253, 41565, 41566
};

// --------------------------------------------------------------------------
// Static Helper Prototypes
// --------------------------------------------------------------------------

static UINT8  beesnes_get_duty(const beesnes_apu_t *apu, UINT8 duty_idx);
static void     beesnes_clock_frame_units(beesnes_apu_t *apu, UINT8 which);

static void     beesnes_delayed_val_init(beesnes_delayed_val_t *dv, size_t delay,
                                     void (*cb)(void *, UINT8, UINT8), void *user_data);
static void     beesnes_delayed_val_write(beesnes_delayed_val_t *dv, UINT8 val);
static void     beesnes_delayed_val_set(beesnes_delayed_val_t *dv, UINT8 val, bool trigger_callback);
static void     beesnes_delayed_val_tick(beesnes_delayed_val_t *dv);
static UINT8  beesnes_delayed_val_get(const beesnes_delayed_val_t *dv);

// Delayed write callbacks
static void     cb_set_4017(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_pulse1_length(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_pulse2_length(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_triangle_length(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_noise_length(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_pulse1_halt(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_pulse2_halt(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_triangle_halt(void *user_data, UINT8 new_val, UINT8 old_val);
static void     cb_noise_halt(void *user_data, UINT8 new_val, UINT8 old_val);

// Unit helpers
static void     beesnes_envelope_init(beesnes_envelope_t *env);
static UINT8  beesnes_envelope_tick(beesnes_envelope_t *env, bool use_volume, bool loop);
static UINT8  beesnes_envelope_get_output(const beesnes_envelope_t *env, bool use_volume);

static void     beesnes_length_counter_init(beesnes_length_counter_t *lc);
static UINT8  beesnes_length_counter_tick(beesnes_length_counter_t *lc, bool enabled, bool halt);

static void     beesnes_linear_counter_init(beesnes_linear_counter_t *lin);
static UINT8  beesnes_linear_counter_tick(beesnes_linear_counter_t *lin, bool halt);

static void     beesnes_sweeper_init(beesnes_sweeper_t *sw);
static UINT16 beesnes_sweeper_next_val(UINT16 reload, UINT8 shift, bool negated, UINT8 ones_comp);
static void     beesnes_sweeper_update_state(beesnes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp);
static UINT16 beesnes_sweeper_tick(beesnes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp);

static void     beesnes_pulse_init(beesnes_pulse_t *p, UINT8 ones_comp);
static void     beesnes_pulse_tick_sequencer(beesnes_pulse_t *p, bool enabled);
static bool     beesnes_pulse_producing_sound(const beesnes_pulse_t *p, bool enabled);

static void     beesnes_triangle_init(beesnes_triangle_t *t);
static void     beesnes_triangle_tick_sequencer(beesnes_triangle_t *t, bool enabled);
static bool     beesnes_triangle_stepping(const beesnes_triangle_t *t);

static void     beesnes_noise_init(beesnes_noise_t *n);
static void     beesnes_noise_tick_sequencer(beesnes_noise_t *n, bool enabled);
static bool     beesnes_noise_producing_sound(const beesnes_noise_t *n, bool enabled);

static void     beesnes_dmc_init(beesnes_dmc_t *dmc);
static void     beesnes_dmc_tick(beesnes_apu_t *apu);
static void     beesnes_dmc_write4010(beesnes_apu_t *apu, UINT8 val);
static void     beesnes_dmc_set_enabled(beesnes_apu_t *apu, bool enabled);
static void     beesnes_dmc_clear_irq(beesnes_apu_t *apu);
static void     beesnes_dmc_receive_sample(beesnes_apu_t *apu, UINT8 sample);
static void     beesnes_dmc_fetch(beesnes_apu_t *apu);
static bool     beesnes_dmc_is_active(const beesnes_dmc_t *dmc);

// --------------------------------------------------------------------------
// Delayed Value Implementation
// --------------------------------------------------------------------------

static void beesnes_delayed_val_init(beesnes_delayed_val_t *dv, size_t delay,
                                 void (*cb)(void *, UINT8, UINT8), void *user_data) {
    memset(dv->buffer, 0, sizeof(dv->buffer));
    memset(dv->is_write, 0, sizeof(dv->is_write));
    dv->delay = delay;
    dv->dirty = 0;
    dv->callback = cb;
    dv->user_data = user_data;
}

static void beesnes_delayed_val_write(beesnes_delayed_val_t *dv, UINT8 val) {
    if (dv->delay == 0) {
        if (dv->callback) {
            dv->callback(dv->user_data, val, dv->buffer[0]);
        }
    }
    dv->buffer[0] = val;
    dv->is_write[0] = 1;
    dv->dirty = dv->delay;
}

static void beesnes_delayed_val_set(beesnes_delayed_val_t *dv, UINT8 val, bool trigger_callback) {
    size_t i;
    if (trigger_callback && dv->callback) {
        dv->callback(dv->user_data, val, dv->buffer[dv->delay]);
    }
    dv->dirty = 0;
    for (i = 0; i <= dv->delay; ++i) {
        dv->buffer[i] = val;
        dv->is_write[i] = 0;
    }
}

static void beesnes_delayed_val_tick(beesnes_delayed_val_t *dv) {
    size_t i;
    if (dv->delay != 0 && dv->dirty > 0) {
        dv->dirty--;
        if (dv->callback && dv->is_write[dv->delay - 1]) {
            dv->callback(dv->user_data, dv->buffer[dv->delay - 1], dv->buffer[dv->delay]);
        }
        for (i = dv->delay; i > 0; --i) {
            dv->buffer[i] = dv->buffer[i - 1];
            dv->is_write[i] = dv->is_write[i - 1];
        }
        dv->is_write[0] = 0;
    }
}

static UINT8 beesnes_delayed_val_get(const beesnes_delayed_val_t *dv) {
    return dv->buffer[dv->delay];
}

// Delayed write callback implementations
static void cb_set_4017(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)new_val;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    apu->mode_switch = true;
}

static void cb_pulse1_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->pulse1.length.counter != 0)) {
        apu->pulse1.length.counter = new_val;
    }
}

static void cb_pulse2_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->pulse2.length.counter != 0)) {
        apu->pulse2.length.counter = new_val;
    }
}

static void cb_triangle_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->triangle.length.counter != 0)) {
        apu->triangle.length.counter = new_val;
    }
}

static void cb_noise_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->noise.length.counter != 0)) {
        apu->noise.length.counter = new_val;
    }
}

static void cb_pulse1_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x00] |= 0x20;
    } else {
        apu->registers[0x00] &= ~0x20;
    }
}

static void cb_pulse2_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x04] |= 0x20;
    } else {
        apu->registers[0x04] &= ~0x20;
    }
}

static void cb_triangle_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x08] |= 0x80;
    } else {
        apu->registers[0x08] &= ~0x80;
    }
}

static void cb_noise_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beesnes_apu_t *apu;
    (void)old_val;
    apu = (beesnes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x0C] |= 0x20;
    } else {
        apu->registers[0x0C] &= ~0x20;
    }
}

// --------------------------------------------------------------------------
// Envelope Generator
// --------------------------------------------------------------------------

static void beesnes_envelope_init(beesnes_envelope_t *env) {
    env->output = 0;
    env->decay_counter = 0;
    env->divider_counter = 0;
    env->volume = 0;
    env->restart = true;
}

static UINT8 beesnes_envelope_tick(beesnes_envelope_t *env, bool use_volume, bool loop) {
    (void)use_volume;
    if (env->restart) {
        env->restart = false;
        env->divider_counter = env->volume;
        env->decay_counter = 15;
    } else {
        if (env->divider_counter == 0) {
            env->divider_counter = env->volume;
            if (env->decay_counter == 0) {
                if (loop) {
                    env->decay_counter = 15;
                }
            } else {
                env->decay_counter--;
            }
        } else {
            env->divider_counter--;
        }
    }
    env->output = env->decay_counter;
    return env->output;
}

static UINT8 beesnes_envelope_get_output(const beesnes_envelope_t *env, bool use_volume) {
    return use_volume ? env->volume : env->output;
}

// --------------------------------------------------------------------------
// Length Counter
// --------------------------------------------------------------------------

static void beesnes_length_counter_init(beesnes_length_counter_t *lc) {
    lc->counter = 0;
}

static UINT8 beesnes_length_counter_tick(beesnes_length_counter_t *lc, bool enabled, bool halt) {
    if (!enabled) {
        lc->counter = 0;
    } else if (lc->counter > 0 && !halt) {
        lc->counter--;
    }
    return lc->counter;
}

// --------------------------------------------------------------------------
// Linear Counter
// --------------------------------------------------------------------------

static void beesnes_linear_counter_init(beesnes_linear_counter_t *lin) {
    lin->linear_counter = 0;
    lin->linear_reload = 0;
    lin->reload_linear = true;
}

static UINT8 beesnes_linear_counter_tick(beesnes_linear_counter_t *lin, bool halt) {
    if (lin->reload_linear) {
        lin->linear_counter = lin->linear_reload;
    } else if (lin->linear_counter > 0) {
        lin->linear_counter--;
    }
    if (!halt) {
        lin->reload_linear = false;
    }
    return lin->linear_counter;
}

// --------------------------------------------------------------------------
// Sweeper
// --------------------------------------------------------------------------

static void beesnes_sweeper_init(beesnes_sweeper_t *sw) {
    sw->shift = 0;
    sw->timer = 0;
    sw->period = 0;
    sw->enabled = false;
    sw->muted = false;
    sw->negated = false;
    sw->need_reload = true;
}

static UINT16 beesnes_sweeper_next_val(UINT16 reload, UINT8 shift, bool negated, UINT8 ones_comp) {
    UINT16 change;
    change = (UINT16)(reload >> shift);
    if (reload >= 8 && change < 0x07FF) {
        if (negated) {
            reload = (UINT16)(reload - change - ones_comp);
        } else {
            reload = (UINT16)(reload + change);
        }
    }
    return reload;
}

static void beesnes_sweeper_update_state(beesnes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp) {
    UINT16 next_val;
    next_val = beesnes_sweeper_next_val(reload, sw->shift, sw->negated, ones_comp);
    sw->muted = (reload < 8) || (!sw->negated && next_val > 0x07FF);
}

static UINT16 beesnes_sweeper_tick(beesnes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp) {
    if (sw->timer == 0 && sw->enabled && sw->shift > 0 && !sw->muted) {
        reload = beesnes_sweeper_next_val(reload, sw->shift, sw->negated, ones_comp);
    }
    if (sw->timer == 0 || sw->need_reload) {
        sw->timer = sw->period;
        sw->need_reload = false;
    } else {
        sw->timer--;
    }
    return reload;
}

// --------------------------------------------------------------------------
// Pulse Channel
// --------------------------------------------------------------------------

static void beesnes_pulse_init(beesnes_pulse_t *p, UINT8 ones_comp) {
    beesnes_length_counter_init(&p->length);
    beesnes_envelope_init(&p->envelope);
    memset(&p->sequencer, 0, sizeof(p->sequencer));
    beesnes_sweeper_init(&p->sweeper);
    p->ones_complement = ones_comp;
}

static void beesnes_pulse_tick_sequencer(beesnes_pulse_t *p, bool enabled) {
    (void)enabled;
    if (p->sequencer.restart_seq) {
        p->sequencer.restart_seq = false;
        p->sequencer.seq_off = 0;
        p->sequencer.out = (UINT8)((p->sequencer.sequence >> p->sequencer.seq_off--) & 1);
        if (p->sequencer.seq_off == 0xFF) {
            p->sequencer.seq_off = 0x07;
        }
    }
    if (--p->sequencer.timer == 0xFFFF) {
        p->sequencer.timer = p->sequencer.reload;
        p->sequencer.out = (UINT8)((p->sequencer.sequence >> p->sequencer.seq_off--) & 1);
        if (p->sequencer.seq_off == 0xFF) {
            p->sequencer.seq_off = 0x07;
        }
    }
}

static bool beesnes_pulse_producing_sound(const beesnes_pulse_t *p, bool enabled) {
    return enabled &&
           (p->length.counter > 0) &&
           (p->sequencer.reload >= 8) &&
           (p->sequencer.out != 0) &&
           (!p->sweeper.muted);
}

// --------------------------------------------------------------------------
// Triangle Channel
// --------------------------------------------------------------------------

static void beesnes_triangle_init(beesnes_triangle_t *t) {
    beesnes_length_counter_init(&t->length);
    beesnes_linear_counter_init(&t->linear);
    memset(&t->sequencer, 0, sizeof(t->sequencer));
    t->sequencer.out = beesnes_triangle_table[0];
}

static void beesnes_triangle_tick_sequencer(beesnes_triangle_t *t, bool enabled) {
    (void)enabled;
    if (t->sequencer.restart_seq) {
        t->sequencer.restart_seq = false;
        t->sequencer.seq_off = 0;
        t->sequencer.out = beesnes_triangle_table[t->sequencer.seq_off];
    }
    if (--t->sequencer.timer == 0xFFFF) {
        t->sequencer.timer = t->sequencer.reload;
        if (t->length.counter != 0 && t->linear.linear_counter != 0) {
            t->sequencer.seq_off = (UINT8)((t->sequencer.seq_off + 1) % 32);
            t->sequencer.out = beesnes_triangle_table[t->sequencer.seq_off];
        }
    }
}

// The sequencer steps while both counters are not 0; otherwise the level stays where it is.
static bool beesnes_triangle_stepping(const beesnes_triangle_t *t) {
    return (t->length.counter > 0) && (t->linear.linear_counter > 0);
}

// --------------------------------------------------------------------------
// Noise Channel
// --------------------------------------------------------------------------

static void beesnes_noise_init(beesnes_noise_t *n) {
    beesnes_length_counter_init(&n->length);
    beesnes_envelope_init(&n->envelope);
    memset(&n->sequencer, 0, sizeof(n->sequencer));
    n->sequencer.sequence = 1;
    n->mode = false;
}

static void beesnes_noise_tick_sequencer(beesnes_noise_t *n, bool enabled) {
    UINT32 bit0;
    UINT32 other;
    UINT32 feedback;
    (void)enabled;

    if (n->sequencer.restart_seq) {
        n->sequencer.restart_seq = false;
        n->sequencer.seq_off = 0;

        bit0 = n->sequencer.sequence & 1;
        other = n->mode ? ((n->sequencer.sequence >> 6) & 1) : ((n->sequencer.sequence >> 1) & 1);
        feedback = bit0 ^ other;
        n->sequencer.sequence = (feedback << 14) | ((n->sequencer.sequence & 0x7FFF) >> 1);
        n->sequencer.out = (UINT8)((n->sequencer.sequence & 1) == 0);
    }
    if (--n->sequencer.timer == 0xFFFF) {
        n->sequencer.timer = n->sequencer.reload;

        bit0 = n->sequencer.sequence & 1;
        other = n->mode ? ((n->sequencer.sequence >> 6) & 1) : ((n->sequencer.sequence >> 1) & 1);
        feedback = bit0 ^ other;
        n->sequencer.sequence = (feedback << 14) | ((n->sequencer.sequence & 0x7FFF) >> 1);
        n->sequencer.out = (UINT8)((n->sequencer.sequence & 1) == 0);
    }
}

static bool beesnes_noise_producing_sound(const beesnes_noise_t *n, bool enabled) {
    return enabled && (n->length.counter > 0) && (n->sequencer.out != 0);
}

// --------------------------------------------------------------------------
// DMC Channel
// --------------------------------------------------------------------------

static void beesnes_dmc_init(beesnes_dmc_t *dmc) {
    dmc->sample_address = 0xC000;
    dmc->sample_length = 1;
    dmc->current_address = 0;
    dmc->bytes_remaining = 0;
    dmc->output_level = 0;
    dmc->sample_buffer = 0;
    dmc->shift_register = 0;
    dmc->bits_remaining = 8;
    dmc->start_delay = 0;
    dmc->enable_delay = 0;
    dmc->fetch_cycle = 0;
    dmc->timer = 0;
    dmc->timer_period = 0;
    dmc->irq_enabled = false;
    dmc->loop = false;
    dmc->irq_asserted = false;
    dmc->buffer_empty = true;
    dmc->silent = true;
    dmc->enabled = false;
    dmc->enable_written = false;
    dmc->set_implicit_abort = false;
    dmc->implicit_abort = false;
    dmc->clocked = false;
}

static void beesnes_dmc_tick(beesnes_apu_t *apu) {
    apu->dmc.implicit_abort = false;
    apu->dmc.clocked = false;

    if (apu->dmc.timer > 0) {
        apu->dmc.timer--;
    } else {
        apu->dmc.timer = apu->dmc.timer_period;
        apu->dmc.clocked = true;

        if (!apu->dmc.silent) {
            if (apu->dmc.shift_register & 1) {
                if (apu->dmc.output_level <= 125) {
                    apu->dmc.output_level += 2;
                }
            } else {
                if (apu->dmc.output_level >= 2) {
                    apu->dmc.output_level -= 2;
                }
            }
        }

        apu->dmc.shift_register >>= 1;
        apu->dmc.bits_remaining--;
        if (apu->dmc.bits_remaining == 0) {
            apu->dmc.bits_remaining = 8;

            if (apu->dmc.buffer_empty) {
                apu->dmc.silent = true;
            } else {
                apu->dmc.silent = false;
                apu->dmc.shift_register = apu->dmc.sample_buffer;
                apu->dmc.buffer_empty = true;
            }

            if (apu->dmc.bytes_remaining > 0 || apu->dmc.set_implicit_abort) {
                if (apu->cycles != apu->dmc.fetch_cycle + 1) {
                    beesnes_dmc_fetch(apu);
                }
                apu->dmc.implicit_abort = apu->dmc.set_implicit_abort;
                apu->dmc.set_implicit_abort = false;
            }
        }
    }

    if (apu->dmc.start_delay && --apu->dmc.start_delay == 0 &&
        apu->dmc.buffer_empty && apu->dmc.bytes_remaining) {
        beesnes_dmc_fetch(apu);
    }

    if (apu->dmc.enable_delay && --apu->dmc.enable_delay == 0) {
        apu->dmc.enabled = apu->dmc.enable_written;
        if (!apu->dmc.enabled) {
            apu->dmc.bytes_remaining = 0;
        }
    }
}

static void beesnes_dmc_write4010(beesnes_apu_t *apu, UINT8 val) {
    apu->dmc.irq_enabled = ((val & 0x80) != 0);
    apu->dmc.loop = ((val & 0x40) != 0);
    apu->dmc.timer_period = (UINT16)(beesnes_dmc_periods[apu->type][val & 0x0F] - 1);

    if (!apu->dmc.irq_enabled) {
        beesnes_dmc_clear_irq(apu);
    }
}

static void beesnes_dmc_set_enabled(beesnes_apu_t *apu, bool enabled) {
    bool b_get;

    b_get = (apu->cycles & 1) != 0;

    apu->dmc.enable_written = enabled;
    apu->dmc.enable_delay = (UINT8)(b_get ? 4 : 3);

    if (!enabled) {
        if (b_get ? (apu->dmc.timer == 0) : apu->dmc.clocked) {
            apu->dmc.enable_delay += 2;
        }
    } else {
        if (apu->dmc.timer == (UINT16)(b_get ? 8 : 7)) {
            apu->dmc.set_implicit_abort = true;
        }
        if (apu->dmc.bytes_remaining == 0) {
            apu->dmc.current_address = apu->dmc.sample_address;
            apu->dmc.bytes_remaining = apu->dmc.sample_length;
            if (apu->dmc.buffer_empty) {
                apu->dmc.start_delay = apu->dmc.enable_delay;
            }
        }
    }
}

static void beesnes_dmc_clear_irq(beesnes_apu_t *apu) {
    if (apu->dmc.irq_asserted) {
        apu->dmc.irq_asserted = false;
    }
}

static bool beesnes_dmc_is_active(const beesnes_dmc_t *dmc) {
    return (dmc->bytes_remaining > 0) && dmc->enable_written;
}

static void beesnes_dmc_receive_sample(beesnes_apu_t *apu, UINT8 sample) {
    apu->dmc.sample_buffer = sample;
    apu->dmc.buffer_empty = false;
    apu->dmc.current_address = (UINT16)((apu->dmc.current_address + 1) | 0x8000);
    apu->dmc.fetch_cycle = apu->cycles;

    if (apu->dmc.bytes_remaining && --apu->dmc.bytes_remaining == 0) {
        if (apu->dmc.loop) {
            apu->dmc.current_address = apu->dmc.sample_address;
            apu->dmc.bytes_remaining = apu->dmc.sample_length;
        } else {
            apu->dmc.enabled = false;
            if (apu->dmc.irq_enabled) {
                apu->dmc.irq_asserted = true;
            }
        }
    }
}

// Fetches the DMC's next sample byte from the CPU bus.
static void beesnes_dmc_fetch(beesnes_apu_t *apu) {
    if (apu->cb_dma_read) {
        beesnes_dmc_receive_sample(apu, apu->cb_dma_read(apu->cb_user_data, apu->dmc.current_address));
    }
}

// --------------------------------------------------------------------------
// Main APU Helpers
// --------------------------------------------------------------------------

static UINT8 beesnes_get_duty(const beesnes_apu_t *apu, UINT8 duty_idx) {
    duty_idx &= 0x03;
    if (apu->swap_duty) {
        return beesnes_duty_swapped[duty_idx];
    }
    return beesnes_duty_normal[duty_idx];
}

static void beesnes_clock_frame_units(beesnes_apu_t *apu, UINT8 which) {
    if (which & BEESNES_FC_QUARTER) {
        beesnes_linear_counter_tick(&apu->triangle.linear, (apu->registers[0x08] & 0x80) != 0);
        beesnes_envelope_tick(&apu->pulse1.envelope, (apu->registers[0x00] & 0x10) != 0, (apu->registers[0x00] & 0x20) != 0);
        beesnes_envelope_tick(&apu->pulse2.envelope, (apu->registers[0x04] & 0x10) != 0, (apu->registers[0x04] & 0x20) != 0);
        beesnes_envelope_tick(&apu->noise.envelope,  (apu->registers[0x0C] & 0x10) != 0, (apu->registers[0x0C] & 0x20) != 0);
    }
    if (which & BEESNES_FC_HALF) {
        beesnes_length_counter_tick(&apu->pulse1.length,   (apu->registers[0x15] & 0x01) != 0, (apu->registers[0x00] & 0x20) != 0);
        beesnes_length_counter_tick(&apu->pulse2.length,   (apu->registers[0x15] & 0x02) != 0, (apu->registers[0x04] & 0x20) != 0);
        beesnes_length_counter_tick(&apu->triangle.length, (apu->registers[0x15] & 0x04) != 0, (apu->registers[0x08] & 0x80) != 0);
        beesnes_length_counter_tick(&apu->noise.length,    (apu->registers[0x15] & 0x08) != 0, (apu->registers[0x0C] & 0x20) != 0);

        apu->pulse1.sequencer.reload = beesnes_sweeper_tick(&apu->pulse1.sweeper, apu->pulse1.sequencer.reload, 1);
        apu->pulse2.sequencer.reload = beesnes_sweeper_tick(&apu->pulse2.sweeper, apu->pulse2.sequencer.reload, 0);
    }
}

// --------------------------------------------------------------------------
// Public API Implementation
// --------------------------------------------------------------------------

void beesnes_apu_init(beesnes_apu_t *apu, beesnes_apu_type_t type) {
    memset(apu, 0, sizeof(*apu));
    apu->type = type;
    apu->swap_duty = (type == BEESNES_AT_PALM || type == BEESNES_AT_PALN);
    apu->letterless = false;

    // Delayed values initialization matching BeesNES delays
    beesnes_delayed_val_init(&apu->dv_4017, 4, cb_set_4017, apu);
    beesnes_delayed_val_init(&apu->dv_pulse1_length, 2, cb_pulse1_length, apu);
    beesnes_delayed_val_init(&apu->dv_pulse2_length, 2, cb_pulse2_length, apu);
    beesnes_delayed_val_init(&apu->dv_triangle_length, 2, cb_triangle_length, apu);
    beesnes_delayed_val_init(&apu->dv_noise_length, 2, cb_noise_length, apu);
    beesnes_delayed_val_init(&apu->dv_pulse1_halt, 2, cb_pulse1_halt, apu);
    beesnes_delayed_val_init(&apu->dv_pulse2_halt, 2, cb_pulse2_halt, apu);
    beesnes_delayed_val_init(&apu->dv_triangle_halt, 2, cb_triangle_halt, apu);
    beesnes_delayed_val_init(&apu->dv_noise_halt, 2, cb_noise_halt, apu);

    beesnes_pulse_init(&apu->pulse1, 1);
    beesnes_pulse_init(&apu->pulse2, 0);
    beesnes_triangle_init(&apu->triangle);
    beesnes_noise_init(&apu->noise);
    beesnes_dmc_init(&apu->dmc);

    beesnes_apu_reset(apu);
}

void beesnes_apu_reset(beesnes_apu_t *apu) {
    size_t i;
    for (i = 0; i < 0x18; ++i) {
        apu->registers[i] = 0x00;
    }
    beesnes_pulse_init(&apu->pulse1, 1);
    beesnes_pulse_init(&apu->pulse2, 0);
    beesnes_noise_init(&apu->noise);
    beesnes_triangle_init(&apu->triangle);
    apu->cycles = 0;

    beesnes_apu_reset_analog(apu);
    beesnes_dmc_init(&apu->dmc);
    beesnes_delayed_val_set(&apu->dv_4017, 0x00, false);

    beesnes_dmc_write4010(apu, apu->registers[0x10]);
    apu->dmc.output_level = apu->registers[0x11] & 0x7F;
    apu->dmc.sample_address = 0xC000 + ((UINT16)apu->registers[0x12] * 64);
    apu->dmc.sample_length = ((UINT16)apu->registers[0x13] * 16) + 1;
}

void beesnes_apu_reset_analog(beesnes_apu_t *apu) {
    apu->registers[0x15] = 0x00;
    apu->cycles &= 1;
    apu->step_cycles = 0;
    apu->frame_irq_clear_cycle = 0;
    apu->frame_irq_flag = false;
    apu->frame_mode = 0;
    apu->frame_step = 0;
    apu->mode_switch = false;
    apu->frame_clock_wait = 0;

    apu->pulse1.sequencer.sequence = beesnes_get_duty(apu, 0);
    apu->pulse2.sequencer.sequence = beesnes_get_duty(apu, 0);
    apu->pulse1.envelope.volume = apu->registers[0x00] & 0x0F;
    apu->pulse2.envelope.volume = apu->registers[0x04] & 0x0F;
    apu->noise.envelope.volume = apu->registers[0x0C] & 0x0F;

    beesnes_apu_write(apu, 0x4015, 0x00);

    apu->ticks_to_len_cntr = (apu->type == BEESNES_AT_PAL) ?
        (INT64)beesnes_timings_pal.m0_s0 : (INT64)beesnes_timings_ntsc.m0_s0;

    if (!apu->letterless || apu->type != BEESNES_AT_NTSC) {
        beesnes_delayed_val_set(&apu->dv_4017, (UINT8)(apu->last_4017 | 0x40), false);
    }

    beesnes_dmc_write4010(apu, apu->registers[0x10]);
    apu->dmc.output_level = apu->registers[0x11] & 0x7F;
    apu->dmc.sample_address = 0xC000 + ((UINT16)apu->registers[0x12] * 64);
    apu->dmc.sample_length = ((UINT16)apu->registers[0x13] * 16) + 1;
}

void beesnes_apu_write(beesnes_apu_t *apu, UINT16 addr, UINT8 val) {
    UINT8 noise_region;

    switch (addr) {
        case 0x4000:
            apu->registers[0x00] = (val & ~0x20) | (apu->registers[0x00] & 0x20);
            beesnes_delayed_val_write(&apu->dv_pulse1_halt, (UINT8)(val & 0x20));
            apu->pulse1.sequencer.sequence = beesnes_get_duty(apu, val >> 6);
            apu->pulse1.envelope.volume = val & 0x0F;
            break;

        case 0x4001:
            apu->registers[0x01] = val;
            apu->pulse1.sweeper.enabled = ((val & 0x80) != 0);
            apu->pulse1.sweeper.period = (val >> 4) & 0x07;
            apu->pulse1.sweeper.negated = ((val & 0x08) != 0);
            apu->pulse1.sweeper.shift = val & 0x07;
            apu->pulse1.sweeper.need_reload = true;
            break;

        case 0x4002:
            apu->registers[0x02] = val;
            apu->pulse1.sequencer.reload = (apu->pulse1.sequencer.reload & 0xFF00) | val;
            break;

        case 0x4003:
            apu->registers[0x03] = val;
            apu->pulse1.sequencer.reload = (apu->pulse1.sequencer.reload & 0x00FF) | (((UINT16)(val & 0x07)) << 8);
            apu->pulse1.sequencer.restart_seq = true;
            if (apu->registers[0x15] & 0x01) {
                beesnes_delayed_val_write(&apu->dv_pulse1_length, beesnes_length_table[(val >> 3) & 0x1F]);
            }
            apu->pulse1.envelope.restart = true;
            break;

        case 0x4004:
            apu->registers[0x04] = (val & ~0x20) | (apu->registers[0x04] & 0x20);
            beesnes_delayed_val_write(&apu->dv_pulse2_halt, (UINT8)(val & 0x20));
            apu->pulse2.sequencer.sequence = beesnes_get_duty(apu, val >> 6);
            apu->pulse2.envelope.volume = val & 0x0F;
            break;

        case 0x4005:
            apu->registers[0x05] = val;
            apu->pulse2.sweeper.enabled = ((val & 0x80) != 0);
            apu->pulse2.sweeper.period = (val >> 4) & 0x07;
            apu->pulse2.sweeper.negated = ((val & 0x08) != 0);
            apu->pulse2.sweeper.shift = val & 0x07;
            apu->pulse2.sweeper.need_reload = true;
            break;

        case 0x4006:
            apu->registers[0x06] = val;
            apu->pulse2.sequencer.reload = (apu->pulse2.sequencer.reload & 0xFF00) | val;
            break;

        case 0x4007:
            apu->registers[0x07] = val;
            apu->pulse2.sequencer.reload = (apu->pulse2.sequencer.reload & 0x00FF) | (((UINT16)(val & 0x07)) << 8);
            apu->pulse2.sequencer.restart_seq = true;
            if (apu->registers[0x15] & 0x02) {
                beesnes_delayed_val_write(&apu->dv_pulse2_length, beesnes_length_table[(val >> 3) & 0x1F]);
            }
            apu->pulse2.envelope.restart = true;
            break;

        case 0x4008:
            apu->registers[0x08] = (val & ~0x80) | (apu->registers[0x08] & 0x80);
            beesnes_delayed_val_write(&apu->dv_triangle_halt, (UINT8)(val & 0x80));
            apu->triangle.linear.linear_reload = val & 0x7F;
            break;

        case 0x400A:
            apu->registers[0x0A] = val;
            apu->triangle.sequencer.reload = (apu->triangle.sequencer.reload & 0xFF00) | val;
            break;

        case 0x400B:
            apu->registers[0x0B] = val;
            apu->triangle.sequencer.reload = (apu->triangle.sequencer.reload & 0x00FF) | (((UINT16)(val & 0x07)) << 8);
            if (apu->registers[0x15] & 0x04) {
                beesnes_delayed_val_write(&apu->dv_triangle_length, beesnes_length_table[(val >> 3) & 0x1F]);
            }
            apu->triangle.linear.reload_linear = true;
            break;

        case 0x400C:
            apu->registers[0x0C] = (val & ~0x20) | (apu->registers[0x0C] & 0x20);
            beesnes_delayed_val_write(&apu->dv_noise_halt, (UINT8)(val & 0x20));
            apu->noise.envelope.volume = val & 0x0F;
            break;

        case 0x400E:
            apu->registers[0x0E] = val;
            if (apu->letterless) {
                apu->noise.sequencer.reload = (UINT16)(beesnes_old_noise_table[val & 0x0F] - 1);
                apu->noise.mode = false;
            } else {
                noise_region = (apu->type == BEESNES_AT_PAL) ? 1 : 0;
                apu->noise.sequencer.reload = (UINT16)(beesnes_noise_table[noise_region][val & 0x0F] - 1);
                apu->noise.mode = ((val & 0x80) != 0);
            }
            break;

        case 0x400F:
            apu->registers[0x0F] = val;
            if (apu->registers[0x15] & 0x08) {
                beesnes_delayed_val_write(&apu->dv_noise_length, beesnes_length_table[(val >> 3) & 0x1F]);
            }
            apu->noise.envelope.restart = true;
            break;

        case 0x4010:
            apu->registers[0x10] = val;
            beesnes_dmc_write4010(apu, val);
            break;

        case 0x4011:
            apu->registers[0x11] = val;
            apu->dmc.output_level = val & 0x7F;
            break;

        case 0x4012:
            apu->registers[0x12] = val;
            apu->dmc.sample_address = 0xC000 + ((UINT16)val * 64);
            break;

        case 0x4013:
            apu->registers[0x13] = val;
            apu->dmc.sample_length = ((UINT16)val * 16) + 1;
            break;

        case 0x4015:
            apu->registers[0x15] = val;
            if ((val & 0x01) == 0) { apu->pulse1.length.counter = 0; }
            if ((val & 0x02) == 0) { apu->pulse2.length.counter = 0; }
            if ((val & 0x04) == 0) { apu->triangle.length.counter = 0; }
            if ((val & 0x08) == 0) { apu->noise.length.counter = 0; }
            beesnes_dmc_clear_irq(apu);
            beesnes_dmc_set_enabled(apu, (val & 0x10) != 0);
            break;

        case 0x4017:
            apu->registers[0x17] = val;
            beesnes_delayed_val_write(&apu->dv_4017, val);
            apu->last_4017 = val;
            if (!apu->frame_clock_wait) {
                apu->frame_clock_wait = (UINT8)(3 - (apu->cycles & 1));
            }
            break;

        default:
            break;
    }
}

UINT8 beesnes_apu_read(beesnes_apu_t *apu, UINT16 addr) {
    UINT8 ret;

    if (addr == 0x4015) {
        ret = 0;

        if (apu->pulse1.length.counter != 0)   { ret |= 0x01; }
        if (apu->pulse2.length.counter != 0)   { ret |= 0x02; }
        if (apu->triangle.length.counter != 0) { ret |= 0x04; }
        if (apu->noise.length.counter != 0)    { ret |= 0x08; }
        if (beesnes_dmc_is_active(&apu->dmc))      { ret |= 0x10; }

        if (apu->frame_irq_flag) {
            ret |= 0x40;
            apu->frame_irq_flag = false;
            apu->frame_irq_clear_cycle = apu->cycles + ((apu->cycles & 1) ? 2 : 1);
        } else if (apu->cycles < apu->frame_irq_clear_cycle) {
            ret |= 0x40;
        }

        if (apu->dmc.irq_asserted) {
            ret |= 0x80;
        }

        return ret;
    }
    return 0;
}

void beesnes_apu_tick(beesnes_apu_t *apu) {
    const beesnes_frame_timings_t *t;
    bool is_odd;
    UINT8 val4017;

    t = (apu->type == BEESNES_AT_PAL) ? &beesnes_timings_pal : &beesnes_timings_ntsc;
    is_odd = (apu->cycles & 1) == 1;

    // 1. Advance delayed pipeline registers
    beesnes_delayed_val_tick(&apu->dv_4017);
    beesnes_delayed_val_tick(&apu->dv_pulse1_length);
    beesnes_delayed_val_tick(&apu->dv_pulse2_length);
    beesnes_delayed_val_tick(&apu->dv_triangle_length);
    beesnes_delayed_val_tick(&apu->dv_noise_length);
    beesnes_delayed_val_tick(&apu->dv_pulse1_halt);
    beesnes_delayed_val_tick(&apu->dv_pulse2_halt);
    beesnes_delayed_val_tick(&apu->dv_triangle_halt);
    beesnes_delayed_val_tick(&apu->dv_noise_halt);

    apu->frame_clocks = 0;

    // 2. Mode switch handling on odd CPU cycles
    if (is_odd) {
        if (apu->mode_switch) {
            val4017 = beesnes_delayed_val_get(&apu->dv_4017);
            apu->mode_switch = false;
            apu->step_cycles = 0;
            apu->frame_step = 0;
            if (val4017 & 0x80) {
                apu->frame_mode = 1;
            } else {
                apu->frame_mode = 0;
            }
            if (val4017 & 0x40) {
                apu->frame_irq_flag = false;
            }
        } else {
            // Sequencer clock on alternate cycles
            beesnes_pulse_tick_sequencer(&apu->pulse1, (apu->registers[0x15] & 0x01) != 0);
            beesnes_pulse_tick_sequencer(&apu->pulse2, (apu->registers[0x15] & 0x02) != 0);
            beesnes_noise_tick_sequencer(&apu->noise,  (apu->registers[0x15] & 0x08) != 0);
        }
    }

    // Triangle sequencer clocks every cycle
    beesnes_triangle_tick_sequencer(&apu->triangle, (apu->registers[0x15] & 0x04) != 0);

    // 3. Frame counter stepping
    if (apu->frame_mode == 0) {
        // 4-Step Mode
        switch (apu->frame_step) {
            case 0:
                apu->ticks_to_len_cntr = (INT64)t->m0_s0;
                if (++apu->step_cycles == t->m0_s0) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER);
                    apu->frame_step = 1;
                }
                break;

            case 1:
                apu->ticks_to_len_cntr = (INT64)(t->m0_s1 - apu->step_cycles - 1);
                if (++apu->step_cycles == t->m0_s1) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER | BEESNES_FC_HALF;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER | BEESNES_FC_HALF);
                    apu->frame_step = 2;
                }
                break;

            case 2:
                apu->ticks_to_len_cntr = (INT64)t->m0_s2;
                if (++apu->step_cycles == t->m0_s2) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER);
                    apu->frame_step = 3;
                }
                break;

            case 3:
                apu->ticks_to_len_cntr = (INT64)((t->m0_s3_2 - 1) - apu->step_cycles - 1);
                if (apu->step_cycles >= (t->m0_s3_2 - 3)) {
                    if ((beesnes_delayed_val_get(&apu->dv_4017) & 0x40) == 0) {
                        apu->frame_irq_flag = true;
                    } else {
                        apu->frame_irq_flag = (apu->step_cycles < (t->m0_s3_2 - 1));
                    }
                }

                if ((apu->step_cycles + 1) == (t->m0_s3_2 - 1)) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER | BEESNES_FC_HALF;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER | BEESNES_FC_HALF);
                }

                if (++apu->step_cycles == t->m0_s3_2) {
                    apu->step_cycles = 0;
                    apu->frame_step = 0;
                }
                break;
        }
    } else {
        // 5-Step Mode
        switch (apu->frame_step) {
            case 0:
                apu->ticks_to_len_cntr = (INT64)t->m1_s0;
                if (++apu->step_cycles == t->m1_s0) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER);
                    apu->frame_step = 1;
                }
                break;

            case 1:
                apu->ticks_to_len_cntr = (INT64)(t->m1_s1 - apu->step_cycles - 1);
                if (++apu->step_cycles == t->m1_s1) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER | BEESNES_FC_HALF;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER | BEESNES_FC_HALF);
                    apu->frame_step = 2;
                }
                break;

            case 2:
                apu->ticks_to_len_cntr = (INT64)t->m1_s2;
                if (++apu->step_cycles == t->m1_s2) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER);
                    apu->frame_step = 3;
                }
                break;

            case 3:
                apu->ticks_to_len_cntr = (INT64)t->m1_s3;
                if (++apu->step_cycles == t->m1_s3) {
                    apu->frame_step = 4;
                }
                break;

            case 4:
                apu->ticks_to_len_cntr = (INT64)((t->m1_s4_1 - 1) - apu->step_cycles - 1);
                if ((apu->step_cycles + 1) == (t->m1_s4_1 - 1)) {
                    apu->frame_clocks |= BEESNES_FC_QUARTER | BEESNES_FC_HALF;
                    beesnes_clock_frame_units(apu, BEESNES_FC_QUARTER | BEESNES_FC_HALF);
                }

                if (++apu->step_cycles == t->m1_s4_1) {
                    apu->step_cycles = 0;
                    apu->frame_step = 0;
                }
                break;
        }
    }

    // 4. Delayed clock from $4017 write with bit 7 set
    if (apu->frame_clock_wait) {
        if (--apu->frame_clock_wait == 0 && (apu->registers[0x17] & 0x80)) {
            beesnes_clock_frame_units(apu, (UINT8)(~apu->frame_clocks));
        }
    }

    // 5. Sweeper update
    beesnes_sweeper_update_state(&apu->pulse1.sweeper, apu->pulse1.sequencer.reload, 1);
    beesnes_sweeper_update_state(&apu->pulse2.sweeper, apu->pulse2.sequencer.reload, 0);

    // 6. DMC Tick
    beesnes_dmc_tick(apu);

    apu->cycles++;
}

static bool beesnes_delayed_val_pending(const beesnes_delayed_val_t *dv) {
    return dv->delay != 0 && dv->dirty > 0;
}

// A register write's delayed effect has not landed yet: a $4017 write (mode switch, frame clock), a $4015
// write (DMC start / stop), a length counter load or halt, a sequencer restart. They land within a few cycles.
bool beesnes_apu_delays_pending(const beesnes_apu_t *apu) {
    return apu->mode_switch || apu->frame_clock_wait || apu->dmc.start_delay || apu->dmc.enable_delay ||
        apu->pulse1.sequencer.restart_seq || apu->pulse2.sequencer.restart_seq ||
        apu->triangle.sequencer.restart_seq || apu->noise.sequencer.restart_seq ||
        beesnes_delayed_val_pending(&apu->dv_4017) ||
        beesnes_delayed_val_pending(&apu->dv_pulse1_length) || beesnes_delayed_val_pending(&apu->dv_pulse2_length) ||
        beesnes_delayed_val_pending(&apu->dv_triangle_length) || beesnes_delayed_val_pending(&apu->dv_noise_length) ||
        beesnes_delayed_val_pending(&apu->dv_pulse1_halt) || beesnes_delayed_val_pending(&apu->dv_pulse2_halt) ||
        beesnes_delayed_val_pending(&apu->dv_triangle_halt) || beesnes_delayed_val_pending(&apu->dv_noise_halt);
}

// The channels whose level stays the same however their sequencer moves: the pulses and the noise when
// they produce no sound for another reason than their sequencer output or their envelope is at 0, the
// triangle when its sequencer is halted. Lengths, envelopes and linear counters change only on frame clocks.
static UINT8 beesnes_constant_channels(const beesnes_apu_t *apu) {
    UINT8 constant = 0;
    const beesnes_pulse_t *p;

    p = &apu->pulse1;
    if (!((apu->registers[0x15] & 0x01) && p->length.counter > 0 && p->sequencer.reload >= 8 && !p->sweeper.muted) ||
        beesnes_envelope_get_output(&p->envelope, (apu->registers[0x00] & 0x10) != 0) == 0) {
        constant |= 1 << BEESNES_C_PULSE_0;
    }
    p = &apu->pulse2;
    if (!((apu->registers[0x15] & 0x02) && p->length.counter > 0 && p->sequencer.reload >= 8 && !p->sweeper.muted) ||
        beesnes_envelope_get_output(&p->envelope, (apu->registers[0x04] & 0x10) != 0) == 0) {
        constant |= 1 << BEESNES_C_PULSE_1;
    }
    if (!beesnes_triangle_stepping(&apu->triangle)) {
        constant |= 1 << BEESNES_C_TRIANGLE;
    }
    if (!((apu->registers[0x15] & 0x08) && apu->noise.length.counter > 0) ||
        beesnes_envelope_get_output(&apu->noise.envelope, (apu->registers[0x0C] & 0x10) != 0) == 0) {
        constant |= 1 << BEESNES_C_NOISE;
    }
    return constant;
}

// The next cycles that beesnes_apu_tick() runs without changing the output: the frame counter takes no step,
// the DMC timer does not run out, no delayed write lands, and only the sequencers of channels whose level
// cannot change (or is in `ignore`, a mask of beesnes_channel_t bits the caller does not hear) run out.
// beesnes_apu_skip() runs them at once.
UINT32 beesnes_apu_quiet_cycles(const beesnes_apu_t *apu, UINT8 ignore) {
    const beesnes_frame_timings_t *t;
    UINT64 next_step;
    UINT64 quiet;
    UINT64 limit;
    UINT8 free_running;

    if (beesnes_apu_delays_pending(apu)) {
        return 0;
    }

    // Frame counter: the step_cycles value whose cycle does something. Each step acts when step_cycles
    // reaches its end; the last step of the 4-step mode starts setting the IRQ flag at m0_s3_2 - 3 and the
    // last step of the 5-step mode clocks the units at m1_s4_1 - 1, a cycle before their ends.
    t = (apu->type == BEESNES_AT_PAL) ? &beesnes_timings_pal : &beesnes_timings_ntsc;
    if (apu->frame_mode == 0) {
        switch (apu->frame_step) {
            case 0: next_step = t->m0_s0; break;
            case 1: next_step = t->m0_s1; break;
            case 2: next_step = t->m0_s2; break;
            default: next_step = t->m0_s3_2 - 2; break;
        }
    } else {
        switch (apu->frame_step) {
            case 0: next_step = t->m1_s0; break;
            case 1: next_step = t->m1_s1; break;
            case 2: next_step = t->m1_s2; break;
            case 3: next_step = t->m1_s3; break;
            default: next_step = t->m1_s4_1 - 1; break;
        }
    }
    if (apu->step_cycles + 1 >= next_step) {
        return 0;
    }
    quiet = next_step - 1 - apu->step_cycles;

    // The DMC timer counts every cycle and runs out on the cycle after it reaches 0.
    if (quiet > apu->dmc.timer) {
        quiet = apu->dmc.timer;
    }
    // The sequencers of the channels that can be heard must not run out. The triangle timer counts every
    // cycle; the pulse and noise timers on odd cycles only: n cycles from here hold (n + (cycles & 1)) / 2
    // odd ones.
    free_running = (UINT8)(beesnes_constant_channels(apu) | ignore);
    if (!(free_running & (1 << BEESNES_C_TRIANGLE)) && quiet > apu->triangle.sequencer.timer) {
        quiet = apu->triangle.sequencer.timer;
    }
    limit = 2 * (UINT64)apu->pulse1.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEESNES_C_PULSE_0)) && quiet > limit) {
        quiet = limit;
    }
    limit = 2 * (UINT64)apu->pulse2.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEESNES_C_PULSE_1)) && quiet > limit) {
        quiet = limit;
    }
    limit = 2 * (UINT64)apu->noise.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEESNES_C_NOISE)) && quiet > limit) {
        quiet = limit;
    }
    return (UINT32)quiet;
}

// `ticks` ticks of a sequencer timer: returns how often it ran out (and was reloaded).
static UINT32 beesnes_timer_run(beesnes_sequencer_t *seq, UINT32 ticks) {
    UINT32 period;
    UINT32 rest;

    if (ticks <= seq->timer) {
        seq->timer = (UINT16)(seq->timer - ticks);
        return 0;
    }
    period = (UINT32)seq->reload + 1;
    rest = ticks - seq->timer - 1;  // the ticks after it first ran out
    seq->timer = (UINT16)(seq->reload - rest % period);
    return 1 + rest / period;
}

// The pulse sequencer after `steps` times its timer ran out: each one outputs the duty bit at seq_off, then
// moves seq_off down by one (7 after 0).
static void beesnes_pulse_steps(beesnes_pulse_t *p, UINT32 steps) {
    if (steps) {
        p->sequencer.out = (UINT8)((p->sequencer.sequence >> ((p->sequencer.seq_off - (steps - 1)) & 7)) & 1);
        p->sequencer.seq_off = (UINT8)((p->sequencer.seq_off - steps) & 7);
    }
}

// Runs `cycles` cycles that beesnes_apu_quiet_cycles() reported, with every timer that runs out in them,
// as beesnes_apu_tick() would. The per-cycle flags (frame_clocks, ticks_to_len_cntr, the DMC's clocked and
// implicit_abort) are left as they are: the caller ticks once more with beesnes_apu_tick() before anything
// reads them.
void beesnes_apu_skip(beesnes_apu_t *apu, UINT32 cycles) {
    UINT32 odd = (cycles + (UINT32)(apu->cycles & 1)) / 2;
    UINT32 steps;
    UINT32 bit0;
    UINT32 other;
    beesnes_triangle_t *tri = &apu->triangle;
    beesnes_noise_t *n = &apu->noise;

    beesnes_pulse_steps(&apu->pulse1, beesnes_timer_run(&apu->pulse1.sequencer, odd));
    beesnes_pulse_steps(&apu->pulse2, beesnes_timer_run(&apu->pulse2.sequencer, odd));

    // the triangle only steps while its length and linear counters are not 0
    steps = beesnes_timer_run(&tri->sequencer, cycles);
    if (steps && tri->length.counter != 0 && tri->linear.linear_counter != 0) {
        tri->sequencer.seq_off = (UINT8)((tri->sequencer.seq_off + steps) % 32);
        tri->sequencer.out = beesnes_triangle_table[tri->sequencer.seq_off];
    }

    // the noise shift register steps each time its timer runs out
    for (steps = beesnes_timer_run(&n->sequencer, odd); steps > 0; steps--) {
        bit0 = n->sequencer.sequence & 1;
        other = n->mode ? ((n->sequencer.sequence >> 6) & 1) : ((n->sequencer.sequence >> 1) & 1);
        n->sequencer.sequence = ((bit0 ^ other) << 14) | ((n->sequencer.sequence & 0x7FFF) >> 1);
        n->sequencer.out = (UINT8)((n->sequencer.sequence & 1) == 0);
    }

    apu->dmc.timer = (UINT16)(apu->dmc.timer - cycles);
    apu->step_cycles += cycles;
    apu->cycles += cycles;
}

void beesnes_apu_get_channel_levels(const beesnes_apu_t *apu, UINT8 levels[BEESNES_C_TOTAL]) {
    memset(levels, 0, BEESNES_C_TOTAL);

    if (beesnes_pulse_producing_sound(&apu->pulse1, (apu->registers[0x15] & 0x01) != 0)) {
        levels[BEESNES_C_PULSE_0] = beesnes_envelope_get_output(&apu->pulse1.envelope, (apu->registers[0x00] & 0x10) != 0);
    }

    if (beesnes_pulse_producing_sound(&apu->pulse2, (apu->registers[0x15] & 0x02) != 0)) {
        levels[BEESNES_C_PULSE_1] = beesnes_envelope_get_output(&apu->pulse2.envelope, (apu->registers[0x04] & 0x10) != 0);
    }

    // Unlike the pulses and the noise, the triangle is not silenced when its length or linear counter reaches 0:
    // the sequencer stops, and its DAC keeps the last value. Dropping to 0 here would add a step (a pop) at the
    // start and the end of every note.
    levels[BEESNES_C_TRIANGLE] = apu->triangle.sequencer.out;

    if (beesnes_noise_producing_sound(&apu->noise, (apu->registers[0x15] & 0x08) != 0)) {
        levels[BEESNES_C_NOISE] = beesnes_envelope_get_output(&apu->noise.envelope, (apu->registers[0x0C] & 0x10) != 0);
    }

    levels[BEESNES_C_DMC] = apu->dmc.output_level;
}

void beesnes_apu_set_dma_read(beesnes_apu_t *apu, void *user_data, UINT8 (*dma_read)(void *user_data, UINT16 addr)) {
    apu->cb_user_data = user_data;
    apu->cb_dma_read = dma_read;
}

void beesnes_apu_set_letterless(beesnes_apu_t *apu, bool letterless) {
    apu->letterless = letterless;
}
