// MIT License

// Copyright (c) 2026 L-Spiro

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

#include "beenes_apu.h"
#include <string.h>

// --------------------------------------------------------------------------
// Hardware tables
// --------------------------------------------------------------------------

// Length table (32 entries)
static const UINT8 beenes_length_table[32] = {
    10, 254, 20,   2, 40,   4, 80,   6, 160,   8, 60,  10, 14,  12, 26,  14,
    12,  16, 24,  18, 48,  20, 96,  22, 192,  24, 72,  26, 16,  28, 32,  30
};

// Noise period tables: NTSC (0) and PAL (1)
static const UINT16 beenes_noise_table[2][16] = {
    { 2, 4, 8, 16, 32, 48, 64, 80, 101, 127, 190, 254, 381, 508, 1017, 2034 },
    { 2, 4, 7, 15, 30, 44, 59, 74,  94, 118, 177, 236, 354, 472,  945, 1889 }
};

// Old noise table for earliest RP2A03 (letterless)
static const UINT16 beenes_old_noise_table[16] = {
    2, 4, 8, 16, 32, 48, 64, 80, 101, 127, 190, 254, 381, 508, 1017, 1023
};

// 32-step triangle sequence table
static const UINT8 beenes_triangle_table[32] = {
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

// Duty sequence waveforms
static const UINT8 beenes_duty_normal[4] = {
    0x80, // 12.5%
    0xC0, // 25.0%
    0xF0, // 50.0%
    0x3F  // 75.0%
};

static const UINT8 beenes_duty_swapped[4] = {
    0x80, // 12.5%
    0xF0, // 50.0%
    0xC0, // 25.0%
    0x3F  // 75.0%
};

// DMC rate countdown periods (CPU cycles)
static const UINT16 beenes_dmc_periods[5][16] = {
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
typedef struct beenes_frame_timings_s {
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
} beenes_frame_timings_t;

static const beenes_frame_timings_t beenes_timings_ntsc = {
    7457, 14913, 22371, 29828, 29829, 29830,
    7457, 14913, 22371, 29829, 37281, 37282
};

static const beenes_frame_timings_t beenes_timings_pal = {
    8313, 16627, 24939, 33252, 33253, 33254,
    8313, 16627, 24939, 33253, 41565, 41566
};

// --------------------------------------------------------------------------
// Static Helper Prototypes
// --------------------------------------------------------------------------

static UINT8  beenes_get_duty(const beenes_apu_t *apu, UINT8 duty_idx);
static void     beenes_clock_frame_units(beenes_apu_t *apu, UINT8 which);

static void     beenes_delayed_val_init(beenes_delayed_val_t *dv, size_t delay,
                                     void (*cb)(void *, UINT8, UINT8), void *user_data);
static void     beenes_delayed_val_write(beenes_delayed_val_t *dv, UINT8 val);
static void     beenes_delayed_val_set(beenes_delayed_val_t *dv, UINT8 val, bool trigger_callback);
static void     beenes_delayed_val_tick(beenes_delayed_val_t *dv);
static UINT8  beenes_delayed_val_get(const beenes_delayed_val_t *dv);

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
static void     beenes_envelope_init(beenes_envelope_t *env);
static UINT8  beenes_envelope_tick(beenes_envelope_t *env, bool use_volume, bool loop);
static UINT8  beenes_envelope_get_output(const beenes_envelope_t *env, bool use_volume);

static void     beenes_length_counter_init(beenes_length_counter_t *lc);
static UINT8  beenes_length_counter_tick(beenes_length_counter_t *lc, bool enabled, bool halt);

static void     beenes_linear_counter_init(beenes_linear_counter_t *lin);
static UINT8  beenes_linear_counter_tick(beenes_linear_counter_t *lin, bool halt);

static void     beenes_sweeper_init(beenes_sweeper_t *sw);
static UINT16 beenes_sweeper_next_val(UINT16 reload, UINT8 shift, bool negated, UINT8 ones_comp);
static void     beenes_sweeper_update_state(beenes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp);
static UINT16 beenes_sweeper_tick(beenes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp);

static void     beenes_pulse_init(beenes_pulse_t *p, UINT8 ones_comp);
static void     beenes_pulse_tick_sequencer(beenes_pulse_t *p, bool enabled);
static bool     beenes_pulse_producing_sound(const beenes_pulse_t *p, bool enabled);

static void     beenes_triangle_init(beenes_triangle_t *t);
static void     beenes_triangle_tick_sequencer(beenes_triangle_t *t, bool enabled);
static bool     beenes_triangle_producing_sound(const beenes_triangle_t *t, bool enabled);

static void     beenes_noise_init(beenes_noise_t *n);
static void     beenes_noise_tick_sequencer(beenes_noise_t *n, bool enabled);
static bool     beenes_noise_producing_sound(const beenes_noise_t *n, bool enabled);

static void     beenes_dmc_init(beenes_dmc_t *dmc);
static void     beenes_dmc_tick(beenes_apu_t *apu);
static void     beenes_dmc_write4010(beenes_apu_t *apu, UINT8 val);
static void     beenes_dmc_set_enabled(beenes_apu_t *apu, bool enabled);
static void     beenes_dmc_clear_irq(beenes_apu_t *apu);
static void     beenes_dmc_receive_sample(beenes_apu_t *apu, UINT8 sample);
static void     beenes_dmc_fetch(beenes_apu_t *apu);
static bool     beenes_dmc_is_active(const beenes_dmc_t *dmc);

// --------------------------------------------------------------------------
// Delayed Value Implementation
// --------------------------------------------------------------------------

static void beenes_delayed_val_init(beenes_delayed_val_t *dv, size_t delay,
                                 void (*cb)(void *, UINT8, UINT8), void *user_data) {
    memset(dv->buffer, 0, sizeof(dv->buffer));
    memset(dv->is_write, 0, sizeof(dv->is_write));
    dv->delay = delay;
    dv->dirty = 0;
    dv->callback = cb;
    dv->user_data = user_data;
}

static void beenes_delayed_val_write(beenes_delayed_val_t *dv, UINT8 val) {
    if (dv->delay == 0) {
        if (dv->callback) {
            dv->callback(dv->user_data, val, dv->buffer[0]);
        }
    }
    dv->buffer[0] = val;
    dv->is_write[0] = 1;
    dv->dirty = dv->delay;
}

static void beenes_delayed_val_set(beenes_delayed_val_t *dv, UINT8 val, bool trigger_callback) {
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

static void beenes_delayed_val_tick(beenes_delayed_val_t *dv) {
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

static UINT8 beenes_delayed_val_get(const beenes_delayed_val_t *dv) {
    return dv->buffer[dv->delay];
}

// Delayed write callback implementations
static void cb_set_4017(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)new_val;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    apu->mode_switch = true;
}

static void cb_pulse1_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->pulse1.length.counter != 0)) {
        apu->pulse1.length.counter = new_val;
    }
}

static void cb_pulse2_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->pulse2.length.counter != 0)) {
        apu->pulse2.length.counter = new_val;
    }
}

static void cb_triangle_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->triangle.length.counter != 0)) {
        apu->triangle.length.counter = new_val;
    }
}

static void cb_noise_length(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (!(apu->ticks_to_len_cntr == 0 && apu->noise.length.counter != 0)) {
        apu->noise.length.counter = new_val;
    }
}

static void cb_pulse1_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x00] |= 0x20;
    } else {
        apu->registers[0x00] &= ~0x20;
    }
}

static void cb_pulse2_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x04] |= 0x20;
    } else {
        apu->registers[0x04] &= ~0x20;
    }
}

static void cb_triangle_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x08] |= 0x80;
    } else {
        apu->registers[0x08] &= ~0x80;
    }
}

static void cb_noise_halt(void *user_data, UINT8 new_val, UINT8 old_val) {
    beenes_apu_t *apu;
    (void)old_val;
    apu = (beenes_apu_t *)user_data;
    if (new_val) {
        apu->registers[0x0C] |= 0x20;
    } else {
        apu->registers[0x0C] &= ~0x20;
    }
}

// --------------------------------------------------------------------------
// Envelope Generator
// --------------------------------------------------------------------------

static void beenes_envelope_init(beenes_envelope_t *env) {
    env->output = 0;
    env->decay_counter = 0;
    env->divider_counter = 0;
    env->volume = 0;
    env->restart = true;
}

static UINT8 beenes_envelope_tick(beenes_envelope_t *env, bool use_volume, bool loop) {
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

static UINT8 beenes_envelope_get_output(const beenes_envelope_t *env, bool use_volume) {
    return use_volume ? env->volume : env->output;
}

// --------------------------------------------------------------------------
// Length Counter
// --------------------------------------------------------------------------

static void beenes_length_counter_init(beenes_length_counter_t *lc) {
    lc->counter = 0;
}

static UINT8 beenes_length_counter_tick(beenes_length_counter_t *lc, bool enabled, bool halt) {
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

static void beenes_linear_counter_init(beenes_linear_counter_t *lin) {
    lin->linear_counter = 0;
    lin->linear_reload = 0;
    lin->reload_linear = true;
}

static UINT8 beenes_linear_counter_tick(beenes_linear_counter_t *lin, bool halt) {
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

static void beenes_sweeper_init(beenes_sweeper_t *sw) {
    sw->shift = 0;
    sw->timer = 0;
    sw->period = 0;
    sw->enabled = false;
    sw->muted = false;
    sw->negated = false;
    sw->need_reload = true;
}

static UINT16 beenes_sweeper_next_val(UINT16 reload, UINT8 shift, bool negated, UINT8 ones_comp) {
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

static void beenes_sweeper_update_state(beenes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp) {
    UINT16 next_val;
    next_val = beenes_sweeper_next_val(reload, sw->shift, sw->negated, ones_comp);
    sw->muted = (reload < 8) || (!sw->negated && next_val > 0x07FF);
}

static UINT16 beenes_sweeper_tick(beenes_sweeper_t *sw, UINT16 reload, UINT8 ones_comp) {
    if (sw->timer == 0 && sw->enabled && sw->shift > 0 && !sw->muted) {
        reload = beenes_sweeper_next_val(reload, sw->shift, sw->negated, ones_comp);
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

static void beenes_pulse_init(beenes_pulse_t *p, UINT8 ones_comp) {
    beenes_length_counter_init(&p->length);
    beenes_envelope_init(&p->envelope);
    memset(&p->sequencer, 0, sizeof(p->sequencer));
    beenes_sweeper_init(&p->sweeper);
    p->ones_complement = ones_comp;
}

static void beenes_pulse_tick_sequencer(beenes_pulse_t *p, bool enabled) {
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

static bool beenes_pulse_producing_sound(const beenes_pulse_t *p, bool enabled) {
    return enabled &&
           (p->length.counter > 0) &&
           (p->sequencer.reload >= 8) &&
           (p->sequencer.out != 0) &&
           (!p->sweeper.muted);
}

// --------------------------------------------------------------------------
// Triangle Channel
// --------------------------------------------------------------------------

static void beenes_triangle_init(beenes_triangle_t *t) {
    beenes_length_counter_init(&t->length);
    beenes_linear_counter_init(&t->linear);
    memset(&t->sequencer, 0, sizeof(t->sequencer));
    t->sequencer.out = beenes_triangle_table[0];
}

static void beenes_triangle_tick_sequencer(beenes_triangle_t *t, bool enabled) {
    (void)enabled;
    if (t->sequencer.restart_seq) {
        t->sequencer.restart_seq = false;
        t->sequencer.seq_off = 0;
        t->sequencer.out = beenes_triangle_table[t->sequencer.seq_off];
    }
    if (--t->sequencer.timer == 0xFFFF) {
        t->sequencer.timer = t->sequencer.reload;
        if (t->length.counter != 0 && t->linear.linear_counter != 0) {
            t->sequencer.seq_off = (UINT8)((t->sequencer.seq_off + 1) % 32);
            t->sequencer.out = beenes_triangle_table[t->sequencer.seq_off];
        }
    }
}

static bool beenes_triangle_producing_sound(const beenes_triangle_t *t, bool enabled) {
    return enabled && (t->length.counter > 0) && (t->linear.linear_counter > 0);
}

// --------------------------------------------------------------------------
// Noise Channel
// --------------------------------------------------------------------------

static void beenes_noise_init(beenes_noise_t *n) {
    beenes_length_counter_init(&n->length);
    beenes_envelope_init(&n->envelope);
    memset(&n->sequencer, 0, sizeof(n->sequencer));
    n->sequencer.sequence = 1;
    n->mode = false;
}

static void beenes_noise_tick_sequencer(beenes_noise_t *n, bool enabled) {
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

static bool beenes_noise_producing_sound(const beenes_noise_t *n, bool enabled) {
    return enabled && (n->length.counter > 0) && (n->sequencer.out != 0);
}

// --------------------------------------------------------------------------
// DMC Channel
// --------------------------------------------------------------------------

static void beenes_dmc_init(beenes_dmc_t *dmc) {
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

static void beenes_dmc_tick(beenes_apu_t *apu) {
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
                    beenes_dmc_fetch(apu);
                }
                apu->dmc.implicit_abort = apu->dmc.set_implicit_abort;
                apu->dmc.set_implicit_abort = false;
            }
        }
    }

    if (apu->dmc.start_delay && --apu->dmc.start_delay == 0 &&
        apu->dmc.buffer_empty && apu->dmc.bytes_remaining) {
        beenes_dmc_fetch(apu);
    }

    if (apu->dmc.enable_delay && --apu->dmc.enable_delay == 0) {
        apu->dmc.enabled = apu->dmc.enable_written;
        if (!apu->dmc.enabled) {
            apu->dmc.bytes_remaining = 0;
        }
    }
}

static void beenes_dmc_write4010(beenes_apu_t *apu, UINT8 val) {
    apu->dmc.irq_enabled = ((val & 0x80) != 0);
    apu->dmc.loop = ((val & 0x40) != 0);
    apu->dmc.timer_period = (UINT16)(beenes_dmc_periods[apu->type][val & 0x0F] - 1);

    if (!apu->dmc.irq_enabled) {
        beenes_dmc_clear_irq(apu);
    }
}

static void beenes_dmc_set_enabled(beenes_apu_t *apu, bool enabled) {
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

static void beenes_dmc_clear_irq(beenes_apu_t *apu) {
    if (apu->dmc.irq_asserted) {
        apu->dmc.irq_asserted = false;
    }
}

static bool beenes_dmc_is_active(const beenes_dmc_t *dmc) {
    return (dmc->bytes_remaining > 0) && dmc->enable_written;
}

static void beenes_dmc_receive_sample(beenes_apu_t *apu, UINT8 sample) {
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
static void beenes_dmc_fetch(beenes_apu_t *apu) {
    if (apu->cb_dma_read) {
        beenes_dmc_receive_sample(apu, apu->cb_dma_read(apu->cb_user_data, apu->dmc.current_address));
    }
}

// --------------------------------------------------------------------------
// Main APU Helpers
// --------------------------------------------------------------------------

static UINT8 beenes_get_duty(const beenes_apu_t *apu, UINT8 duty_idx) {
    duty_idx &= 0x03;
    if (apu->swap_duty) {
        return beenes_duty_swapped[duty_idx];
    }
    return beenes_duty_normal[duty_idx];
}

static void beenes_clock_frame_units(beenes_apu_t *apu, UINT8 which) {
    if (which & BEENES_FC_QUARTER) {
        beenes_linear_counter_tick(&apu->triangle.linear, (apu->registers[0x08] & 0x80) != 0);
        beenes_envelope_tick(&apu->pulse1.envelope, (apu->registers[0x00] & 0x10) != 0, (apu->registers[0x00] & 0x20) != 0);
        beenes_envelope_tick(&apu->pulse2.envelope, (apu->registers[0x04] & 0x10) != 0, (apu->registers[0x04] & 0x20) != 0);
        beenes_envelope_tick(&apu->noise.envelope,  (apu->registers[0x0C] & 0x10) != 0, (apu->registers[0x0C] & 0x20) != 0);
    }
    if (which & BEENES_FC_HALF) {
        beenes_length_counter_tick(&apu->pulse1.length,   (apu->registers[0x15] & 0x01) != 0, (apu->registers[0x00] & 0x20) != 0);
        beenes_length_counter_tick(&apu->pulse2.length,   (apu->registers[0x15] & 0x02) != 0, (apu->registers[0x04] & 0x20) != 0);
        beenes_length_counter_tick(&apu->triangle.length, (apu->registers[0x15] & 0x04) != 0, (apu->registers[0x08] & 0x80) != 0);
        beenes_length_counter_tick(&apu->noise.length,    (apu->registers[0x15] & 0x08) != 0, (apu->registers[0x0C] & 0x20) != 0);

        apu->pulse1.sequencer.reload = beenes_sweeper_tick(&apu->pulse1.sweeper, apu->pulse1.sequencer.reload, 1);
        apu->pulse2.sequencer.reload = beenes_sweeper_tick(&apu->pulse2.sweeper, apu->pulse2.sequencer.reload, 0);
    }
}

// --------------------------------------------------------------------------
// Public API Implementation
// --------------------------------------------------------------------------

void beenes_apu_init(beenes_apu_t *apu, beenes_apu_type_t type) {
    memset(apu, 0, sizeof(*apu));
    apu->type = type;
    apu->swap_duty = (type == BEENES_AT_PALM || type == BEENES_AT_PALN);
    apu->letterless = false;

    // Delayed values initialization matching BeesNES delays
    beenes_delayed_val_init(&apu->dv_4017, 4, cb_set_4017, apu);
    beenes_delayed_val_init(&apu->dv_pulse1_length, 2, cb_pulse1_length, apu);
    beenes_delayed_val_init(&apu->dv_pulse2_length, 2, cb_pulse2_length, apu);
    beenes_delayed_val_init(&apu->dv_triangle_length, 2, cb_triangle_length, apu);
    beenes_delayed_val_init(&apu->dv_noise_length, 2, cb_noise_length, apu);
    beenes_delayed_val_init(&apu->dv_pulse1_halt, 2, cb_pulse1_halt, apu);
    beenes_delayed_val_init(&apu->dv_pulse2_halt, 2, cb_pulse2_halt, apu);
    beenes_delayed_val_init(&apu->dv_triangle_halt, 2, cb_triangle_halt, apu);
    beenes_delayed_val_init(&apu->dv_noise_halt, 2, cb_noise_halt, apu);

    beenes_pulse_init(&apu->pulse1, 1);
    beenes_pulse_init(&apu->pulse2, 0);
    beenes_triangle_init(&apu->triangle);
    beenes_noise_init(&apu->noise);
    beenes_dmc_init(&apu->dmc);

    beenes_apu_reset(apu);
}

void beenes_apu_reset(beenes_apu_t *apu) {
    size_t i;
    for (i = 0; i < 0x18; ++i) {
        apu->registers[i] = 0x00;
    }
    beenes_pulse_init(&apu->pulse1, 1);
    beenes_pulse_init(&apu->pulse2, 0);
    beenes_noise_init(&apu->noise);
    beenes_triangle_init(&apu->triangle);
    apu->cycles = 0;

    beenes_apu_reset_analog(apu);
    beenes_dmc_init(&apu->dmc);
    beenes_delayed_val_set(&apu->dv_4017, 0x00, false);

    beenes_dmc_write4010(apu, apu->registers[0x10]);
    apu->dmc.output_level = apu->registers[0x11] & 0x7F;
    apu->dmc.sample_address = 0xC000 + ((UINT16)apu->registers[0x12] * 64);
    apu->dmc.sample_length = ((UINT16)apu->registers[0x13] * 16) + 1;
}

void beenes_apu_reset_analog(beenes_apu_t *apu) {
    apu->registers[0x15] = 0x00;
    apu->cycles &= 1;
    apu->step_cycles = 0;
    apu->frame_irq_clear_cycle = 0;
    apu->frame_irq_flag = false;
    apu->frame_mode = 0;
    apu->frame_step = 0;
    apu->mode_switch = false;
    apu->frame_clock_wait = 0;

    apu->pulse1.sequencer.sequence = beenes_get_duty(apu, 0);
    apu->pulse2.sequencer.sequence = beenes_get_duty(apu, 0);
    apu->pulse1.envelope.volume = apu->registers[0x00] & 0x0F;
    apu->pulse2.envelope.volume = apu->registers[0x04] & 0x0F;
    apu->noise.envelope.volume = apu->registers[0x0C] & 0x0F;

    beenes_apu_write(apu, 0x4015, 0x00);

    apu->ticks_to_len_cntr = (apu->type == BEENES_AT_PAL) ?
        (INT64)beenes_timings_pal.m0_s0 : (INT64)beenes_timings_ntsc.m0_s0;

    if (!apu->letterless || apu->type != BEENES_AT_NTSC) {
        beenes_delayed_val_set(&apu->dv_4017, (UINT8)(apu->last_4017 | 0x40), false);
    }

    beenes_dmc_write4010(apu, apu->registers[0x10]);
    apu->dmc.output_level = apu->registers[0x11] & 0x7F;
    apu->dmc.sample_address = 0xC000 + ((UINT16)apu->registers[0x12] * 64);
    apu->dmc.sample_length = ((UINT16)apu->registers[0x13] * 16) + 1;
}

void beenes_apu_write(beenes_apu_t *apu, UINT16 addr, UINT8 val) {
    UINT8 noise_region;

    switch (addr) {
        case 0x4000:
            apu->registers[0x00] = (val & ~0x20) | (apu->registers[0x00] & 0x20);
            beenes_delayed_val_write(&apu->dv_pulse1_halt, (UINT8)(val & 0x20));
            apu->pulse1.sequencer.sequence = beenes_get_duty(apu, val >> 6);
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
                beenes_delayed_val_write(&apu->dv_pulse1_length, beenes_length_table[(val >> 3) & 0x1F]);
            }
            apu->pulse1.envelope.restart = true;
            break;

        case 0x4004:
            apu->registers[0x04] = (val & ~0x20) | (apu->registers[0x04] & 0x20);
            beenes_delayed_val_write(&apu->dv_pulse2_halt, (UINT8)(val & 0x20));
            apu->pulse2.sequencer.sequence = beenes_get_duty(apu, val >> 6);
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
                beenes_delayed_val_write(&apu->dv_pulse2_length, beenes_length_table[(val >> 3) & 0x1F]);
            }
            apu->pulse2.envelope.restart = true;
            break;

        case 0x4008:
            apu->registers[0x08] = (val & ~0x80) | (apu->registers[0x08] & 0x80);
            beenes_delayed_val_write(&apu->dv_triangle_halt, (UINT8)(val & 0x80));
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
                beenes_delayed_val_write(&apu->dv_triangle_length, beenes_length_table[(val >> 3) & 0x1F]);
            }
            apu->triangle.linear.reload_linear = true;
            break;

        case 0x400C:
            apu->registers[0x0C] = (val & ~0x20) | (apu->registers[0x0C] & 0x20);
            beenes_delayed_val_write(&apu->dv_noise_halt, (UINT8)(val & 0x20));
            apu->noise.envelope.volume = val & 0x0F;
            break;

        case 0x400E:
            apu->registers[0x0E] = val;
            if (apu->letterless) {
                apu->noise.sequencer.reload = (UINT16)(beenes_old_noise_table[val & 0x0F] - 1);
                apu->noise.mode = false;
            } else {
                noise_region = (apu->type == BEENES_AT_PAL) ? 1 : 0;
                apu->noise.sequencer.reload = (UINT16)(beenes_noise_table[noise_region][val & 0x0F] - 1);
                apu->noise.mode = ((val & 0x80) != 0);
            }
            break;

        case 0x400F:
            apu->registers[0x0F] = val;
            if (apu->registers[0x15] & 0x08) {
                beenes_delayed_val_write(&apu->dv_noise_length, beenes_length_table[(val >> 3) & 0x1F]);
            }
            apu->noise.envelope.restart = true;
            break;

        case 0x4010:
            apu->registers[0x10] = val;
            beenes_dmc_write4010(apu, val);
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
            beenes_dmc_clear_irq(apu);
            beenes_dmc_set_enabled(apu, (val & 0x10) != 0);
            break;

        case 0x4017:
            apu->registers[0x17] = val;
            beenes_delayed_val_write(&apu->dv_4017, val);
            apu->last_4017 = val;
            if (!apu->frame_clock_wait) {
                apu->frame_clock_wait = (UINT8)(3 - (apu->cycles & 1));
            }
            break;

        default:
            break;
    }
}

UINT8 beenes_apu_read(beenes_apu_t *apu, UINT16 addr) {
    UINT8 ret;

    if (addr == 0x4015) {
        ret = 0;

        if (apu->pulse1.length.counter != 0)   { ret |= 0x01; }
        if (apu->pulse2.length.counter != 0)   { ret |= 0x02; }
        if (apu->triangle.length.counter != 0) { ret |= 0x04; }
        if (apu->noise.length.counter != 0)    { ret |= 0x08; }
        if (beenes_dmc_is_active(&apu->dmc))      { ret |= 0x10; }

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

void beenes_apu_tick(beenes_apu_t *apu) {
    const beenes_frame_timings_t *t;
    bool is_odd;
    UINT8 val4017;

    t = (apu->type == BEENES_AT_PAL) ? &beenes_timings_pal : &beenes_timings_ntsc;
    is_odd = (apu->cycles & 1) == 1;

    // 1. Advance delayed pipeline registers
    beenes_delayed_val_tick(&apu->dv_4017);
    beenes_delayed_val_tick(&apu->dv_pulse1_length);
    beenes_delayed_val_tick(&apu->dv_pulse2_length);
    beenes_delayed_val_tick(&apu->dv_triangle_length);
    beenes_delayed_val_tick(&apu->dv_noise_length);
    beenes_delayed_val_tick(&apu->dv_pulse1_halt);
    beenes_delayed_val_tick(&apu->dv_pulse2_halt);
    beenes_delayed_val_tick(&apu->dv_triangle_halt);
    beenes_delayed_val_tick(&apu->dv_noise_halt);

    apu->frame_clocks = 0;

    // 2. Mode switch handling on odd CPU cycles
    if (is_odd) {
        if (apu->mode_switch) {
            val4017 = beenes_delayed_val_get(&apu->dv_4017);
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
            beenes_pulse_tick_sequencer(&apu->pulse1, (apu->registers[0x15] & 0x01) != 0);
            beenes_pulse_tick_sequencer(&apu->pulse2, (apu->registers[0x15] & 0x02) != 0);
            beenes_noise_tick_sequencer(&apu->noise,  (apu->registers[0x15] & 0x08) != 0);
        }
    }

    // Triangle sequencer clocks every cycle
    beenes_triangle_tick_sequencer(&apu->triangle, (apu->registers[0x15] & 0x04) != 0);

    // 3. Frame counter stepping
    if (apu->frame_mode == 0) {
        // 4-Step Mode
        switch (apu->frame_step) {
            case 0:
                apu->ticks_to_len_cntr = (INT64)t->m0_s0;
                if (++apu->step_cycles == t->m0_s0) {
                    apu->frame_clocks |= BEENES_FC_QUARTER;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER);
                    apu->frame_step = 1;
                }
                break;

            case 1:
                apu->ticks_to_len_cntr = (INT64)(t->m0_s1 - apu->step_cycles - 1);
                if (++apu->step_cycles == t->m0_s1) {
                    apu->frame_clocks |= BEENES_FC_QUARTER | BEENES_FC_HALF;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER | BEENES_FC_HALF);
                    apu->frame_step = 2;
                }
                break;

            case 2:
                apu->ticks_to_len_cntr = (INT64)t->m0_s2;
                if (++apu->step_cycles == t->m0_s2) {
                    apu->frame_clocks |= BEENES_FC_QUARTER;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER);
                    apu->frame_step = 3;
                }
                break;

            case 3:
                apu->ticks_to_len_cntr = (INT64)((t->m0_s3_2 - 1) - apu->step_cycles - 1);
                if (apu->step_cycles >= (t->m0_s3_2 - 3)) {
                    if ((beenes_delayed_val_get(&apu->dv_4017) & 0x40) == 0) {
                        apu->frame_irq_flag = true;
                    } else {
                        apu->frame_irq_flag = (apu->step_cycles < (t->m0_s3_2 - 1));
                    }
                }

                if ((apu->step_cycles + 1) == (t->m0_s3_2 - 1)) {
                    apu->frame_clocks |= BEENES_FC_QUARTER | BEENES_FC_HALF;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER | BEENES_FC_HALF);
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
                    apu->frame_clocks |= BEENES_FC_QUARTER;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER);
                    apu->frame_step = 1;
                }
                break;

            case 1:
                apu->ticks_to_len_cntr = (INT64)(t->m1_s1 - apu->step_cycles - 1);
                if (++apu->step_cycles == t->m1_s1) {
                    apu->frame_clocks |= BEENES_FC_QUARTER | BEENES_FC_HALF;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER | BEENES_FC_HALF);
                    apu->frame_step = 2;
                }
                break;

            case 2:
                apu->ticks_to_len_cntr = (INT64)t->m1_s2;
                if (++apu->step_cycles == t->m1_s2) {
                    apu->frame_clocks |= BEENES_FC_QUARTER;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER);
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
                    apu->frame_clocks |= BEENES_FC_QUARTER | BEENES_FC_HALF;
                    beenes_clock_frame_units(apu, BEENES_FC_QUARTER | BEENES_FC_HALF);
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
            beenes_clock_frame_units(apu, (UINT8)(~apu->frame_clocks));
        }
    }

    // 5. Sweeper update
    beenes_sweeper_update_state(&apu->pulse1.sweeper, apu->pulse1.sequencer.reload, 1);
    beenes_sweeper_update_state(&apu->pulse2.sweeper, apu->pulse2.sequencer.reload, 0);

    // 6. DMC Tick
    beenes_dmc_tick(apu);

    apu->cycles++;
}

static bool beenes_delayed_val_pending(const beenes_delayed_val_t *dv) {
    return dv->delay != 0 && dv->dirty > 0;
}

// The channels whose level stays the same however their sequencer moves: the pulses and the noise when
// they produce no sound for another reason than their sequencer output or their envelope is at 0, the
// triangle when it produces no sound. Lengths, envelopes and linear counters change only on frame clocks.
static UINT8 beenes_constant_channels(const beenes_apu_t *apu) {
    UINT8 constant = 0;
    const beenes_pulse_t *p;

    p = &apu->pulse1;
    if (!((apu->registers[0x15] & 0x01) && p->length.counter > 0 && p->sequencer.reload >= 8 && !p->sweeper.muted) ||
        beenes_envelope_get_output(&p->envelope, (apu->registers[0x00] & 0x10) != 0) == 0) {
        constant |= 1 << BEENES_C_PULSE_0;
    }
    p = &apu->pulse2;
    if (!((apu->registers[0x15] & 0x02) && p->length.counter > 0 && p->sequencer.reload >= 8 && !p->sweeper.muted) ||
        beenes_envelope_get_output(&p->envelope, (apu->registers[0x04] & 0x10) != 0) == 0) {
        constant |= 1 << BEENES_C_PULSE_1;
    }
    if (!beenes_triangle_producing_sound(&apu->triangle, (apu->registers[0x15] & 0x04) != 0)) {
        constant |= 1 << BEENES_C_TRIANGLE;
    }
    if (!((apu->registers[0x15] & 0x08) && apu->noise.length.counter > 0) ||
        beenes_envelope_get_output(&apu->noise.envelope, (apu->registers[0x0C] & 0x10) != 0) == 0) {
        constant |= 1 << BEENES_C_NOISE;
    }
    return constant;
}

// The next cycles that beenes_apu_tick() runs without changing the output: the frame counter takes no step,
// the DMC timer does not run out, no delayed write lands, and only the sequencers of channels whose level
// cannot change (or is in `ignore`, a mask of beenes_channel_t bits the caller does not hear) run out.
// beenes_apu_skip() runs them at once.
UINT32 beenes_apu_quiet_cycles(const beenes_apu_t *apu, UINT8 ignore) {
    const beenes_frame_timings_t *t;
    UINT64 next_step;
    UINT64 quiet;
    UINT64 limit;
    UINT8 free_running;

    if (apu->mode_switch || apu->frame_clock_wait || apu->dmc.start_delay || apu->dmc.enable_delay ||
        apu->pulse1.sequencer.restart_seq || apu->pulse2.sequencer.restart_seq ||
        apu->triangle.sequencer.restart_seq || apu->noise.sequencer.restart_seq) {
        return 0;
    }
    if (beenes_delayed_val_pending(&apu->dv_4017) ||
        beenes_delayed_val_pending(&apu->dv_pulse1_length) || beenes_delayed_val_pending(&apu->dv_pulse2_length) ||
        beenes_delayed_val_pending(&apu->dv_triangle_length) || beenes_delayed_val_pending(&apu->dv_noise_length) ||
        beenes_delayed_val_pending(&apu->dv_pulse1_halt) || beenes_delayed_val_pending(&apu->dv_pulse2_halt) ||
        beenes_delayed_val_pending(&apu->dv_triangle_halt) || beenes_delayed_val_pending(&apu->dv_noise_halt)) {
        return 0;
    }

    // Frame counter: the step_cycles value whose cycle does something. Each step acts when step_cycles
    // reaches its end; the last step of the 4-step mode starts setting the IRQ flag at m0_s3_2 - 3 and the
    // last step of the 5-step mode clocks the units at m1_s4_1 - 1, a cycle before their ends.
    t = (apu->type == BEENES_AT_PAL) ? &beenes_timings_pal : &beenes_timings_ntsc;
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
    free_running = (UINT8)(beenes_constant_channels(apu) | ignore);
    if (!(free_running & (1 << BEENES_C_TRIANGLE)) && quiet > apu->triangle.sequencer.timer) {
        quiet = apu->triangle.sequencer.timer;
    }
    limit = 2 * (UINT64)apu->pulse1.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEENES_C_PULSE_0)) && quiet > limit) {
        quiet = limit;
    }
    limit = 2 * (UINT64)apu->pulse2.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEENES_C_PULSE_1)) && quiet > limit) {
        quiet = limit;
    }
    limit = 2 * (UINT64)apu->noise.sequencer.timer + 1 - (apu->cycles & 1);
    if (!(free_running & (1 << BEENES_C_NOISE)) && quiet > limit) {
        quiet = limit;
    }
    return (UINT32)quiet;
}

// `ticks` ticks of a sequencer timer: returns how often it ran out (and was reloaded).
static UINT32 beenes_timer_run(beenes_sequencer_t *seq, UINT32 ticks) {
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
static void beenes_pulse_steps(beenes_pulse_t *p, UINT32 steps) {
    if (steps) {
        p->sequencer.out = (UINT8)((p->sequencer.sequence >> ((p->sequencer.seq_off - (steps - 1)) & 7)) & 1);
        p->sequencer.seq_off = (UINT8)((p->sequencer.seq_off - steps) & 7);
    }
}

// Runs `cycles` cycles that beenes_apu_quiet_cycles() reported, with every timer that runs out in them,
// as beenes_apu_tick() would. The per-cycle flags (frame_clocks, ticks_to_len_cntr, the DMC's clocked and
// implicit_abort) are left as they are: the caller ticks once more with beenes_apu_tick() before anything
// reads them.
void beenes_apu_skip(beenes_apu_t *apu, UINT32 cycles) {
    UINT32 odd = (cycles + (UINT32)(apu->cycles & 1)) / 2;
    UINT32 steps;
    UINT32 bit0;
    UINT32 other;
    beenes_triangle_t *tri = &apu->triangle;
    beenes_noise_t *n = &apu->noise;

    beenes_pulse_steps(&apu->pulse1, beenes_timer_run(&apu->pulse1.sequencer, odd));
    beenes_pulse_steps(&apu->pulse2, beenes_timer_run(&apu->pulse2.sequencer, odd));

    // the triangle only steps while its length and linear counters are not 0
    steps = beenes_timer_run(&tri->sequencer, cycles);
    if (steps && tri->length.counter != 0 && tri->linear.linear_counter != 0) {
        tri->sequencer.seq_off = (UINT8)((tri->sequencer.seq_off + steps) % 32);
        tri->sequencer.out = beenes_triangle_table[tri->sequencer.seq_off];
    }

    // the noise shift register steps each time its timer runs out
    for (steps = beenes_timer_run(&n->sequencer, odd); steps > 0; steps--) {
        bit0 = n->sequencer.sequence & 1;
        other = n->mode ? ((n->sequencer.sequence >> 6) & 1) : ((n->sequencer.sequence >> 1) & 1);
        n->sequencer.sequence = ((bit0 ^ other) << 14) | ((n->sequencer.sequence & 0x7FFF) >> 1);
        n->sequencer.out = (UINT8)((n->sequencer.sequence & 1) == 0);
    }

    apu->dmc.timer = (UINT16)(apu->dmc.timer - cycles);
    apu->step_cycles += cycles;
    apu->cycles += cycles;
}

void beenes_apu_get_channel_levels(const beenes_apu_t *apu, UINT8 levels[BEENES_C_TOTAL]) {
    memset(levels, 0, BEENES_C_TOTAL);

    if (beenes_pulse_producing_sound(&apu->pulse1, (apu->registers[0x15] & 0x01) != 0)) {
        levels[BEENES_C_PULSE_0] = beenes_envelope_get_output(&apu->pulse1.envelope, (apu->registers[0x00] & 0x10) != 0);
    }

    if (beenes_pulse_producing_sound(&apu->pulse2, (apu->registers[0x15] & 0x02) != 0)) {
        levels[BEENES_C_PULSE_1] = beenes_envelope_get_output(&apu->pulse2.envelope, (apu->registers[0x04] & 0x10) != 0);
    }

    if (beenes_triangle_producing_sound(&apu->triangle, (apu->registers[0x15] & 0x04) != 0)) {
        levels[BEENES_C_TRIANGLE] = apu->triangle.sequencer.out;
    }

    if (beenes_noise_producing_sound(&apu->noise, (apu->registers[0x15] & 0x08) != 0)) {
        levels[BEENES_C_NOISE] = beenes_envelope_get_output(&apu->noise.envelope, (apu->registers[0x0C] & 0x10) != 0);
    }

    levels[BEENES_C_DMC] = apu->dmc.output_level;
}

void beenes_apu_set_dma_read(beenes_apu_t *apu, void *user_data, UINT8 (*dma_read)(void *user_data, UINT16 addr)) {
    apu->cb_user_data = user_data;
    apu->cb_dma_read = dma_read;
}

void beenes_apu_set_letterless(beenes_apu_t *apu, bool letterless) {
    apu->letterless = letterless;
}
