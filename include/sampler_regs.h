/* Sampler engine register map — shared by RTL testbench, golden model and firmware.
 *
 * Engine registers are written through a single (addr, data) command port:
 *   addr[7:5] = slot (0..6), 7 = global block
 *   addr[4:0] = register index
 * All values are raw fixed point. Firmware converts engineering units
 * to these engine units (see fw/slot.c).
 *
 * Status words are read from a separate 64-word status window:
 *   status[slot*8 + i]  per slot, status[56 + i] global.
 */
#ifndef SAMPLER_REGS_H
#define SAMPLER_REGS_H

#define SMP_NUM_SLOTS_MAX     7
#define SMP_GRAINS_PER_SLOT   32
#define SMP_GLOBAL_SLOT       7
#define SMP_ADDR(slot, reg)   ((((slot) & 7) << 5) | ((reg) & 31))

/* Buffer layout in SDRAM (16-bit words), starting at BASE:
 *   [PREPAD words: copy of the last PREPAD samples][FRAMES samples][POSTPAD: copy of first samples]
 * The engine reads sample i at BASE + PREPAD + i, and i+1 may run into the post-pad. */
#define SMP_PREPAD            64
#define SMP_POSTPAD           128
#define SMP_LINE_WORDS        64

/* ---- per-slot registers ---- */
enum {
    SMP_R_CTRL        = 0,  /* bit0 ENABLE, bit1 POS_PIN, bit2 ENV_HOLD, bit3 POS_BG            */
    SMP_R_STROBE      = 1,  /* write-only: bit0 TRIG (latches POS), bit1 GATE_OFF, bit2 KILL, bit3 SEEK */
    SMP_R_BASE        = 2,  /* u24  SDRAM word address of the buffer (pre-pad start)            */
    SMP_R_FRAMES      = 3,  /* u24  frames (>= 128)                                             */
    SMP_R_POS         = 4,  /* u24.8 sample position latched on TRIG / pinned when POS_PIN      */
    SMP_R_PHASOR_INC  = 5,  /* s8.24 playhead samples/sample = RATE*TEMPO_RATIO*bufSR/engineSR  */
    SMP_R_PITCH       = 6,  /* s8.24 grain read samples/sample = PITCH*bufSR/engineSR           */
    SMP_R_WIN_INC     = 7,  /* u32  2^32 / grain length in samples                              */
    SMP_R_OVERLAP     = 8,  /* u8.24                                                            */
    SMP_R_AMP_TARGET  = 9,  /* u1.16 1/max(OVERLAP,1)                                           */
    SMP_R_PROB        = 10, /* u1.16 (65536 = always)                                           */
    SMP_R_SEPARATION  = 11, /* u0.16                                                            */
    SMP_R_RANDPOS     = 12, /* u24  RAND_POS*FRAMES, span of the random read offset in samples  */
    SMP_R_RANDPITCH   = 13, /* s4.12 octaves                                                    */
    SMP_R_MAX_GRAINS  = 14, /* 1..32                                                            */
    SMP_R_SEED        = 15, /* xorshift32 seed, reseeds on write (0 -> 0x9E3779B9)              */
    SMP_R_ENV_DUR     = 16, /* u24  gate length in samples                                      */
    SMP_R_REPEATS     = 17, /* u7   stutter: retrigger the TRIG position this many times        */
    SMP_R_INTERVAL    = 18, /* u14  stutter period in samples (live)                            */
    SMP_R_ALGO        = 19, /* bits1:0 algorithm (0 grain, 1 phase vocoder), bit2 PV no transient */
    SMP_R_AMP         = 20, /* u4.12                                                            */
    SMP_R_ALGO_AMP    = 21, /* u1.15                                                            */
    SMP_R_OUT         = 22, /* bit0 to the left output, bit1 to the right (reset 3: both)      */
    SMP_R_BG_INC      = 23, /* s8.24 background phasor = TEMPO_RATIO*bufSR/engineSR: the wall
                               clock of POS_BG, without RATE (RATE applies from the latch)    */
    SMP_NUM_SLOT_REGS = 24
};

#define SMP_CTRL_ENABLE    (1u << 0)
#define SMP_CTRL_POS_PIN   (1u << 1)
#define SMP_CTRL_ENV_HOLD  (1u << 2)
#define SMP_CTRL_POS_BG    (1u << 3)   /* position = (POS + background phasor) mod FRAMES */
#define SMP_STROBE_TRIG     (1u << 0)
#define SMP_STROBE_GATE_OFF (1u << 1)
#define SMP_STROBE_KILL     (1u << 2)
#define SMP_STROBE_SEEK     (1u << 3)   /* playhead = POS, no onset / gate / stutter       */
#define SMP_STROBE_BG_RESET (1u << 4)   /* background phasor = 0 (transport start)         */
#define SMP_ALGO_GRAIN      0u          /* BufGrainPlay time-domain grains                  */
#define SMP_ALGO_PV         1u          /* phase vocoder (Bungee Basic), model/pv_model.h   */
#define SMP_ALGO_MASK       3u
#define SMP_ALGO_NO_TRANSIENT (1u << 2) /* PV: no transient partial suppression             */
/* Every slot runs a background phasor that always advances by PHASOR_INC (the sample
   "keeps playing" while the gate is closed). With CTRL.POS_BG, TRIG / stutter latch and
   POS_PIN use (POS + background) mod FRAMES instead of POS. */
/* Output latency: every slot output (grain and phase vocoder path) is delayed by
   SMP_LATENCY samples, the time the phase vocoder needs to start a grain at a new
   position. A change of ALGO crossfades the two paths over SMP_ALGO_XF samples.
   Phase vocoder restarts (old content fades out, new grains from the playhead) on
   TRIG, stutter retrigger, SEEK, KILL and when the slot switches to it. */
#define SMP_LATENCY        128
#define SMP_ALGO_XF        256
#define SMP_PV_LOOKAHEAD   512   /* steady grains are issued this much before they are due */
/* BASE/FRAMES may change while grains play: every grain keeps reading the buffer it
   was started in, so a buffer switch crossfades through the overlap. A playhead past
   the new FRAMES restarts at 0 (firmware normally SEEKs to the equivalent beat). */

/* ---- global registers (slot 7) ---- */
enum {
    SMP_G_MASTER      = 0,  /* u4.12 master gain                                                */
    SMP_G_BEAT_INC    = 1,  /* u0.32 beats per sample = bpm/60/fs * 2^32                        */
    SMP_G_BEAT_CTRL   = 2,  /* write-only: bit0 reset beat phase to 0                           */
    SMP_G_CLR_STATS   = 3   /* write-only: clear miss/drop counters and cycle high-water mark   */
};

/* ---- status words ---- */
enum {
    SMP_S_PHASOR      = 0,  /* u24.8 playhead                                                   */
    SMP_S_ACTIVE      = 1,  /* active grain count                                               */
    SMP_S_GATE        = 2,  /* 0 closed, 1 ramp-up, 2 open, 3 ramp-down, 4 stutter              */
    SMP_S_CACHE_MISS  = 3,  /* saturating miss counter (grain-samples muted)                    */
    SMP_S_OUT         = 4,  /* last slot output sample, s24 (Q8 sample units)                   */
    SMP_S_PV          = 5,  /* bit0 PV active, bits 31:16 crossfade position (u0.16)            */
};
enum {
    SMP_SG_CYCLES_MAX = 56, /* engine cycle high-water mark per sample (budget 1024)            */
    SMP_SG_CYCLES     = 57, /* cycles used by the last sample                                   */
    SMP_SG_BEAT_HI    = 58, /* beat count (integer part)                                        */
    SMP_SG_BEAT_LO    = 59, /* beat phase fraction u0.32                                        */
    SMP_SG_SAMPLES    = 60, /* sample counter                                                   */
    SMP_SG_REQ_DROPS  = 61, /* line requests deferred because the request FIFO was full         */
    SMP_SG_PV_MISSES  = 62, /* phase-vocoder grains that missed their deadline (RTL only)       */
    SMP_SG_MAGIC      = 63  /* 0x534D504C 'SMPL'                                                */
};
#define SMP_MAGIC 0x534D504Cu

#endif
