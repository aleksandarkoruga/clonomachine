/* The wavetable synth's parameters (rtl/synth_*.v driven by fw/synth_ctl.c): one list for the
 * firmware, the MIDI map (fw/midimap.c: names in /midimap.json's "synth" page) and the OSC bridge
 * (host/osc_bridge.py parses the SY() lines: /synth/<name> <value in natural units>).
 *
 *   SY(ID, name, kind, lo, hi, default, mod)
 *     kind  SK_LIN  lo + k (hi - lo)          k: the knob, 0..1 (MIDI CC / 127)
 *           SK_EXP  lo (hi / lo)^k
 *           SK_CUBE lo + (hi - lo) k^3        (times: fine near 0)
 *           SK_INT  round(lo + k (hi - lo))   (switches, modes, counts)
 *     mod   1: a mod-matrix destination (modulation adds to the knob, per voice)
 *
 * Mod sources (SYS_*) are unipolar 0..1 except the LFOs (-1..1). One SY() per line. */
#ifndef SYNTH_PARAMS_H
#define SYNTH_PARAMS_H

enum { SK_LIN, SK_EXP, SK_CUBE, SK_INT };

#define SYNTH_PARAMS(SY)                                                                         \
    /* voice / global */                                                                         \
    SY(VOLUME,      volume,      SK_LIN,  0,     1,     0.7,    1)                               \
    SY(MONO,        mono,        SK_INT,  0,     1,     0,      0)                               \
    SY(LEGATO,      legato,      SK_INT,  0,     1,     0,      0)                               \
    SY(PORTA,       porta,       SK_CUBE, 0,     2,     0,      0)  /* glide time, s       */    \
    SY(PORTAALWAYS, portaAlways, SK_INT,  0,     1,     0,      0)                               \
    SY(BENDRANGE,   bendRange,   SK_INT,  0,     24,    2,      0)                               \
    SY(PITCH,       pitch,       SK_LIN,  -12,   12,    0,      1)  /* semitones           */    \
    SY(NOTEOFFSET,  noteOffset,  SK_INT,  -24,   24,    0,      0)  /* added to the MIDI note (arp) */ \
    SY(ARPRETRIG,   arpRetrig,   SK_INT,  0,     1,     0,      0)  /* noteOffset restarts the envelopes */ \
    SY(SCALE,       scale,       SK_INT,  0,     127,   0,      0)  /* fw/scales.c (CC = index) */    \
    SY(TUNING,      tuning,      SK_INT,  0,     127,   0,      0)                               \
    SY(ROOT,        root,        SK_INT,  0,     11,    0,      0)  /* semitones above C   */    \
    SY(SCALEMAP,    scaleMap,    SK_INT,  0,     1,     0,      0)  /* 1: MIDI keys = scale degrees */ \
    SY(FXSEND,      fxSend,      SK_LIN,  0,     1,     0,      0)  /* into the plate reverb */  \
    SY(MACRO1,      macro1,      SK_LIN,  0,     1,     0,      0)                               \
    SY(MACRO2,      macro2,      SK_LIN,  0,     1,     0,      0)                               \
    SY(MACRO3,      macro3,      SK_LIN,  0,     1,     0,      0)                               \
    SY(MACRO4,      macro4,      SK_LIN,  0,     1,     0,      0)                               \
    SY(MODWHEEL,    modwheel,    SK_LIN,  0,     1,     0,      0)                               \
    SY(AFTERTOUCH,  aftertouch,  SK_LIN,  0,     1,     0,      0)                               \
    /* oscillator A */                                                                           \
    SY(A_ON,        aOn,         SK_INT,  0,     1,     1,      0)                               \
    SY(A_TABLE,     aTable,      SK_INT,  0,     127,   0,      0)  /* "synth tables" (CC = index) */    \
    SY(A_LEVEL,     aLevel,      SK_LIN,  0,     1,     0.75,   1)                               \
    SY(A_OCT,       aOct,        SK_INT,  -4,    4,     0,      0)                               \
    SY(A_SEMI,      aSemi,       SK_INT,  -12,   12,    0,      0)                               \
    SY(A_FINE,      aFine,       SK_LIN,  -100,  100,   0,      1)  /* cents               */    \
    SY(A_POS,       aPos,        SK_LIN,  0,     1,     0,      1)  /* table position      */    \
    SY(A_SMOOTH,    aSmooth,     SK_INT,  0,     1,     0,      0)  /* crossfade the frames */   \
    SY(A_WARP,      aWarp,       SK_INT,  0,     19,    0,      0)  /* model/synth_warp.h  */    \
    SY(A_WARPAMT,   aWarpAmt,    SK_LIN,  0,     1,     0,      1)                               \
    SY(A_WARP2,     aWarp2,      SK_INT,  0,     19,    0,      0)                               \
    SY(A_WARP2AMT,  aWarp2Amt,   SK_LIN,  0,     1,     0,      1)                               \
    SY(A_UNISON,    aUnison,     SK_INT,  1,     4,     1,      0)                               \
    SY(A_DETUNE,    aDetune,     SK_LIN,  0,     1,     0.25,   1)  /* outer pair +-k^2 x 2 semitones */ \
    SY(A_BLEND,     aBlend,      SK_LIN,  0,     1,     0.75,   1)  /* centre vs side voices */  \
    SY(A_PHASE,     aPhase,      SK_LIN,  0,     1,     0,      0)  /* start phase         */    \
    SY(A_RAND,      aRand,       SK_LIN,  0,     1,     0,      0)  /* 1: free running     */    \
    SY(A_FILT,      aFilt,       SK_INT,  0,     1,     1,      0)  /* through the filter  */    \
    /* oscillator B */                                                                           \
    SY(B_ON,        bOn,         SK_INT,  0,     1,     0,      0)                               \
    SY(B_TABLE,     bTable,      SK_INT,  0,     127,   1,      0)                               \
    SY(B_LEVEL,     bLevel,      SK_LIN,  0,     1,     0.75,   1)                               \
    SY(B_OCT,       bOct,        SK_INT,  -4,    4,     0,      0)                               \
    SY(B_SEMI,      bSemi,       SK_INT,  -12,   12,    0,      0)                               \
    SY(B_FINE,      bFine,       SK_LIN,  -100,  100,   0,      1)                               \
    SY(B_POS,       bPos,        SK_LIN,  0,     1,     0,      1)                               \
    SY(B_SMOOTH,    bSmooth,     SK_INT,  0,     1,     0,      0)                               \
    SY(B_WARP,      bWarp,       SK_INT,  0,     19,    0,      0)                               \
    SY(B_WARPAMT,   bWarpAmt,    SK_LIN,  0,     1,     0,      1)                               \
    SY(B_WARP2,     bWarp2,      SK_INT,  0,     19,    0,      0)                               \
    SY(B_WARP2AMT,  bWarp2Amt,   SK_LIN,  0,     1,     0,      1)                               \
    SY(B_UNISON,    bUnison,     SK_INT,  1,     4,     1,      0)                               \
    SY(B_DETUNE,    bDetune,     SK_LIN,  0,     1,     0.25,   1)                               \
    SY(B_BLEND,     bBlend,      SK_LIN,  0,     1,     0.75,   1)                               \
    SY(B_PHASE,     bPhase,      SK_LIN,  0,     1,     0,      0)                               \
    SY(B_RAND,      bRand,       SK_LIN,  0,     1,     0,      0)                               \
    SY(B_FILT,      bFilt,       SK_INT,  0,     1,     1,      0)                               \
    /* sub (sine / triangle, octaves below the note) and noise */                                \
    SY(SUB_ON,      subOn,       SK_INT,  0,     1,     0,      0)                               \
    SY(SUB_LEVEL,   subLevel,    SK_LIN,  0,     1,     0.75,   1)                               \
    SY(SUB_OCT,     subOct,      SK_INT,  -3,    0,     -1,     0)                               \
    SY(SUB_SHAPE,   subShape,    SK_INT,  0,     1,     0,      0)  /* 0 sine, 1 triangle  */    \
    SY(SUB_FILT,    subFilt,     SK_INT,  0,     1,     0,      0)                               \
    SY(NOISE_ON,    noiseOn,     SK_INT,  0,     1,     0,      0)                               \
    SY(NOISE_LEVEL, noiseLevel,  SK_LIN,  0,     1,     0.25,   1)                               \
    SY(NOISE_FILT,  noiseFilt,   SK_INT,  0,     1,     1,      0)                               \
    /* filter: four slots blended by an XY morph (equal power: slot 1 at 0,0, 2 at 1,0, 3 at 0,1, 4 at 1,1) */ \
    SY(FILT_ON,     filtOn,      SK_INT,  0,     1,     1,      0)                               \
    SY(CUTOFF,      cutoff,      SK_EXP,  20,    20000, 20000,  1)  /* Hz                  */    \
    SY(RES,         res,         SK_EXP,  0.1,   10,    0.707,  1)                               \
    SY(DRIVE,       drive,       SK_LIN,  0,     1,     0,      1)  /* input gain 1 .. 8   */    \
    SY(MORPHX,      morphX,      SK_LIN,  0,     1,     0,      1)                               \
    SY(MORPHY,      morphY,      SK_LIN,  0,     1,     0,      1)                               \
    SY(FILTKEY,     filtKey,     SK_LIN,  0,     1,     0,      0)  /* key tracking, octave per octave */ \
    SY(SLOT1,       slot1,       SK_INT,  0,     2,     0,      0)  /* 0 clean SVF, 1 Moog, 2 TB-303 */ \
    SY(SLOT2,       slot2,       SK_INT,  0,     2,     0,      0)                               \
    SY(SLOT3,       slot3,       SK_INT,  0,     2,     0,      0)                               \
    SY(SLOT4,       slot4,       SK_INT,  0,     2,     0,      0)                               \
    SY(SLOT1TYPE,   slot1Type,   SK_INT,  0,     3,     0,      0)  /* SVF: 0 LP 1 BP 2 HP 3 notch */ \
    SY(SLOT2TYPE,   slot2Type,   SK_INT,  0,     3,     1,      0)                               \
    SY(SLOT3TYPE,   slot3Type,   SK_INT,  0,     3,     2,      0)                               \
    SY(SLOT4TYPE,   slot4Type,   SK_INT,  0,     3,     3,      0)                               \
    SY(SLOT1OCT,    slot1Oct,    SK_LIN,  -4,    4,     0,      0)  /* cutoff offset, octaves */ \
    SY(SLOT2OCT,    slot2Oct,    SK_LIN,  -4,    4,     0,      0)                               \
    SY(SLOT3OCT,    slot3Oct,    SK_LIN,  -4,    4,     0,      0)                               \
    SY(SLOT4OCT,    slot4Oct,    SK_LIN,  -4,    4,     0,      0)                               \
    /* envelopes (env 1 = amplitude): s */                                                       \
    SY(ENV1A,       env1A,       SK_CUBE, 0,     10,    0.0005, 0)                               \
    SY(ENV1H,       env1H,       SK_CUBE, 0,     10,    0,      0)                               \
    SY(ENV1D,       env1D,       SK_CUBE, 0,     10,    1,      0)                               \
    SY(ENV1S,       env1S,       SK_LIN,  0,     1,     1,      0)                               \
    SY(ENV1R,       env1R,       SK_CUBE, 0,     10,    0.015,  0)                               \
    SY(ENV2A,       env2A,       SK_CUBE, 0,     10,    0.0005, 0)                               \
    SY(ENV2H,       env2H,       SK_CUBE, 0,     10,    0,      0)                               \
    SY(ENV2D,       env2D,       SK_CUBE, 0,     10,    0.5,    0)                               \
    SY(ENV2S,       env2S,       SK_LIN,  0,     1,     0,      0)                               \
    SY(ENV2R,       env2R,       SK_CUBE, 0,     10,    0.1,    0)                               \
    SY(ENV3A,       env3A,       SK_CUBE, 0,     10,    0.0005, 0)                               \
    SY(ENV3H,       env3H,       SK_CUBE, 0,     10,    0,      0)                               \
    SY(ENV3D,       env3D,       SK_CUBE, 0,     10,    1,      0)                               \
    SY(ENV3S,       env3S,       SK_LIN,  0,     1,     0,      0)                               \
    SY(ENV3R,       env3R,       SK_CUBE, 0,     10,    0.1,    0)                               \
    /* LFOs: shape 0 sine 1 tri 2 saw down 3 saw up 4 square 5 S&H (fractions crossfade); rate Hz, \
       or cycles per beat when synced (powers of two); mode 0 restarts per note 1 one cycle 2 free */ \
    SY(LFO1SHAPE,   lfo1Shape,   SK_LIN,  0,     5,     0,      0)                               \
    SY(LFO1RATE,    lfo1Rate,    SK_EXP,  0.01,  100,   1,      0)                               \
    SY(LFO1SYNC,    lfo1Sync,    SK_INT,  0,     1,     0,      0)                               \
    SY(LFO1MODE,    lfo1Mode,    SK_INT,  0,     2,     0,      0)                               \
    SY(LFO1RISE,    lfo1Rise,    SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO1DELAY,   lfo1Delay,   SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO1PHASE,   lfo1Phase,   SK_LIN,  0,     1,     0,      0)                               \
    SY(LFO2SHAPE,   lfo2Shape,   SK_LIN,  0,     5,     0,      0)                               \
    SY(LFO2RATE,    lfo2Rate,    SK_EXP,  0.01,  100,   5,      0)                               \
    SY(LFO2SYNC,    lfo2Sync,    SK_INT,  0,     1,     0,      0)                               \
    SY(LFO2MODE,    lfo2Mode,    SK_INT,  0,     2,     0,      0)                               \
    SY(LFO2RISE,    lfo2Rise,    SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO2DELAY,   lfo2Delay,   SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO2PHASE,   lfo2Phase,   SK_LIN,  0,     1,     0,      0)                               \
    SY(LFO3SHAPE,   lfo3Shape,   SK_LIN,  0,     5,     1,      0)                               \
    SY(LFO3RATE,    lfo3Rate,    SK_EXP,  0.01,  100,   1,      0)                               \
    SY(LFO3SYNC,    lfo3Sync,    SK_INT,  0,     1,     1,      0)                               \
    SY(LFO3MODE,    lfo3Mode,    SK_INT,  0,     2,     0,      0)                               \
    SY(LFO3RISE,    lfo3Rise,    SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO3DELAY,   lfo3Delay,   SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO3PHASE,   lfo3Phase,   SK_LIN,  0,     1,     0,      0)                               \
    SY(LFO4SHAPE,   lfo4Shape,   SK_LIN,  0,     5,     5,      0)                               \
    SY(LFO4RATE,    lfo4Rate,    SK_EXP,  0.01,  100,   4,      0)                               \
    SY(LFO4SYNC,    lfo4Sync,    SK_INT,  0,     1,     1,      0)                               \
    SY(LFO4MODE,    lfo4Mode,    SK_INT,  0,     2,     2,      0)                               \
    SY(LFO4RISE,    lfo4Rise,    SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO4DELAY,   lfo4Delay,   SK_CUBE, 0,     10,    0,      0)                               \
    SY(LFO4PHASE,   lfo4Phase,   SK_LIN,  0,     1,     0,      0)                               \
    /* mod matrix: source (SYS_*), destination (index into the mod-destination list: the mod 1  \
       parameters in this order, 0 none), amount -1..1 of the destination's knob range */        \
    SY(MOD1SRC,     mod1Src,     SK_INT,  0,     16,    2,      0)  /* env2                */    \
    SY(MOD1DST,     mod1Dst,     SK_INT,  0,     23,    19,     0)  /* cutoff              */    \
    SY(MOD1AMT,     mod1Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD2SRC,     mod2Src,     SK_INT,  0,     16,    4,      0)  /* lfo1                */    \
    SY(MOD2DST,     mod2Dst,     SK_INT,  0,     23,    2,      0)  /* pitch               */    \
    SY(MOD2AMT,     mod2Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD3SRC,     mod3Src,     SK_INT,  0,     16,    5,      0)  /* lfo2                */    \
    SY(MOD3DST,     mod3Dst,     SK_INT,  0,     23,    5,      0)  /* aPos                */    \
    SY(MOD3AMT,     mod3Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD4SRC,     mod4Src,     SK_INT,  0,     16,    15,     0)  /* modwheel            */    \
    SY(MOD4DST,     mod4Dst,     SK_INT,  0,     23,    19,     0)  /* cutoff              */    \
    SY(MOD4AMT,     mod4Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD5SRC,     mod5Src,     SK_INT,  0,     16,    0,      0)                               \
    SY(MOD5DST,     mod5Dst,     SK_INT,  0,     23,    0,      0)                               \
    SY(MOD5AMT,     mod5Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD6SRC,     mod6Src,     SK_INT,  0,     16,    0,      0)                               \
    SY(MOD6DST,     mod6Dst,     SK_INT,  0,     23,    0,      0)                               \
    SY(MOD6AMT,     mod6Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD7SRC,     mod7Src,     SK_INT,  0,     16,    0,      0)                               \
    SY(MOD7DST,     mod7Dst,     SK_INT,  0,     23,    0,      0)                               \
    SY(MOD7AMT,     mod7Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    SY(MOD8SRC,     mod8Src,     SK_INT,  0,     16,    0,      0)                               \
    SY(MOD8DST,     mod8Dst,     SK_INT,  0,     23,    0,      0)                               \
    SY(MOD8AMT,     mod8Amt,     SK_LIN,  -1,    1,     0,      0)                               \
    /* the harmony generator on the ESP32 (esp/harmony): 1 play, it plays the voices (MIDI notes \
       on the synth channel are ignored then); 2 offset, the chord's root is added to the MIDI   \
       notes (in scale degrees, like noteOffset); chords in scale degrees, through scale / tuning */ \
    SY(HARMONY,     harmony,     SK_INT,  0,     2,     0,      0)  /* 0 off, 1 play, 2 offset */ \
    SY(HARMBEATS,   harmBeats,   SK_INT,  1,     16,    4,      0)  /* beats (harmTrig: trigs) per chord step */ \
    SY(HARMPHRASE,  harmPhrase,  SK_INT,  1,     32,    8,      0)  /* steps per phrase (its cadence at the end) */ \
    SY(HARMOCTAVE,  harmOctave,  SK_INT,  -2,    2,     0,      0)                               \
    SY(HARMVOICES,  harmVoices,  SK_INT,  1,     3,     3,      0)                               \
    SY(HARMGATE,    harmGate,    SK_LIN,  0.05,  1,     1,      0)  /* of the step; 1 = legato */ \
    SY(HARMVEL,     harmVel,     SK_INT,  1,     127,   100,    0)                               \
    SY(HARMMELODY,  harmMelody,  SK_INT,  0,     1,     1,      0)  /* the melody moves the chords */ \
    SY(HARMSEED,    harmSeed,    SK_INT,  0,     127,   0,      0)  /* a new melody from this seed */ \
    SY(HARMTRIG,    harmTrig,    SK_INT,  0,     16,    0,      0)  /* 0 the beat, 1..16 note-ons on that MIDI channel step it */

#define SY_ENUM(id, name, kind, lo, hi, def, mod) SP_##id,
enum { SYNTH_PARAMS(SY_ENUM) SP_NUM };
#undef SY_ENUM

/* mod sources (modNSrc) */
#define SYNTH_SOURCES(S) S(NONE, none) S(ENV1, env1) S(ENV2, env2) S(ENV3, env3) S(LFO1, lfo1) \
    S(LFO2, lfo2) S(LFO3, lfo3) S(LFO4, lfo4) S(VEL, vel) S(NOTE, note) S(RAND, rand)           \
    S(MACRO1, macro1) S(MACRO2, macro2) S(MACRO3, macro3) S(MACRO4, macro4) S(MODWHEEL, modwheel) \
    S(AFTERTOUCH, aftertouch)
#define SYS_ENUM(id, name) SYS_##id,
enum { SYNTH_SOURCES(SYS_ENUM) SYS_NUM };
#undef SYS_ENUM

/* mod destinations (modNDst): 0 none, then the mod = 1 parameters in list order:
   1 volume 2 pitch 3 aLevel 4 aFine 5 aPos 6 aWarpAmt 7 aWarp2Amt 8 aDetune 9 aBlend 10 bLevel
   11 bFine 12 bPos 13 bWarpAmt 14 bWarp2Amt 15 bDetune 16 bBlend 17 subLevel 18 noiseLevel
   19 cutoff 20 res 21 drive 22 morphX 23 morphY */

#endif
