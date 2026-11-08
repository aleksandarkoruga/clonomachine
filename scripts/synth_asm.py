#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
# The filter programs (TB-303, Moog, the gain control) are derived from QuadMorphFilter
# (https://github.com/OTODESK4193/QuadMorphFilter) by OTODESK4193, GNU AGPL v3, by way of
# model/synth_qmf.h; this file and its output rtl/sy_ucode.hex are therefore under the AGPL-3.0
# as well (the rest of this repository is MIT, see LICENSE).
"""Assembler for the synth filter engine's microcode (rtl/synth_filt.v).

The engine runs one program per filter slot (thread), for the three voices at once (lanes), twice
per output sample (the 2x sub-samples). Registers r0..r31 are per instance (thread, lane), r32..r63
are constants shared by all instances; all 32-bit Q8.24, written by the soft core (coefficients,
constants), the states by the programs.

  MAC  d, a, b, c    d = c + (a * b) >> 24          MSUB d, a, b, c   d = c - (a * b) >> 24
  MUL  d, a, b       (MAC with c = r31)
  ADD  d, a, b       SUB d, a, b       MAX d, a, b       MIN d, a, b       ABS d, a
  TANH d, a          RECIP d, a (a > 0)      (function unit: takes two issue slots)
  LDX  d             the slot's input sample (the voice's filter sum)
  OUT  a             the slot's output sample
  END                end of the program (the thread runs it again for the second sub-sample)
All results saturate to 32 bits. One instruction per thread every 4 cycles; a function costs 2.

Word (36 bits): op [35:32], d [31:26], a [25:20], b [19:14], c [13:8], imm [7:0].
Writes rtl/sy_ucode.hex (512 x 36 bit) and model/synth_ucode.h (program starts, register map).
"""
import os, re, sys

OPS = {"NOP": 0, "MAC": 1, "MSUB": 2, "ADD": 3, "SUB": 4, "FN": 5, "MAX": 6, "MIN": 7, "LDX": 8,
       "OUT": 9, "END": 10, "ABS": 12}
FN_TANH, FN_RECIP = 0, 1

# ---- register maps: name -> number, per program (coefficients are written by the soft core)
# shared constants (r32..r63): the same for every instance
CONST = {"ZERO": 63, "ONE": 62, "TWO": 61, "HALF": 60, "eps": 59, "gmin": 58, "gmax": 57, "cm": 56,
         "ca": 55, "c95": 54}
CONST_VALUES = {"ZERO": 0.0, "ONE": 1.0, "TWO": 2.0, "HALF": 0.5, "eps": 1e-6, "gmin": 0.1, "gmax": 15.0,
                "cm": None, "ca": None, "c95": 0.95}  # cm / ca: the meter / AGC one-poles (rate)

def regs(names, coefs):
    m = dict(CONST)
    for i, n in enumerate(coefs):          # coefficients from r31 down
        m[n] = 31 - i
    for i, n in enumerate(names):          # states and temporaries from r0 up
        m[n] = i
    assert len(names) + len(coefs) <= 32, f"too many registers: {len(names)} + {len(coefs)}"
    return m

# AGC wrapper (all models but the clean SVF): mean |x| meters (the plugin uses mean squares;
# magnitudes keep their precision in fixed point), gain = in / out clamped to 0.1 .. 15,
# one-pole smoothed; near self-oscillation (ladders) the soft core's 'byp' lerps it to 1.
def agc(x, y):
    return f"""
    ABS  t0, {x}
    SUB  t0, t0, mi
    MAC  mi, t0, cm, mi
    ABS  t1, {y}
    SUB  t1, t1, mo
    MAC  mo, t1, cm, mo
    MAX  t1, mo, eps
    RECIP t1, t1
    MUL  t0, mi, t1
    MAX  t0, t0, gmin
    MIN  t0, t0, gmax
    SUB  t0, t0, agc
    MAC  agc, t0, ca, agc
    SUB  t0, ONE, agc
    MAC  t0, t0, byp, agc
    MUL  {y}, {y}, t0
"""

AGC_STATE = ["mi", "mo", "agc", "t0", "t1"]
AGC_COEF = ["byp"]

PROGRAMS = {}

# ---- model 0: clean SVF, one stage; output = cl lp + cb bp + ch hp (the type, as coefficients)
PROGRAMS["svf"] = (["x", "hp", "bp", "lp", "y", "s1", "s2"],
                   ["comp", "k1", "g", "h", "cl", "cb", "ch"], """
    LDX  x
    MUL  x, x, comp
    MSUB hp, k1, s1, x        ; x - (2R + g) s1
    SUB  hp, hp, s2
    MUL  hp, hp, h
    MAC  bp, g, hp, s1
    MAC  lp, g, bp, s2
    MAC  s1, g, hp, bp
    MAC  s2, g, bp, lp
    MUL  y, cl, lp
    MAC  y, cb, bp, y
    MAC  y, ch, hp, y
    OUT  y
    END
""")

# ---- model 2: diode (TB-303) ladder, Newton-Raphson with 2 iterations (the plugin: 3); the chain's
# derivative prod(1 - y^2) is formed as the tanh outputs come, so only the last one is kept
def tb303():
    st = ["x", "u", "y", "d", "S1", "S2", "S3", "S4", "s1", "s2", "s3", "s4", "hs"] + AGC_STATE
    co = ["comp", "inv1pg", "G", "G4", "k", "invn", "hg"] + AGC_COEF
    nr = """
    TANH y, u                 ; u_sat
    MSUB d, y, y, ONE
    MAC  y, G, y, S1
    TANH y, y                 ; y1
    MSUB t0, y, y, ONE
    MUL  d, d, t0
    MAC  y, G, y, S2
    TANH y, y                 ; y2
    MSUB t0, y, y, ONE
    MUL  d, d, t0
    MAC  y, G, y, S3
    TANH y, y                 ; y3
    MSUB t0, y, y, ONE
    MUL  d, d, t0
    MAC  y, G, y, S4
    TANH y, y                 ; y4
    MSUB t0, y, y, ONE
    MUL  d, d, t0
    MUL  d, d, G4
    MAC  d, k, d, ONE         ; f' = 1 + k G^4 prod(1 - y^2)
    RECIP d, d
    MAC  t0, k, y, u          ; f = u + k y4 - x
    SUB  t0, t0, x
    MSUB u, t0, d, u
    """
    src = """
    LDX  x
    MUL  x, x, comp
    MUL  S1, s1, inv1pg
    MUL  S2, s2, inv1pg
    MUL  S3, s3, inv1pg
    MUL  S4, s4, inv1pg
    MAC  t0, G, S1, S2        ; sigma = ((G S1 + S2) G + S3) G + S4
    MAC  t0, G, t0, S3
    MAC  t0, G, t0, S4
    MSUB u, k, t0, x
    MUL  u, u, invn
    """ + nr + nr + """
    TANH y, u
    SUB  t0, y, s1            ; stage 1..4: v = (in - s) G, y = tanh(v + s), s += 2v
    MUL  t0, t0, G
    ADD  y, t0, s1
    MAC  s1, t0, TWO, s1
    TANH y, y
    SUB  t0, y, s2
    MUL  t0, t0, G
    ADD  y, t0, s2
    MAC  s2, t0, TWO, s2
    TANH y, y
    SUB  t0, y, s3
    MUL  t0, t0, G
    ADD  y, t0, s3
    MAC  s3, t0, TWO, s3
    TANH y, y
    SUB  t0, y, s4
    MUL  t0, t0, G
    ADD  y, t0, s4
    MAC  s4, t0, TWO, s4
    TANH y, y
    SUB  t0, y, hs            ; 8 Hz high-pass, x 0.95
    MUL  t0, t0, hg
    ADD  t1, t0, hs
    ADD  hs, t1, t0
    SUB  y, y, t1
    MUL  y, y, c95
    """ + agc("x", "y") + """
    OUT  y
    END
"""
    return st, co, src
PROGRAMS["tb303"] = tb303()

# ---- model 1: Moog ladder, NR with 3 iterations on the input tanh, 4 linear one-poles, LP taps
# (the ADAA of the reference is replaced by tanh of the midpoint, its own fallback)
def moog():
    st = ["x", "u", "c", "t", "f", "S", "y", "s1", "s2", "s3", "s4", "pv"] + AGC_STATE
    co = ["comp", "inv1pg", "G", "G2", "G3", "k", "inv", "kg4", "kg4i"] + AGC_COEF
    it = """
    MUL  t, u, inv
    TANH t, t
    MAC  f, kg4, t, u         ; f = u + k G^4 tanh - c
    SUB  f, f, c
    MSUB t, t, t, ONE         ; sech^2
    MAC  t, kg4i, t, ONE      ; f' = 1 + k G^4 inv sech^2
    RECIP t, t
    MSUB u, f, t, u
    """
    src = """
    LDX  x
    MUL  x, x, comp
    MUL  S, s1, inv1pg        ; sigma = G^3 S1 + G^2 S2 + G S3 + S4
    MUL  S, S, G3
    MUL  t, s2, inv1pg
    MAC  S, G2, t, S
    MUL  t, s3, inv1pg
    MAC  S, G, t, S
    MAC  S, s4, inv1pg, S
    MSUB c, k, S, x           ; c = x - k sigma
    ADD  u, c, ZERO
    """ + it * 3 + """
    MUL  t, u, inv            ; tanh at the midpoint of this and the last input
    ADD  y, t, pv
    ADD  pv, t, ZERO
    MUL  y, y, HALF
    TANH u, y
    SUB  t, u, s1             ; 4 linear one-poles
    MUL  t, t, G
    ADD  y, t, s1
    MAC  s1, t, TWO, s1
    SUB  t, y, s2
    MUL  t, t, G
    ADD  y, t, s2
    MAC  s2, t, TWO, s2
    SUB  t, y, s3
    MUL  t, t, G
    ADD  y, t, s3
    MAC  s3, t, TWO, s3
    SUB  t, y, s4
    MUL  t, t, G
    ADD  y, t, s4
    MAC  s4, t, TWO, s4
    """ + agc("x", "y") + """
    OUT  y
    END
"""
    return st, co, src
PROGRAMS["moog"] = moog()


# ---- test programs for the function unit (sim/tb_synth_filt.cpp)
PROGRAMS["t_tanh"] = (["x", "y"], ["sc"], """
    LDX  x
    MUL  x, x, sc
    TANH y, x
    OUT  y
    END
""")
PROGRAMS["t_recip"] = (["x", "y"], ["sc", "of"], """
    LDX  x
    MAC  x, x, sc, of
    RECIP y, x
    OUT  y
    END
""")

def assemble(name, states, coefs, src):
    m = regs(states, coefs)
    words = []
    def r(tok):
        tok = tok.strip()
        if tok in m:
            return m[tok]
        mm = re.fullmatch(r"r(\d+)", tok)
        if mm:
            return int(mm.group(1))
        raise SystemExit(f"{name}: unknown register {tok}")
    for line in src.splitlines():
        line = line.split(";")[0].strip()
        if not line:
            continue
        op, _, rest = line.partition(" ")
        args = [a for a in rest.replace(" ", "").split(",") if a]
        d = a = b = c = 0
        imm = 0
        if op in ("MAC", "MSUB"):
            d, a, b, c = map(r, args)
        elif op == "MUL":
            d, a, b = map(r, args); c = m["ZERO"]; op = "MAC"
        elif op in ("ADD", "SUB", "MAX", "MIN"):
            d, a, b = map(r, args)
        elif op == "ABS":
            d, a = map(r, args)
        elif op in ("TANH", "RECIP"):
            d, a = map(r, args); imm = FN_TANH if op == "TANH" else FN_RECIP; op = "FN"
        elif op == "LDX":
            d = r(args[0])
        elif op == "OUT":
            a = r(args[0])
        elif op in ("END", "NOP"):
            pass
        else:
            raise SystemExit(f"{name}: unknown op {op}")
        words.append(OPS[op] << 32 | d << 26 | a << 20 | b << 14 | c << 8 | imm)
    return words, m

def main():
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    rom, starts, maps, slots = [], {}, {}, {}
    for name, (st, co, src) in PROGRAMS.items():
        w, m = assemble(name, st, co, src)
        starts[name] = len(rom)
        maps[name] = m
        # cost in issue slots: a function takes 2
        slots[name] = len(w) + sum(1 for x in w if x >> 32 == OPS["FN"])
        rom += w
    assert len(rom) <= 512, len(rom)
    with open(os.path.join(root, "rtl", "sy_ucode.hex"), "w") as f:
        for x in rom + [0] * (512 - len(rom)):
            f.write("%09x\n" % x)
    with open(os.path.join(root, "model", "synth_ucode.h"), "w") as f:
        f.write("// Generated by scripts/synth_asm.py: the filter engine's programs (rtl/sy_ucode.hex).\n")
        f.write("#pragma once\n\nnamespace synth_ucode {\n")
        for name in PROGRAMS:
            f.write(f"constexpr int {name.upper()}_START = {starts[name]};  // {slots[name]} issue slots\n")
            for reg, num in sorted(maps[name].items(), key=lambda kv: kv[1]):
                if num < 32:
                    f.write(f"constexpr int {name.upper()}_{reg} = {num};\n")
        for reg, num in CONST.items():
            f.write(f"constexpr int C_{reg.upper()} = {num};\n")
        f.write("}  // namespace synth_ucode\n")
    for name in PROGRAMS:
        print(f"{name:6s} start {starts[name]:3d}  {slots[name]:3d} issue slots per sub-sample "
              f"(budget 128)")

if __name__ == "__main__":
    main()
