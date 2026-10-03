# ULX3S-85F time-stretching sampler

Implementation of `ULX3S_Sampler_Handoff.md` (phase 1): BufGrainPlay-style OLA/granular
engine in fabric, VexRiscv/LiteX SoC, SD sample library, OSC control, ΔΣ audio out.

```bash
. ./env.sh          # tools/ (yosys, nextpnr, verilator, openFPGALoader, riscv gcc) + .venv (LiteX)
```

`tools/`, `.venv/`, `deps/` (LiteX git clones) and `build/` are not versioned; the LiteX
revisions in use are pinned in `deps.lock`.

## Layout

| Path | What |
|---|---|
| `rtl/sampler_engine.v` | audio engine: 2 slots × 32 grains, per-grain SDRAM line cache, gate, stutter, mixer |
| `rtl/dsm_dac.v` | 2nd-order ΔΣ → 4-bit R-2R (mode 0) or 1-bit (mode 1), OSR 256 |
| `rtl/*.hex` | Hann / 2^x ROMs (`scripts/gen_roms.py`) |
| `model/` | bit-exact C++ golden model + comparison against the float BufGrainPlay |
| `sim/` | Verilator: RTL vs model (`make run`), DSM spectrum test |
| `include/sampler_regs.h` | engine register map (RTL, model, firmware) |
| `include/sampler_events.h` | 8-byte control events + SLIP/CRC framing + RAW_PARAM ids |
| `soc/ulx3s_sampler.py` | LiteX SoC (VexRiscv, SDRAM, SD, SPI flash, ESP32-S3 UART, sampler) |
| `fw/` | firmware: SD index + SDRAM cache, tempo/slot logic, Monomachine map, shell |
| `host/` | loader, console, flash script, OSC bridge, SuperCollider example |

## Build / run

```bash
python soc/ulx3s_sampler.py --build                 # gateware + BIOS (~3 min)
make -C fw                                          # firmware -> fw/sampler.bin
python host/load_fw.py --bit build/ulx3s_sampler/gateware/radiona_ulx3s.bit -i   # RAM, dev loop
host/flash.sh [all|bit|fw]                          # SPI flash: boots standalone at power-up
python host/ftdi_console.py -i                      # shell (status, list, buf, note, set, ...)
python host/osc_bridge.py                           # OSC udp:57130 -> board, 1 Mbaud (see host/sampler_example.scd)
python host/osc_stress.py                           # Monomachine-like load test against the bridge
cd sim && make run                                  # RTL == model regression
```

The console and loader talk to the FT231X over libusb (pyftdi) because openFPGALoader
leaves the chip in bit-bang mode. Only one program can hold the board at a time
(bridge, console or loader).

## SD card

FAT32 or exFAT, MBR or GPT. Folders are libraries: `/drums` → slot 0, `/melody` → slot 1
(`/bass`, `/kick` indexed for later). WAV PCM 8/16/24/32-bit, mono/stereo, any rate.

- **Banks:** every subfolder is a bank; loose files are packed in name order into banks of
  ≤ 7 MB / 127 files. A slot's whole bank sits in SDRAM: CC1 / `bufNum` switch instantly
  inside it. Program change / `/slot/n/bank` loads the new bank in the background, the
  requested sample first (~0.1–0.3 s), while the old bank keeps playing; the next bank is
  prefetched, so stepping banks upward is usually instant. Loading ~2 MB/s.
- **Index:** `<dir>/.smpidx` caches format, length, BPM, peak per WAV (name + size + date);
  only new or changed files are parsed (731 files index in ~1.2 s).
- **Seamless switching:** every grain keeps reading the buffer it started in, the playhead
  moves to the same sixteenth of the bar in the new loop: switches crossfade, no gap.

## Behaviour notes (design decisions)

- One time-domain algorithm (BufGrainPlay): "OLA stretch" and "granular" are only
  parameter values. `/slot/n/` names = BufGrainPlay arguments (`bufNum pos rate pitch
  grainDur overlap prob separation randPos randomPitch`); `pos` ≥ 0 pins, < 0 runs free.
  FFT phase vocoder / paulstretch: later, as separate blocks.
- Gate: `(CC10+1)/32` bar in real time (`240/bpm` s per bar, measured engine rate),
  identical for every loop at a given bpm.
- Stutter: TRIG latches POS; `REPEATS` retrigger pulses every `INTERVAL` samples
  re-latch the playhead (no record buffer).
- Engine rate 48014.3 Hz (PLL 49.1667 MHz); tempo and pitch are compensated.
- External clock (24 ppq `clockTrig`, start/stop, internal/external mode): not yet.

## Board facts (this unit)

ULX3S v3.0.8, LFE5U-85F, SDRAM MT48LC16M16-compatible (32 MB, memtest OK),
SPI flash ISSI IS25LP128 (bitstream @0, firmware boot image @4 MB).
ESP32-S3 UART reserved on GP0 (FPGA TX) / GN0 (FPGA RX), 3 Mbaud; GP1 frame-ready.
