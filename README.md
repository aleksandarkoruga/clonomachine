# ULX3S-85F time-stretching sampler

BufGrainPlay grain engine and a Bungee
phase vocoder in fabric (per slot, switchable live), VexRiscv/LiteX SoC, SD sample
library, OSC control, ΔΣ audio out.

```bash
. ./env.sh          # tools/ (yosys, nextpnr, verilator, openFPGALoader, riscv gcc) + .venv (LiteX)
```

`tools/`, `.venv/`, `deps/` (LiteX git clones) and `build/` are not versioned; the LiteX
revisions in use are pinned in `deps.lock`.

## Layout

| Path | What |
|---|---|
| `rtl/sampler_engine.v` | audio engine: 3 slots × 32 grains, per-grain SDRAM line cache, gate, stutter, latency line, algorithm crossfade, stereo mixer (per-slot left/right/both) |
| `rtl/pv_unit.v` | phase vocoder (Bungee Basic) shared by the slots: FFT, CORDIC, partials, overlap-add rings |
| `rtl/dsm_dac.v` | 2nd-order ΔΣ → 4-bit R-2R (mode 0) or 1-bit (mode 1), OSR 256; one per jack channel |
| `rtl/*.hex` | Hann / 2^x / PV sine, CORDIC, synthesis-window ROMs (`scripts/gen_roms.py`) |
| `model/` | bit-exact C++ golden models (engine, `pv_model`), float references (BufGrainPlay, `pv_float` = Bungee), `pv_render`, `test_pv_engine` |
| `sim/` | Verilator: RTL vs model (`make run`), PV unit job by job (`make pvrun`), DSM spectrum test |
| `include/sampler_regs.h` | engine register map (RTL, model, firmware) |
| `include/sampler_events.h` | 8-byte control events + SLIP/CRC framing + RAW_PARAM ids |
| `soc/ulx3s_sampler.py` | LiteX SoC (VexRiscv, SDRAM, SD, SPI flash, ESP32-S3 UART, USB host on US2 (OHCI), sampler) |
| `fw/` | firmware: SD index + SDRAM cache, tempo/slot logic, USB-MIDI host + MIDI pages (`/midimap.json`), Monomachine map, shell |
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
python host/console_cmd.py status usb "list 1"      # one-shot console lines (fixed read time each)
python host/check_wallclock.py                      # board check: bgAdvance latch vs rate
cd sim && make run                                  # RTL == model regression
cd sim && make pvrun                                # pv_unit == pv_model, job by job, all stages
g++ -O2 -std=c++20 -fwrapv -DPV_RTL_DIR=\"$PWD/rtl\" -o pv_render model/pv_render.cpp model/pv_float.cpp model/pv_model.cpp
./pv_render in.wav out_float.wav --speed 0.5 --pitch 3 --loop --fixed out_fpga.wav   # offline PV renders
```

The console and loader talk to the FT231X over libusb (pyftdi) because openFPGALoader
leaves the chip in bit-bang mode. Only one program can hold the board at a time
(bridge, console or loader).

## SD card

FAT32 or exFAT, MBR or GPT. Folders are libraries: `/drums` → slot 0, `/melody` → slot 1
(`/bass`, `/kick` indexed for later). WAV PCM 8/16/24/32-bit or float 32/64-bit, 1–8
channels (mixed to mono), any rate.

- **Banks:** every subfolder is a bank; loose files are packed in name order into banks of
  ≤ 7 MB / 127 files. A slot's whole bank sits in SDRAM: CC1 / `bufNum` switch instantly
  inside it. Program change / `/slot/n/bank` loads the new bank in the background, the
  requested sample first (~0.1–0.3 s), while the old bank keeps playing; the next bank is
  prefetched, so stepping banks upward is usually instant. Loading ~2 MB/s.
- **Long files are split:** a WAV longer than 60 s (or larger than a bank) becomes 4-bar
  parts (16 beats at the tempo in the name, `..._128bpm.wav`, else 120 BPM = 8 s), each a
  sample of its own (`list` shows `name.wav#k`). The parts of a file fill banks of their own
  (`list 1`: `0014 38-Audio #0-9`, `#10-19`, ...): program change steps through the file,
  CC1 picks the part. A remainder of at least ¼ part becomes a last part ending at the
  file end. Constants in `fw/wavutil.h`; `host/test_wavutil.cpp` checks the conversion.
- **Index:** `<dir>/.smpidx` caches format, length, BPM, peak per WAV (name + size + date);
  only new or changed files are parsed (731 files index in ~1.2 s).
- **Seamless switching:** every grain keeps reading the buffer it started in, the playhead
  moves to the same sixteenth of the bar in the new loop: switches crossfade, no gap.
- **Preparing a card on the PC** (faster loads, nothing left to analyse on the board), also
  after adding new samples: `host/prepare_sd.sh /run/media/$USER/FPGA` reports,
  `... --write` backs the card up to `~/Music/fpga_sd_backup_<date>`, converts every WAV to
  mono 16-bit (own rate, normalised) and writes `.smpidx` / `.smptr` with the firmware's own
  code. Drum loops that are not 2^k beats get a fitting whole-beats tempo (`list` shows
  "N beats (PC)"); uncertain ones are listed as "maybe". The script builds the tool itself.
- **Transients:** `<dir>/.smptr` caches up to 64 onsets per sample, found when a sample is
  first loaded (`list` shows the count, the boot line how many are analysed so far); used
  by `noteMode` 1/2.

## MIDI (USB-MIDI cable on US2)

A class-compliant USB-MIDI interface on **US2** (tested: CH345 cables `1a86:752d`; "U2MIDI Pro" `7104:1400`, which stays clean with the board's audio going into the Machinedrum) is
enumerated by the firmware. MIDI pages from **`/midimap.json`** on the card (written with the
defaults when missing; edit on a PC, `midimap reload` on the console) map channels to voices:
default Machinedrum MIDI machines on channel 5 (drums, voice 0), 3 (melody, voice 1), 6
(third voice). Consecutive CCs from the page's first one (16, 16, 25) drive bufNum bank rate
pitch gate interval (the 6 the MD's MIDI machine sends), then repeats fxSend overlap randPos
randomPitch prob separation maxGrains tempoLock bgAdvance envHold pvTransient; velocity =
volume, pitch bend = cutter handover (voice 0) / noteMode (the MD sends only its 7 coarse bits),
mod wheel = grains / time stretcher, aftertouch = grainDur. `midimap defaults` rewrites the
file with these (the old one stays as `/midimap.json.bak`). Other channels use the Monomachine
table.

- **Clock:** the global bpm follows MIDI clock (least-squares slope over the last 96 ticks,
  ticks the cable lost counted in from the interval, 0.1 bpm steps); `clock 0` turns that off. **Start** restarts the beat and every background playhead, **Stop**
  closes every gate, **Continue** changes nothing.
- The Elektron TM-1 does not come up on US2 (it reboots: US2 seems to give it too little 5 V);
  TurboMIDI from the Machinedrum can't be read by a plain cable (set the MD to 1x).
- **Note-off closes the gate:** a short note length on the sending track (MD MID machine LEN)
  gives clicks.
- **CH345 cables:** running-status messages arrive without their status byte (the firmware puts
  it back), and the input is not opto-isolated: with the board's audio going into the
  Machinedrum it drops out for 40-400 ms whenever the board plays (notes in the gap are lost).
  An isolated MIDI input fixes that (audio isolation transformer, an opto-isolated interface,
  or a DIN opto input on a GPIO).
- The host keeps 8 IN transfers queued and also collects them inside SD reads (time-stamped),
  so loading banks doesn't drop or delay-stamp MIDI.
- Console: `usb`, `usbdesc`, `midimon 1` (decoded) / `2` (raw, timestamped) / `3` (raw incl.
  clock), `midimap [reload]`, `midiin <st> <d1> <d2>` (hex).

## Behaviour notes (design decisions)

- Grain engine = BufGrainPlay: "OLA stretch" and "granular" are only parameter values.
  `/slot/n/` names = BufGrainPlay arguments (`bufNum pos rate pitch grainDur overlap prob
  separation randPos randomPitch`); `pos` ≥ 0 pins, < 0 runs free.
- `/slot/n/algo 1`: phase vocoder (Bungee Basic) with the same rate/pitch/pos/gate
  semantics; switches live with a 5 ms crossfade. Every slot output has a constant
  2.7 ms latency (the PV start-up time), so grain and PV slots stay in sync. Pinned pos +
  PV = spectral freeze. Paulstretch: next.
- `/slot/n/noteMode`: 0 note → equal 16ths (as SC), 1 note k → k-th transient, 2 16ths
  snapped to the nearest transient.
- Gate: `(CC10+1)/32` bar in real time (`240/bpm` s per bar, measured engine rate),
  identical for every loop at a given bpm. A note-off ends the gate early (1 ms ramp), so
  the gate length is the longest a note plays; `envHold 1` keeps it open until the note-off.
- Stutter: TRIG latches POS; `REPEATS` retrigger pulses every `INTERVAL` samples
  re-latch the playhead (no record buffer).
- Engine rate 48014.3 Hz (PLL 49.1667 MHz); tempo and pitch are compensated.
- Outputs: slot 0 (drums) on the left channel, every other slot on the right;
  `/slot/n/out` 0 left, 1 right, 2 both.
- Voices: 0 plays `/drums`, 1 and 2 play `/melody`. Defaults: time stretcher, drum notes on
  transients, melody voices with `bgAdvance` (a note latches pos + the wall clock).
- `bgAdvance`: the background playhead is the wall clock (tempo only); a trigger latches
  pos + background, then the playhead runs at `rate` from there (slow, reverse...).

## Board facts (this unit)

ULX3S v3.0.8, LFE5U-85F, SDRAM MT48LC16M16-compatible (32 MB, memtest OK),
SPI flash ISSI IS25LP128 (bitstream @0, firmware boot image @4 MB).
ESP32-S3 UART reserved on GP0 (FPGA TX) / GN0 (FPGA RX), 3 Mbaud; GP1 frame-ready.
