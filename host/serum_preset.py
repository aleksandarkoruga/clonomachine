#!/usr/bin/env python3
"""Read Serum 2 presets (.SerumPreset): what the patches use, and their embedded wavetables as
WAVs for host/prepare_wt.sh. The presets and what comes out of them are their authors': keep
them out of the repository. Needs cbor2 and zstandard (.venv/bin/pip install cbor2 zstandard).

File layout: "XferJson\\0", u64 LE length, JSON metadata; then u32 LE size of the decompressed
body, u32 LE format (2), a zstd frame holding a CBOR document (the patch).

  .venv/bin/python -I host/serum_preset.py dump  <preset>            # the patch as JSON
  .venv/bin/python -I host/serum_preset.py keys  <preset>            # its structure, values elided
  .venv/bin/python -I host/serum_preset.py survey <preset>...        # what a set of patches uses
  .venv/bin/python -I host/serum_preset.py tables <dir> <preset>...  # embedded wavetables -> WAVs
                                                                     # (2048-sample frames, 'clm ')
"""
import collections
import json
import struct
import sys

import cbor2
import zstandard


def read_preset(path):
    b = open(path, "rb").read()
    if not b.startswith(b"XferJson\0"):
        raise ValueError(f"{path}: not an Xfer JSON container")
    n = struct.unpack_from("<Q", b, 9)[0]
    meta = json.loads(b[17:17 + n])
    rest = b[17 + n:]
    size, fmt = struct.unpack_from("<II", rest, 0)
    body = zstandard.ZstdDecompressor().decompress(rest[8:], max_output_size=size)
    return meta, fmt, cbor2.loads(body)


def jsonable(x, elide=False, depth=0):
    if isinstance(x, dict):
        return {str(k): jsonable(v, elide, depth + 1) for k, v in x.items()}
    if isinstance(x, list):
        if elide and len(x) > 4:
            return [jsonable(v, elide, depth + 1) for v in x[:2]] + [f"... {len(x)} items"]
        return [jsonable(v, elide, depth + 1) for v in x]
    if isinstance(x, (bytes, bytearray)):
        return f"<{len(x)} bytes>"
    if elide and isinstance(x, (int, float, str)) and depth > 0:
        return type(x).__name__ if not isinstance(x, str) or len(x) > 24 else x
    return x


def params(mod):
    p = mod.get("plainParams") if isinstance(mod, dict) else None
    return p if isinstance(p, dict) else {}


SUBOSC = ("WTOsc", "SampleOsc", "GranularOsc", "SpectralOsc", "MultiSampleOsc")


def survey(paths):
    """Counts over the patches: enabled oscillators and their kind, warps, unison, tables,
    filters, modulation destinations, effects, key zones, envelopes / LFOs in use."""
    c = collections.defaultdict(collections.Counter)
    n = 0
    for path in paths:
        try:
            _, _, p = read_preset(path)
        except Exception as e:  # noqa: BLE001 - report and go on
            print(f"skip {path}: {e}")
            continue
        n += 1
        on_main = 0
        for i in range(3):   # the three main oscillators (A on by default, B / C off)
            o = p.get(f"Oscillator{i}", {})
            pp = params(o)
            if pp.get("kParamEnable", 1.0 if i == 0 else 0.0) < 0.5:
                continue
            on_main += 1
            c["osc params"].update(pp.keys())
            kind = pp.get("kParamType", pp.get("kParamOscType", "default(WT?)"))
            c["osc kind"][str(kind)] += 1
            c["unison"][int(pp.get("kParamUnison", 1))] += 1
            c["octave"][pp.get("kParamOctave", 0.0)] += 1
            if "kParamKeyZoneMin" in pp or "kParamKeyZoneMax" in pp:
                c["key zones"]["osc with a key zone"] += 1
            for sub in SUBOSC:
                so = o.get(f"{sub}{i}")
                if not isinstance(so, dict):
                    continue
                sp = params(so)
                if sp:
                    c["sub-osc params set"][sub] += 1
                if sub == "WTOsc":
                    c["warp 1"][sp.get("kParamWarpMenu", "(none)")] += 1
                    c["warp 2"][sp.get("kParamWarpMenu2", "(none)")] += 1
                    c["wt params"].update(sp.keys())
                    src = "embedded" if "embeddedWTData" in so else "factory path"
                    c["table source"][src] += 1
                    nf = so.get("numFrames")
                    if nf:
                        c["table frames (x2048)"][round(nf / 2048, 2)] += 1
                    if "relativePathToWT" in so:
                        c["table name"][so["relativePathToWT"]] += 1
        c["main oscillators on"][on_main] += 1
        noise, sub = params(p.get("Oscillator3", {})), params(p.get("Oscillator4", {}))
        c["noise / sub"]["noise on"] += noise.get("kParamEnable", 0.0) >= 0.5
        c["noise / sub"]["sub on"] += sub.get("kParamEnable", 0.0) >= 0.5
        if sub.get("kParamEnable", 0.0) >= 0.5:
            c["sub shape"][params(p["Oscillator4"].get("SubOsc4", {})).get("kParamShape", "default")] += 1
        nf = 0
        for k in ("VoiceFilter0", "VoiceFilter1"):
            fp = params(p.get(k, {}))
            if fp.get("kParamEnable", 0.0) >= 0.5:
                nf += 1
                c["filter type"][fp.get("kParamType", "default")] += 1
        c["voice filters on"][nf] += 1
        used = 0
        for k, v in p.items():
            if k.startswith("ModSlot") and isinstance(v, dict) and "destModuleTypeString" in v:
                used += 1
                c["mod destination"][f'{v["destModuleTypeString"]}.{v.get("destModuleParamName", "?")}'] += 1
                c["mod source id"][str(v.get("source"))] += 1
        c["mod slots used (count)"][used // 4 * 4] += 1
        for k in ("FXRack0", "FXRack1", "FXRack2"):
            for fx in p.get(k, {}).get("FX", []) if isinstance(p.get(k), dict) else []:
                c["fx"][next((t for t in fx if t.startswith("FX")), "?")] += 1
        c["envelopes set"][sum(1 for i in range(4) if params(p.get(f"Env{i}", {})))] += 1
        c["lfos set"][sum(1 for i in range(10) if params(p.get(f"LFO{i}", {})))] += 1
        c["global params"].update(params(p.get("Global0", {})).keys())
        for k, v in params(p.get("Global0", {})).items():
            if isinstance(v, str):
                c["global menus"][f"{k}={v}"] += 1
    print(f"{n} patches")
    for name, cnt in c.items():
        items = cnt.most_common(40)
        print(f"\n{name}:")
        for k, v in items:
            print(f"  {v:4d}  {k}")


def write_table_wav(path, data, n0=2048):
    """32-bit float WAV with Serum's 'clm ' chunk (frame size), as model/wt_tables.cpp reads."""
    pcm = struct.pack(f"<{len(data)}f", *data)
    clm = f"<!>{n0:<4d} 10000000 wavetable".encode()
    clm += b"\0" * (len(clm) & 1)
    fmt = struct.pack("<HHIIHH", 3, 1, 44100, 44100 * 4, 4, 32)
    body = (b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt + b"clm " + struct.pack("<I", len(clm)) + clm
            + b"data" + struct.pack("<I", len(pcm)) + pcm)
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", len(body)) + body)


def export_tables(out, paths):
    import os
    os.makedirs(out, exist_ok=True)
    for path in paths:
        _, _, p = read_preset(path)
        base = os.path.basename(path).rsplit(".", 1)[0]
        for i in range(3):
            so = p.get(f"Oscillator{i}", {}).get(f"WTOsc{i}", {})
            d = so.get("embeddedWTData") if isinstance(so, dict) else None
            if not d or len(d) < 2048:
                continue
            d = d[:len(d) // 2048 * 2048]
            name = os.path.join(out, f"{base}_osc{i}.wav")
            write_table_wav(name, d)
            print(f"{name}: {len(d) // 2048} frames ({so.get('tableDisplayName', so.get('relativePathToWT', '?'))})")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    cmd, path = sys.argv[1], sys.argv[2]
    if cmd == "survey":
        survey(sys.argv[2:])
        return 0
    if cmd == "tables" and len(sys.argv) > 3:
        export_tables(sys.argv[2], sys.argv[3:])
        return 0
    meta, fmt, patch = read_preset(path)
    if cmd == "dump":
        print(json.dumps({"meta": meta, "format": fmt, "patch": jsonable(patch)}, indent=1))
    elif cmd == "keys":
        print(json.dumps(jsonable(patch, elide=True), indent=1))
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
