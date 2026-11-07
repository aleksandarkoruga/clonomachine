// A synth test: patch parameters, modulation routes, notes and parameter changes, read from a
// small text format shared by the float reference (synth_float) and the fixed-point model
// (synth_model):
//
//   # comment
//   bpm 174
//   set <param> <value>            the patch (names: see the param table in synth_float.cpp)
//   mod <source> <dest> <amount>   a mod-matrix slot (amount in the destination's units)
//   note <beat> <midi> <beats> [velocity 0..1]
//   at <beat> <param> <value>      a parameter change during the song (like a knob or a CC)
//   end <beat>                     song length (default: the last note's release + 1 beat)
#pragma once
#include <map>
#include <string>
#include <vector>

namespace synth {

struct ModRoute {
    std::string src, dst;
    double amount = 0;
};
struct NoteEv {
    double beat = 0, beats = 0, vel = 1;
    int midi = 60;
};
struct ParamEv {
    double beat = 0;
    std::string name;
    double value = 0;
};
struct Song {
    double bpm = 120;
    std::map<std::string, double> set;  // patch values given in the file
    std::vector<ModRoute> mods;
    std::vector<NoteEv> notes;
    std::vector<ParamEv> changes;
    std::map<std::string, std::string> tables;  // "tableA" / "tableB" -> built-in name or WAV path
    double endBeat = -1;
};

bool load_song(const std::string& path, Song& s, std::string& err);

}  // namespace synth
