// Song / patch text format, see synth_patch.h.
#include "synth_patch.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace synth {

bool load_song(const std::string& path, Song& s, std::string& err) {
    std::ifstream f(path);
    if (!f) {
        err = "can't open " + path;
        return false;
    }
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        if (auto h = line.find('#'); h != std::string::npos)
            line.resize(h);
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd))
            continue;
        bool ok = true;
        if (cmd == "bpm") {
            ok = bool(in >> s.bpm);
        } else if (cmd == "set") {
            std::string k, v;
            ok = bool(in >> k >> v);
            if (ok && (k == "tableA" || k == "tableB"))
                s.tables[k] = v;
            else if (ok)
                s.set[k] = std::stod(v);
        } else if (cmd == "mod") {
            ModRoute m;
            ok = bool(in >> m.src >> m.dst >> m.amount);
            s.mods.push_back(m);
        } else if (cmd == "note") {
            NoteEv n;
            ok = bool(in >> n.beat >> n.midi >> n.beats);
            in >> n.vel;
            s.notes.push_back(n);
        } else if (cmd == "at") {
            ParamEv p;
            ok = bool(in >> p.beat >> p.name >> p.value);
            s.changes.push_back(p);
        } else if (cmd == "end") {
            ok = bool(in >> s.endBeat);
        } else {
            ok = false;
        }
        if (!ok) {
            err = path + ":" + std::to_string(ln) + ": can't parse '" + line + "'";
            return false;
        }
    }
    std::stable_sort(s.notes.begin(), s.notes.end(), [](auto& a, auto& b) { return a.beat < b.beat; });
    std::stable_sort(s.changes.begin(), s.changes.end(), [](auto& a, auto& b) { return a.beat < b.beat; });
    return true;
}

}  // namespace synth
