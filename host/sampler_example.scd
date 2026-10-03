// Driving the ULX3S sampler from SuperCollider through host/osc_bridge.py
// (later the ESP32-S3 over WiFi: same messages, just another NetAddr).
//
//   cd clonomachine && . ./env.sh && python host/osc_bridge.py
//
// Channels are SC's 0-based MIDI channels, exactly what SC sends to the
// Monomachine: chan 9 = slot 0 (/drums), chan 10 = slot 1 (/melody).

~fpga = NetAddr("127.0.0.1", 57130);

// ---- tempo: every loop is stretched to it, the gate is in bars of it -------------------
~fpga.sendMsg("/bpm", 172);

// ---- Monomachine-style control ---------------------------------------------------------
~fpga.sendMsg("/midi/pc", 9, 3);         // program change: bank 3 of /drums (loads in background)
~fpga.sendMsg("/midi/cc", 9, 1, 3);     // CC1: buffer 12 inside the bank (instant)
~fpga.sendMsg("/midi/cc", 9, 2, 63);     // CC2: rate (0 -> 0, 63 -> 1, 127 -> 4)
~fpga.sendMsg("/midi/cc", 9, 7, 63);     // CC7: pitch (0 -> 0.1, 63 -> 1, 127 -> 4)
~fpga.sendMsg("/midi/cc", 9, 10, 7);     // CC10: gate = (7+1)/32 bar = 1/4 bar
~fpga.sendMsg("/midi/cc", 9, 12, 60);    // CC12: grainDur (exp 1 ms .. 1 s)
~fpga.sendMsg("/midi/cc", 9, 13, 90);    // CC13: overlap (exp 0.25 .. 16)
~fpga.sendMsg("/midi/note", 9, 12, 1);   // note 12 = slice 0, velocity 1 = no stutter
~fpga.sendMsg("/midi/note", 9, 20, 4);   // slice 8, velocity 4 = 3 stutter retriggers
~fpga.sendMsg("/midi/bend", 9, 4000);    // stutter interval (lincurve 1e-6..0.25 s)

// ---- BufGrainPlay parameters, same names and meaning as the UGen ----------------------
//   BufGrainPlay.ar(bufNum, pos, rate, pitch, grainDur, overlap, prob, separation, randPos, randomPitch)
~fpga.sendMsg("/slot/0/bufNum", 5);       // inside the slot's bank
~fpga.sendMsg("/slot/0/pos", 0.25);       // >= 0: playhead pinned there (like the UGen)
~fpga.sendMsg("/slot/0/pos", -1);         // < 0: runs free from wherever it is
~fpga.sendMsg("/slot/0/rate", -1);        // reverse (at the global tempo)
~fpga.sendMsg("/slot/0/rate", 1);
~fpga.sendMsg("/slot/0/pitch", 1);
~fpga.sendMsg("/slot/0/grainDur", 0.05);  // seconds
~fpga.sendMsg("/slot/0/overlap", 2);
~fpga.sendMsg("/slot/0/prob", 1);
~fpga.sendMsg("/slot/0/separation", 0);
~fpga.sendMsg("/slot/0/randPos", 0);
~fpga.sendMsg("/slot/0/randomPitch", 0); // octaves

~fpga.sendMsg("/slot/0/bufNum", 5);       // inside the slot's bank
~fpga.sendMsg("/slot/0/pos", 0.25);       // >= 0: playhead pinned there (like the UGen)
~fpga.sendMsg("/slot/0/pos", -1);         // < 0: runs free from wherever it is
~fpga.sendMsg("/slot/1/rate", -1);        // reverse (at the global tempo)
~fpga.sendMsg("/slot/0/rate", 1);
~fpga.sendMsg("/slot/0/pitch", 0.5);
~fpga.sendMsg("/slot/1/grainDur", 0.1);  // seconds
~fpga.sendMsg("/slot/1/overlap", 4);
~fpga.sendMsg("/slot/0/prob", 1);
~fpga.sendMsg("/slot/1/separation", 0);
~fpga.sendMsg("/slot/0/randPos", 0);
~fpga.sendMsg("/slot/0/randomPitch", 0); // octaves


// granular cloud vs. smooth stretch is only a matter of values:
~fpga.sendMsg("/slot/0/grainDur", 0.0071); ~fpga.sendMsg("/slot/0/overlap", 1);
~fpga.sendMsg("/slot/0/grainDur", 0.04);   ~fpga.sendMsg("/slot/0/overlap", 4);
~fpga.sendMsg("/slot/0/prob", 1); ~fpga.sendMsg("/slot/0/separation", 0);
~fpga.sendMsg("/slot/0/randPos", 0); ~fpga.sendMsg("/slot/0/randomPitch", 0);

// ---- sampler extras -------------------------------------------------------------------
~fpga.sendMsg("/slot/0/bank", 7);         // = program change
~fpga.sendMsg("/slot/1/envHold", 1);      // gate held open until note-off / gateOff
~fpga.sendMsg("/slot/1/trig");
~fpga.sendMsg("/slot/1/gateOff");
~fpga.sendMsg("/slot/0/envBars", 0.5);    // gate length in bars of the global tempo
~fpga.sendMsg("/slot/0/repeats", 3);      // stutter on the next trig
~fpga.sendMsg("/slot/0/interval", 0.08);
~fpga.sendMsg("/slot/1/tempoLock", 0);    // melody at its own speed (rate 1 = original)
~fpga.sendMsg("/slot/1/bgAdvance", 1);    // notes enter a sample that kept running in the background
~fpga.sendMsg("/transport/start");        // restart background playheads + beat counter together

// ---- algorithm per slot, switchable live (5 ms crossfade, both stay BPM locked) -------
~fpga.sendMsg("/slot/1/algo", 1);         // phase vocoder (Bungee): smooth stretch, rate/pitch as above
~fpga.sendMsg("/slot/1/algo", 0);         // back to the BufGrainPlay grains
~fpga.sendMsg("/slot/1/pvTransient", 0);  // phase vocoder without Bungee's transient handling
~fpga.sendMsg("/midi/cc", 10, 22, 127);   // CC22 >= 64: phase vocoder, < 64: grains (proposal)
~fpga.sendMsg("/slot/1/pos", 0.4); ~fpga.sendMsg("/slot/1/algo", 1); // pinned pos + PV = spectral freeze

// ---- a quick break-cutter: random 16th slices of the drum loop ------------------------
(
~rand=0.1;
~cut = Routine({
    var dur = 60 / 172 / 4;               // one 16th at 172 BPM
    loop {
		if(~rand>(1.0.rand),{
        var slice = 16.rand, rep = [1, 1, 1, 20.rand, 3].choose;
		~fpga.sendMsg("/midi/cc", 9, 1, 12.rand);
			~fpga.sendMsg("/midi/bend", 9, 16000.rand);
        ~fpga.sendMsg("/midi/cc", 9, 10, 60+[1, 3, 7].choose);
			~fpga.sendMsg("/midi/note", 9, 12 + slice, rep);}
		);
        dur.wait;
    }
}).play(TempoClock.default);
)
~cut.stop;

// ---- random loop switching inside the bank (instant: the bank is in SDRAM) ------------
(
~shuffle = Routine({ loop { ~fpga.sendMsg("/midi/cc", 9, 1, 128.rand); (60 / 172 * 4).wait } }).play;
)
~shuffle.stop;
