# Verilator RTL-vs-model regression.  make && ./obj_dir/Vsampler_engine [scenario]
VERILATOR ?= verilator
RTL = ../rtl/sampler_engine.v
all: obj_dir/Vsampler_engine

obj_dir/Vsampler_engine: $(RTL) tb_engine.cpp ../model/sampler_model.cpp ../model/sampler_model.h ../include/sampler_regs.h
	$(VERILATOR) --cc --exe --build -O3 -j 8 --x-assign 0 --x-initial 0 \
	  -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDPARAM \
	  -GHANN_HEX='"../rtl/hann.hex"' -GEXP2_HEX='"../rtl/exp2.hex"' \
	  -CFLAGS "-std=c++20 -O2" $(RTL) tb_engine.cpp ../model/sampler_model.cpp

run: all
	./obj_dir/Vsampler_engine

clean:
	rm -rf obj_dir
.PHONY: all run clean
