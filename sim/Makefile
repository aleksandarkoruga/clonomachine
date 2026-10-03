# Verilator RTL-vs-model regression.  make && ./obj_dir/Vsampler_engine [scenario]
VERILATOR ?= verilator
RTL = ../rtl/sampler_engine.v ../rtl/pv_unit.v
all: obj_dir/Vsampler_engine

obj_dir/Vsampler_engine: $(RTL) tb_engine.cpp ../model/sampler_model.cpp ../model/sampler_model.h ../model/pv_model.cpp ../model/pv_model.h ../include/sampler_regs.h
	$(VERILATOR) --cc --exe --build -O3 -j 8 --x-assign 0 --x-initial 0 --top-module sampler_engine \
	  -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDPARAM \
	  -GHANN_HEX='"../rtl/hann.hex"' -GEXP2_HEX='"../rtl/exp2.hex"' \
	  -GPV_SIN_HEX='"../rtl/pv_sin.hex"' -GPV_ATAN_HEX='"../rtl/pv_atan.hex"' -GPV_WIN_HEX='"../rtl/pv_win.hex"' \
	  -CFLAGS "-std=c++20 -O2" $(RTL) tb_engine.cpp ../model/sampler_model.cpp ../model/pv_model.cpp

run: all
	./obj_dir/Vsampler_engine

clean:
	rm -rf obj_dir
.PHONY: all run clean

# phase-vocoder unit vs model, job by job:  make pv && ./obj_pv/Vpv_unit [file.wav]
PV_RTL = ../rtl/pv_unit.v
obj_pv/Vpv_unit: $(PV_RTL) tb_pv.cpp ../model/pv_model.cpp ../model/pv_model.h ../include/sampler_regs.h
	$(VERILATOR) --cc --exe --build -O3 -j 8 --x-assign 0 --x-initial 0 --Mdir obj_pv \
	  -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDPARAM \
	  -GSIN_HEX='"../rtl/pv_sin.hex"' -GATAN_HEX='"../rtl/pv_atan.hex"' -GWIN_HEX='"../rtl/pv_win.hex"' \
	  --public-flat-rw -CFLAGS "-std=c++20 -O2" $(PV_RTL) tb_pv.cpp ../model/pv_model.cpp
pv: obj_pv/Vpv_unit
pvrun: pv
	./obj_pv/Vpv_unit
.PHONY: pv pvrun
