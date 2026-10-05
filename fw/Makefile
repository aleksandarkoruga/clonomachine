# Sampler firmware (VexRiscv, LiteX libbase + liblitesdcard + vendored FatFs with write).
#   make            -> sampler.bin   (load: host/load_fw.sh, or copy to the SD card as boot.bin)
BUILD_DIR ?= ../build/ulx3s_sampler

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

OBJECTS = crt0.o main.o uart.o libc_extra.o samples.o transients.o wavutil.o slot.o mapping.o slip.o ff.o ffunicode.o
# LiteX's read-only libfatfs is replaced by fatfs/
FW_LIBS = $(filter-out libfatfs,$(LIBS))
CFLAGS += -I$(CURDIR) -Wno-missing-prototypes

all: sampler.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	@$(PYTHON) -c "import os;print('sampler.bin: %d bytes' % os.path.getsize('$@'))"

sampler.elf: $(OBJECTS) linker.ld
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ $(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--start-group $(FW_LIBS:lib%=-l%) -Wl,--end-group \
		-Wl,--gc-sections -Wl,-Map,$@.map

# hot loader / format conversion code: speed over size
samples.o transients.o wavutil.o: CFLAGS += -O2

-include $(OBJECTS:.o=.d)

VPATH = $(CURDIR):$(CURDIR)/fatfs:$(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)
%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) sampler.elf sampler.bin sampler.elf.map

.PHONY: all clean
