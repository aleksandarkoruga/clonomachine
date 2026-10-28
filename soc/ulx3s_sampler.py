#!/usr/bin/env python3
#
# ULX3S-85F time-stretching sampler SoC: VexRiscv + 32 MB SDRAM + SD card + sampler engine.
#
#   ./soc/ulx3s_sampler.py --build            # gateware + BIOS
#   ./soc/ulx3s_sampler.py --load             # program SRAM via openFPGALoader (volatile)
#   ./soc/ulx3s_sampler.py --flash            # program SPI flash (persistent)
#
# Clocks: 25 MHz -> PLL0: sys 45 MHz (+ sys_ps for the SDRAM clock)
#                -> PLL1: audio ~49.152 MHz (1024 cycles per 48 kHz sample). ECP5 integer
#                   dividers give 25/3*59/10 = 49.1667 MHz (+0.03 %, fs = 48.014 kHz); the
#                   exact value is exported to firmware as AUDIO_CLK_FREQ so tempo and
#                   pitch maths compensate.
#                -> PLL2: usb 48.077 MHz (US2 USB host, OHCI)

import argparse
import os
import sys

from migen import *

from litex.gen import *
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.build.io import DDROutput

from litex_boards.platforms import radiona_ulx3s

from litex.soc.cores.clock import ECP5PLL
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOIn, GPIOOut
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCCore
from litex.soc.interconnect.csr import CSRStorage, CSRField
from litex.soc.integration.builder import Builder

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
FW_FLASH_OFFSET = 0x400000
from sampler import Sampler

# On-board ESP32 with the ULX3S v3.1.6+ pinout (emard/ulx3s doc/constraints/prototype/
# ulx3s_v316.lpf). The LiteX platform has the v3.0.x pins: there EN = F1 and GPIO0 = L2, here
# GPIO0 = F1, EN = J5 and L2 is GPIO22. EN and GPIO0 stay inputs with pull-ups: the ESP32 boots
# its own firmware and the sampler never resets it (host/esp32/ has the pass-through for that).
_esp_io = [
    ("esp_serial", 0,
        Subsignal("tx", Pins("K3")),                        # FPGA TX -> ESP32 RX (wifi_rxd)
        Subsignal("rx", Pins("K4"), Misc("PULLMODE=UP")),   # ESP32 TX -> FPGA RX (wifi_txd)
        IOStandard("LVCMOS33"),
    ),
    ("esp_ctl", 0,
        Subsignal("en",    Pins("J5"), Misc("PULLMODE=UP")),
        Subsignal("gpio0", Pins("F1"), Misc("PULLMODE=UP")),
        IOStandard("LVCMOS33"),
    ),
]


class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, audio_clk_freq, with_usb=False):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_sys_ps = ClockDomain()
        self.cd_audio = ClockDomain()

        clk25 = platform.request("clk25")
        rst   = platform.request("rst")

        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(rst | self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)

        self.audio_pll = apll = ECP5PLL()
        self.comb += apll.reset.eq(rst | self.rst)
        apll.register_clkin(clk25, 25e6)
        apll.create_clkout(self.cd_audio, audio_clk_freq, margin=1e-3)

        if with_usb:
            # own PLL, sys and audio untouched. With the ECP5's >= 10 MHz phase detector 25 MHz
            # can't make exactly 48 MHz: 625 MHz / 13 = 48.077 MHz (+0.16 %, USB full speed
            # allows +-0.25 %)
            self.cd_usb = ClockDomain()
            self.usb_pll = upll = ECP5PLL()
            self.comb += upll.reset.eq(rst | self.rst)
            upll.register_clkin(clk25, 25e6)
            upll.create_clkout(self.cd_usb, 48e6, margin=2.5e-3)

        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), ClockSignal("sys_ps"))


class _UsbPull(LiteXModule):
    """The US2 D+/D- resistors' FPGA ends (ULX3S USB_FPGA_PULL_D+/-): driven low = pull-downs
    for host mode (reset), high = device pull-ups, or released."""
    def __init__(self, pads):
        self.ctrl = CSRStorage(fields=[
            CSRField("dp", size=1, reset=0, description="D+ resistor level"),
            CSRField("dn", size=1, reset=0, description="D- resistor level"),
            CSRField("dp_oe", size=1, reset=1, description="drive the D+ resistor (0: released)"),
            CSRField("dn_oe", size=1, reset=1, description="drive the D- resistor (0: released)"),
        ])
        f = self.ctrl.fields
        for i, (o, oe) in enumerate(((f.dp, f.dp_oe), (f.dn, f.dn_oe))):  # pins B12 (D+), C12 (D-)
            t = TSTriple()
            self.specials += t.get_tristate(pads[i])
            self.comb += [t.o.eq(o), t.oe.eq(oe)]


def pll_out_freq(pll, n=0):
    cfg = pll.compute_config()
    return cfg["clko{}_freq".format(n)], cfg


class SamplerSoC(SoCCore):
    def __init__(self, device="LFE5U-85F", revision="2.0", sys_clk_freq=45e6, audio_clk_freq=49.152e6,
                 sdram_module="MT48LC16M16", n_slots=2, with_usb_host=True, **kwargs):
        platform = radiona_ulx3s.Platform(device=device, revision=revision, toolchain="trellis")
        platform.add_extension(_esp_io)

        self.crg = _CRG(platform, sys_clk_freq, audio_clk_freq, with_usb=with_usb_host)

        kwargs.setdefault("integrated_rom_size", 0x10000)
        # console = shell + SLIP events from host/osc_bridge.py: 1 Mbaud, deep RX FIFO
        kwargs["uart_baudrate"] = 1_000_000
        kwargs["uart_fifo_depth"] = 256
        SoCCore.__init__(self, platform, sys_clk_freq, ident="ULX3S sampler", **kwargs)

        # SDRAM ------------------------------------------------------------------------------------
        self.sdrphy = GENSDRPHY(platform.request("sdram"), sys_clk_freq)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = getattr(litedram_modules, sdram_module)(sys_clk_freq, "1:1"),
            l2_cache_size = 8192,
        )

        # SPI flash: bitstream at 0, firmware boot image (length, crc32, data) at 4 MB --------------
        # 1x mode with plain 25-series reads: works with whichever 16 MB chip the board carries.
        from litespi.modules import IS25LP128
        from litespi.opcodes import SpiNorFlashOpCodes as Codes
        self.add_spi_flash(mode="1x", module=IS25LP128(Codes.READ_1_1_1), clk_freq=25e6)
        self.add_constant("FLASH_BOOT_ADDRESS", self.bus.regions["spiflash"].origin + FW_FLASH_OFFSET)

        # SD card ----------------------------------------------------------------------------------
        self.add_sdcard()

        # On-board ESP32 link: second UART, 3 Mbaud; EN / GPIO0 readable (inputs) -----------------
        self.add_uart(name="esp_uart", uart_name="esp_serial", baudrate=3_000_000, fifo_depth=256)
        esp_ctl = platform.request("esp_ctl")
        self.esp_ctl = GPIOIn(Cat(esp_ctl.en, esp_ctl.gpio0))

        # Sampler ----------------------------------------------------------------------------------
        port = self.sdram.crossbar.get_port(mode="read")
        self.sampler = Sampler(platform, port, platform.request("audio"), n_slots=n_slots)
        self.platform.add_period_constraint(self.crg.cd_audio.clk, 1e9 / audio_clk_freq)
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, self.crg.cd_audio.clk)

        f, cfg = pll_out_freq(self.crg.audio_pll)
        self.add_constant("AUDIO_CLK_FREQ", int(round(f)))
        self.add_constant("SAMPLER_N_SLOTS", n_slots)
        self.logger.info("Audio PLL: {:.4f} MHz -> fs = {:.3f} Hz ({})".format(f / 1e6, f / 1024, cfg))

        # USB host on US2 (OHCI): USB-MIDI devices such as the Elektron TM-1 -----------------------
        if with_usb_host:
            from litex.soc.cores.usb_ohci import USBOHCI
            usb_pads = platform.request("usb")
            class _OhciPads:
                dp = [usb_pads.d_p]
                dm = [usb_pads.d_n]
            self.usb_ohci = USBOHCI(platform, _OhciPads, usb_clk_freq=int(48e6))
            self.bus.add_slave("usb_ohci_ctrl", self.usb_ohci.wb_ctrl,
                region=SoCRegion(origin=0x90000000, size=0x1000, cached=False))
            self.bus.add_master("usb_ohci_dma", master=self.usb_ohci.wb_dma)
            self.usb_pull = _UsbPull(usb_pads.pullup)
            self.platform.add_period_constraint(self.crg.cd_usb.clk, 1e9 / 48e6)
            self.platform.add_false_path_constraints(self.crg.cd_sys.clk, self.crg.cd_usb.clk)

        # UI ---------------------------------------------------------------------------------------
        self.leds    = LedChaser(pads=platform.request_all("user_led"), sys_clk_freq=sys_clk_freq)
        # F1 (user_btn 1) is the SoC reset; the other six buttons are readable.
        self.buttons = GPIOIn(Cat(*[platform.request("user_btn", i) for i in (0, 2, 3, 4, 5, 6)]))


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=radiona_ulx3s.Platform, description="ULX3S sampler SoC")
    parser.add_target_argument("--device",       default="LFE5U-85F")
    parser.add_target_argument("--revision",     default="2.0")
    # 45 MHz: a LiteX bus path (SD DMA master -> shared decoder -> SPI-flash CS) needs
    # ~20-21 ns whatever the placement; 45 MHz clears it on every seed
    parser.add_target_argument("--sys-clk-freq", default=45e6, type=float)
    parser.add_target_argument("--sdram-module", default="MT48LC16M16",
        help="MT48LC16M16 / IS42S16160 (32 MB) or AS4C32M16 (64 MB, ULX3S v3.1.x)")
    parser.add_target_argument("--n-slots",      default=3, type=int)
    parser.add_target_argument("--flash",        action="store_true", help="Write the bitstream to SPI flash.")
    # seed 4 (audio 58.6 / sys 49.7 MHz at 45 MHz). After the engine pipelining every seed
    # of a sweep passed with >= 10 % margin, so the seed is no longer critical
    parser.set_defaults(cpu_type="vexriscv", cpu_variant="standard", output_dir="build/ulx3s_sampler", nextpnr_seed=4)
    args = parser.parse_args()

    soc = SamplerSoC(
        device       = args.device,
        revision     = args.revision,
        sys_clk_freq = args.sys_clk_freq,
        sdram_module = args.sdram_module,
        n_slots      = args.n_slots,
        **parser.soc_argdict,
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if getattr(args, "load", False):
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))
    if getattr(args, "flash", False):
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".bit"))


if __name__ == "__main__":
    main()
