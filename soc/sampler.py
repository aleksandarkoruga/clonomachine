#
# LiteX wrapper for rtl/sampler_engine.v + rtl/dsm_dac.v.
#
# sys domain  : CSRs, SDRAM line fetcher (LiteDRAM native read port)
# audio domain: engine (+ phase-vocoder unit) + delta-sigma DAC (1024 cycles per output sample)
#
# Clock-domain crossings: command FIFO (sys->audio), line-request FIFO (audio->sys),
# line-data FIFO (sys->audio), dual-clock status RAM (audio->sys), MultiReg for ctrl.

import os

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litedram.frontend.dma import LiteDRAMDMAReader

RTL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))


class LineFetcher(LiteXModule):
    """One request (word address) -> 64 consecutive 16-bit words on `source`."""
    def __init__(self, port, line_words=64):
        assert port.data_width == 16, "line fetcher expects a 16-bit native port (SDR x16)"
        self.sink   = sink   = stream.Endpoint([("addr", 25)])
        self.source = source = stream.Endpoint([("data", 16)])

        self.dma = dma = LiteDRAMDMAReader(port, fifo_depth=64, fifo_buffered=True)

        count = Signal(max=line_words)
        base  = Signal(25)
        busy  = Signal()
        self.comb += [
            dma.sink.address.eq(base + count),
            dma.sink.valid.eq(busy),
            sink.ready.eq(~busy),
            dma.source.connect(source, omit={"data"}),
            source.data.eq(dma.source.data),
        ]
        self.sync += [
            If(~busy & sink.valid,
                base.eq(sink.addr),
                count.eq(0),
                busy.eq(1),
            ).Elif(busy & dma.sink.ready,
                count.eq(count + 1),
                If(count == line_words - 1, busy.eq(0)),
            )
        ]


class Sampler(LiteXModule):
    def __init__(self, platform, dram_port, audio_pads, n_slots=2, cd_audio="audio"):
        slot_bits = max(1, (n_slots - 1).bit_length())

        # CSRs -------------------------------------------------------------------------------------
        self.cmd_data  = CSRStorage(32, description="Engine register value (write before CMD_ADDR).")
        self.cmd_addr  = CSRStorage(8,  description="Writing pushes {CMD_ADDR, CMD_DATA} to the engine.")
        self.cmd_ready = CSRStatus(1,   description="1 when the command FIFO can take a write.")
        self.stat_addr = CSRStorage(6,  description="Status word index (see sampler_regs.h).")
        self.stat_data = CSRStatus(32,  description="Status word at STAT_ADDR (1 cycle latency).")
        self.ctrl      = CSRStorage(fields=[
            CSRField("dsm_mode", size=1, description="0: 4-bit error-feedback, 1: 1-bit CIFB"),
            CSRField("reset",    size=1, description="Hold the audio engine in reset."),
        ])

        # Command FIFO sys -> audio ----------------------------------------------------------------
        cmd_fifo = stream.AsyncFIFO([("addr", 8), ("data", 32)], depth=64, buffered=False)
        cmd_fifo = ClockDomainsRenamer({"write": "sys", "read": cd_audio})(cmd_fifo)
        self.cmd_fifo = cmd_fifo
        self.comb += [
            cmd_fifo.sink.valid.eq(self.cmd_addr.re),
            cmd_fifo.sink.addr.eq(self.cmd_addr.storage),
            cmd_fifo.sink.data.eq(self.cmd_data.storage),
            self.cmd_ready.status.eq(cmd_fifo.sink.ready),
        ]

        # Line requests audio -> sys, data sys -> audio ---------------------------------------------
        req_fifo = stream.AsyncFIFO([("addr", 25)], depth=32, buffered=False)
        req_fifo = ClockDomainsRenamer({"write": cd_audio, "read": "sys"})(req_fifo)
        self.req_fifo = req_fifo
        fill_fifo = stream.AsyncFIFO([("data", 16)], depth=256, buffered=True)
        fill_fifo = ClockDomainsRenamer({"write": "sys", "read": cd_audio})(fill_fifo)
        self.fill_fifo = fill_fifo
        self.fetcher = fetcher = LineFetcher(dram_port)
        self.comb += [
            req_fifo.source.connect(fetcher.sink),
            fetcher.source.connect(fill_fifo.sink),
            fill_fifo.source.ready.eq(1),   # engine never stalls the fill stream
        ]

        # Status RAM audio -> sys ------------------------------------------------------------------
        stat_mem = Memory(32, 64)
        self.specials += stat_mem
        stat_wp = stat_mem.get_port(write_capable=True, clock_domain=cd_audio)
        stat_rp = stat_mem.get_port(clock_domain="sys")
        self.specials += stat_wp, stat_rp
        self.comb += [
            stat_rp.adr.eq(self.stat_addr.storage),
            self.stat_data.status.eq(stat_rp.dat_r),
        ]

        # Control (sys -> audio) -------------------------------------------------------------------
        dsm_mode  = Signal()
        eng_reset = Signal()
        self.specials += [
            MultiReg(self.ctrl.fields.dsm_mode, dsm_mode,  cd_audio),
            MultiReg(self.ctrl.fields.reset,    eng_reset, cd_audio),
        ]

        # Engine -----------------------------------------------------------------------------------
        out_stb    = Signal()
        out_sample = Signal(24)
        tick       = Signal()
        code       = Signal(4)
        rst        = Signal()
        self.comb += rst.eq(ResetSignal(cd_audio) | eng_reset)
        self.specials += Instance("sampler_engine",
            p_N_SLOTS   = n_slots,
            p_SLOT_BITS = slot_bits,
            p_HANN_HEX  = os.path.join(RTL_DIR, "hann.hex"),
            p_EXP2_HEX  = os.path.join(RTL_DIR, "exp2.hex"),
            p_PV_SIN_HEX  = os.path.join(RTL_DIR, "pv_sin.hex"),
            p_PV_ATAN_HEX = os.path.join(RTL_DIR, "pv_atan.hex"),
            p_PV_WIN_HEX  = os.path.join(RTL_DIR, "pv_win.hex"),
            i_clk        = ClockSignal(cd_audio),
            i_rst        = rst,
            i_cmd_valid  = cmd_fifo.source.valid,
            o_cmd_ready  = cmd_fifo.source.ready,
            i_cmd_addr   = cmd_fifo.source.addr,
            i_cmd_data   = cmd_fifo.source.data,
            o_req_valid  = req_fifo.sink.valid,
            i_req_ready  = req_fifo.sink.ready,
            o_req_addr   = req_fifo.sink.addr,
            i_fill_valid = fill_fifo.source.valid,
            i_fill_data  = fill_fifo.source.data,
            o_out_stb    = out_stb,
            o_out_sample = out_sample,
            o_tick_out   = tick,
            o_stat_we    = stat_wp.we,
            o_stat_addr  = stat_wp.adr,
            o_stat_data  = stat_wp.dat_w,
        )
        self.specials += Instance("dsm_dac",
            i_clk    = ClockSignal(cd_audio),
            i_rst    = rst,
            i_tick   = tick,
            i_sample = out_sample,
            i_mode   = dsm_mode,
            o_code   = code,
        )
        # Mono for now: same stream on both channels (OUT_BUS routing comes with the mixer).
        self.comb += [
            audio_pads.l.eq(code),
            audio_pads.r.eq(code),
        ]
        platform.add_source(os.path.join(RTL_DIR, "sampler_engine.v"))
        platform.add_source(os.path.join(RTL_DIR, "pv_unit.v"))
        platform.add_source(os.path.join(RTL_DIR, "dsm_dac.v"))
