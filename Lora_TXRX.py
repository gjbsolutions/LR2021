"""
LR2021 LoRa TX/RX demo -- generic (sub-GHz LF path and 2.4 GHz HF path)
------------------------------------------------------------------------
TX side: sends "Hello World! <N>" every INTERVAL_MS milliseconds.
RX side: prints every received packet to the console.

Edit ROLE and FREQ_HZ below. The band is derived automatically from FREQ_HZ:
    150 MHz  .. 960 MHz   -> LF path, LF PA
    1500 MHz .. 2500 MHz  -> HF path, HF PA   (LR2021 / LR2022 only)
Both ends must use the same FREQ_HZ, SF, BW, CR, SYNCWORD.

Everything the script sends was checked against the LR2021 datasheet
Rev 2.2. References:
    Table 5-20  IRQ bits                    Table 6-28  Blocks_to_calibrate
    Table 6-29  CalibFE (HF path = bit 15)  Table 6-38  Stat definition
    Table 6-43  GetErrors bits              Table 7-2   SetRxPath
    Table 7-18  SetPaConfig                 Table 7-19  PA values, 915 MHz ref
    Table 7-20  PA values, 490 MHz ref      Table 7-21  PA values, 2445 MHz ref
    Table 7-23  SetTxParams                 Table 8-1   SetPacketType
    Table 9-1   SetLoraModulationParams     Table 9-5   SetLoraPacketParams
    Table 9-7   SetLoraSyncword             Table 9-13  GetLoraPacketStatus

Corrections vs. the previous version of this script:
  1. CALIB_AAF is bit 3 (Table 6-28: bit7 rfu, 6 PA_OFF, 5 MU, 4 rfu, 3 AAF,
     2 PLL, 1 HF_RC, 0 LF_RC). An earlier "fix" moved it to bit 4, which is
     reserved. CALIB_ALL is now 0x6F.
  2. SetPacketType(LoRa) is now actually sent. The function existed but was
     never called. The datasheet says it must be the first radio command.
  3. calib_fe() sets bit 15 of the frequency word for the HF path.
  4. SetRxPath selects LF/HF from the band and uses the recommended boost
     (0 for LF, 4 for HF).
  5. TX power uses the datasheet PA tables (LF: 7-19 / 7-20, HF: 7-21).
  6. LDRO is computed from SF/BW using the datasheet recommendation.
"""

from machine import SPI, Pin
import time

# ==============================================================================
# USER CONFIG
# ==============================================================================
ROLE         = 'TX'          # 'TX' or 'RX'
FREQ_HZ      = 868_000_000   # e.g. 868_000_000 (LF) or 2_440_000_000 (HF)
SF           = 0x7           # 5..12  (0x5..0xC)
BW           = 0x4           # see BW_KHZ below; 0x4 = 125 kHz, 0xF = 812 kHz
CR           = 0x1           # 1..4 = 4/5..4/8
LDRO         = None          # None = automatic; or force 0 / 1
SYNCWORD     = 0x12          # 0x12 = private, 0x34 = public LoRaWAN
TX_POWER_DBM = 14            # LF: 10..22 dBm, HF: 0..12 dBm use datasheet tables
INTERVAL_MS  = 2000          # TX interval
RX_BOOST     = None          # None = automatic (0 for LF, 4 for HF); or 0..7
USE_TCXO     = False         # True if your board has a TCXO on XTA
TCXO_VOLTAGE = 0x02          # 1.8 V (only used if USE_TCXO=True)
USE_DCDC     = True          # True = SIMO DC-DC, False = LDO
IRQ_DIO      = 9             # DIO number wired to the IRQ pin below (5..11)
TX_WAIT_MS   = 10_000        # software guard while waiting for TxDone

# LoRa bandwidth codes (Table 9-3) -> actual kHz (used for LDRO decision)
BW_KHZ = {
    0x2: 31.25, 0x3: 62.5,  0x4: 125.0, 0x5: 250.0, 0x6: 500.0, 0x7: 1000.0,
    0xA: 41.67, 0xB: 83.34, 0xC: 101.5625,
    0xD: 203.125, 0xE: 406.25, 0xF: 812.5,
}

# ==============================================================================
# BAND SELECTION
# ==============================================================================
def band_is_hf(freq_hz):
    """Return True for the HF (1.5-2.5 GHz) path, False for the LF path."""
    if 150_000_000 <= freq_hz <= 960_000_000:
        return False
    if 1_500_000_000 <= freq_hz <= 2_500_000_000:
        return True
    raise ValueError("FREQ_HZ must be 150-960 MHz (LF) or 1500-2500 MHz (HF)")

USE_HF = band_is_hf(FREQ_HZ)

# ==============================================================================
# HARDWARE
# ==============================================================================
spi  = SPI(1, baudrate=1_000_000, polarity=0, phase=0, bits=8,
           firstbit=SPI.MSB, sck=Pin(4), mosi=Pin(6), miso=Pin(5))
CS   = Pin(7, Pin.OUT)
IRQ  = Pin(10, Pin.IN)     # DIO9
NRST = Pin(2, Pin.OUT)
BUSY = Pin(3, Pin.IN)

# ==============================================================================
# IRQ BIT MASKS  (Table 5-20)
# ==============================================================================
IRQ_RX_FIFO          = (1 << 0)
IRQ_TX_FIFO          = (1 << 1)
IRQ_PREAMBLE         = (1 << 5)
IRQ_HEADER_VALID     = (1 << 6)
IRQ_CAD_DETECTED     = (1 << 7)
IRQ_HEADER_ERR       = (1 << 9)
IRQ_LOW_BATTERY      = (1 << 10)
IRQ_PA_OCP_OVP       = (1 << 11)
IRQ_ERROR            = (1 << 16)
IRQ_CMD_ERROR        = (1 << 17)
IRQ_RX_DONE          = (1 << 18)
IRQ_TX_DONE          = (1 << 19)
IRQ_CAD_DONE         = (1 << 20)
IRQ_TIMEOUT          = (1 << 21)
IRQ_CRC_ERROR        = (1 << 22)
IRQ_LEN_ERROR        = (1 << 23)
IRQ_ADDR_ERROR       = (1 << 24)

# ==============================================================================
# CALIBRATION MASKS  (Table 6-28)
#   bit7 rfu | bit6 PA_OFF | bit5 MU | bit4 rfu | bit3 AAF | bit2 PLL |
#   bit1 HF_RC | bit0 LF_RC
# ==============================================================================
CALIB_LF_RC  = (1 << 0)
CALIB_HF_RC  = (1 << 1)
CALIB_PLL    = (1 << 2)
CALIB_AAF    = (1 << 3)
CALIB_MU     = (1 << 5)
CALIB_PA_OFF = (1 << 6)
CALIB_ALL    = CALIB_LF_RC | CALIB_HF_RC | CALIB_PLL | CALIB_AAF | CALIB_MU | CALIB_PA_OFF  # 0x6F

# ==============================================================================
# SPI HELPERS
# ==============================================================================
def wait_busy():
    while BUSY.value() == 1:
        pass

def _u32_bytes(v):
    return [(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]

def _cmd_write(oh, ol, args=()):
    wait_busy()
    CS.value(0)
    spi.write(bytes([oh, ol] + list(args)))
    CS.value(1)

def _cmd_write_capture(oh, ol, args=(), n=6):
    wait_busy()
    CS.value(0)
    tx = bytes([oh, ol] + list(args))
    rx = bytearray(max(len(tx), n))
    spi.write_readinto(tx, rx)
    CS.value(1)
    return rx

def _cmd_read(oh, ol, args, n):
    # Read command = two SPI frames (Fig 5-3). Response frame starts with
    # Stat(15:8), Stat(7:0), then the data bytes.
    wait_busy()
    CS.value(0)
    spi.write(bytes([oh, ol] + list(args)))
    CS.value(1)
    wait_busy()
    CS.value(0)
    rx = bytearray(n)
    spi.readinto(rx, 0x00)
    CS.value(1)
    return rx

def _fifo_read(length):
    # Direct read (Fig 5-4): opcode 0x0001, then Stat(2 bytes) + data.
    if length == 0:
        return b''
    wait_busy()
    CS.value(0)
    tx = bytes([0x00, 0x01] + [0x00] * length)
    rx = bytearray(2 + length)
    spi.write_readinto(tx, rx)
    CS.value(1)
    return bytes(rx[2:])

# ==============================================================================
# STATUS / DIAGNOSTICS
# ==============================================================================
def decode_stat(hi, lo):
    # Table 6-38: (15:12)=0 | CommandStatus(11:9) | IntStatus(8) |
    #             ResetSource(7:4) | rfu(3) | ChipMode(2:0)
    cs = (hi >> 1) & 0x07
    cs_name = {0: "CMD_FAIL", 1: "CMD_PERR", 2: "CMD_OK", 3: "CMD_DAT"}.get(cs, "?")
    irq = bool(hi & 0x01)
    rs  = (lo >> 4) & 0x0F
    cm  = lo & 0x07
    rs_name = {0: "Cleared", 1: "POR/BRN", 2: "NRESET"}.get(rs, f"RFU({rs})")
    cm_name = {0: "SLEEP", 1: "STDBY_RC", 2: "STDBY_XOSC",
               3: "FS", 4: "Rx", 5: "Tx"}.get(cm, f"RFU({cm})")
    return cs_name, irq, rs_name, cm_name

def decode_errors(e16):
    # Table 6-43
    names = {
        0: "HF_XOSC_START", 1: "LF_XOSC_START", 2: "PLL_LOCK",
        3: "LF_RC_CALIB", 4: "HF_RC_CALIB", 5: "PLL_CALIB",
        6: "AAF_CALIB", 7: "IMG_CALIB", 8: "CHIP_BUSY",
        9: "RXFREQ_NO_FE_CAL", 10: "MEAS_UNIT_ADC_CALIB",
        11: "PA_OFFSET_CALIB",
    }
    return [n for b, n in names.items() if e16 & (1 << b)] or ["No errors"]

def get_status():
    r = _cmd_write_capture(0x01, 0x00, [], 6)
    cs, irq, rs, cm = decode_stat(r[0], r[1])
    irq32 = (r[2] << 24) | (r[3] << 16) | (r[4] << 8) | r[5]
    return cs, irq, rs, cm, irq32

def get_errors():
    r = _cmd_read(0x01, 0x10, [], 4)
    return decode_errors((r[2] << 8) | r[3])

def clear_errors():
    _cmd_write(0x01, 0x11)

def get_and_clear_irq_status():
    r = _cmd_read(0x01, 0x17, [], 6)
    return (r[2] << 24) | (r[3] << 16) | (r[4] << 8) | r[5]

def clear_irq(mask=0xFFFFFFFF):
    _cmd_write(0x01, 0x16, _u32_bytes(mask))

# ==============================================================================
# MODE / CLOCK / CALIBRATION
# ==============================================================================
def set_standby(mode=0x00):
    # 0 = Standby RC, 1 = Standby XOSC (Table 6-8)
    _cmd_write(0x01, 0x28, [mode])

def set_rx(timeout=0x000000):
    # 0x000000 = single Rx until reception; 0xFFFFFF = continuous (Table 6-11)
    _cmd_write(0x02, 0x0C, [(timeout >> 16) & 0xFF,
                            (timeout >> 8) & 0xFF,
                             timeout & 0xFF])

def set_tx(timeout=0x000000):
    # 0x000000 = no timeout (Table 6-12)
    _cmd_write(0x02, 0x0D, [(timeout >> 16) & 0xFF,
                            (timeout >> 8) & 0xFF,
                             timeout & 0xFF])

def set_tcxo(voltage=0x02, start=0x0007A120):
    # Table 6-64. Standby RC only.
    _cmd_write(0x01, 0x20, [voltage & 0x07,
                            (start >> 24) & 0xFF,
                            (start >> 16) & 0xFF,
                            (start >>  8) & 0xFF,
                             start        & 0xFF])

def set_reg_mode(simo=0x02):
    # 0x00 = LDO only, 0x02 = SIMO normal (Table 6-26). Standby RC only.
    _cmd_write(0x01, 0x21, [simo & 0xFF])

def calibrate(blocks=CALIB_ALL):
    _cmd_write(0x01, 0x22, [blocks & 0xFF])
    wait_busy()

def calib_fe(f1=None, f2=None, f3=None, hf=False):
    """
    Front-end (image / ADC offset) calibration, Table 6-29.
    Each frequency is a 15-bit value in 4 MHz steps; bit 15 selects the
    path: 0 = LF, 1 = HF (900 MHz -> 0x00E1 LF / 0x80E1 HF).
    Not allowed in Rx or Tx mode.
    """
    def enc(f):
        if f is None:
            return [0x00, 0x00]
        c = (f // 4_000_000) & 0x7FFF
        if hf:
            c |= 0x8000
        return [(c >> 8) & 0xFF, c & 0xFF]
    _cmd_write(0x01, 0x23, enc(f1) + enc(f2) + enc(f3))
    wait_busy()

# ==============================================================================
# RF / DIO / PA
# ==============================================================================
def set_rf_frequency(hz):
    _cmd_write(0x02, 0x00, _u32_bytes(hz))

def set_rx_path(hf=False, boost=None):
    # Table 7-2: byte2 = rx_path (bit0): 0 = LF, 1 = HF; byte3 = rx_boost(2:0)
    # Recommended boost: 0 for LF, 4 for HF.
    if boost is None:
        boost = 4 if hf else 0
    _cmd_write(0x02, 0x01, [0x01 if hf else 0x00, boost & 0x07])

def set_dio_function(dio, func, pull=0x00):
    # Table 6-44: byte3 = Func(3:0) | pull_drive(3:0). DIO allowed 5..11.
    _cmd_write(0x01, 0x12, [dio & 0xFF, ((func & 0x0F) << 4) | (pull & 0x0F)])

def set_dio_irq_config(dio, mask):
    _cmd_write(0x01, 0x15, [dio & 0xFF] + _u32_bytes(mask))

def configure_irq_pin(dio=9, mask=0):
    set_dio_function(dio, func=0x1, pull=0x0)   # 0x1 = DIO_FUNCTION_IRQ
    set_dio_irq_config(dio, mask)

def set_packet_type_lora():
    # Table 8-1: 0x0 = LoRa. Allowed in Standby RC / Standby XOSC / FS.
    _cmd_write(0x02, 0x07, [0x00])

def set_pa_config(pa_sel=0, pa_lf_mode=0, duty=4, slices=4, hf_duty=16):
    # Table 7-18:
    #   byte2 = pa_sel(bit7) | rfu(bits6:2) | pa_lf_mode(bits1:0)
    #   byte3 = pa_lf_duty_cycle(7:4) | pa_lf_slices(3:0)
    #   byte4 = rfu(7:5) | pa_hf_duty_cycle(4:0)
    # LF PA unused: duty = 6, slices = 7.  HF PA unused: hf_duty = 16.
    # hf_duty must be 16..31.
    b2 = ((pa_sel & 0x01) << 7) | (pa_lf_mode & 0x03)
    b3 = ((duty & 0x0F) << 4) | (slices & 0x0F)
    b4 = hf_duty & 0x1F
    _cmd_write(0x02, 0x02, [b2, b3, b4])

def set_tx_params(raw, ramp=0x04):
    # Table 7-23: tx_power is signed, +0.5 dB per LSB. ramp 0x04 = 32 us.
    if raw < 0:
        raw = 0x100 + raw
    _cmd_write(0x02, 0x03, [raw & 0xFF, ramp & 0xFF])

# ------------------------------------------------------------------------------
# PA optimal values, Semtech reference designs.
#   LF tables: target dBm : (TX_POWER, PA_LF_DUTY_CYCLE, PA_LF_SLICES)
#   HF table : target dBm : (TX_POWER, PA_HF_DUTY_CYCLE)
# ------------------------------------------------------------------------------
PA_TABLE_915 = {   # Table 7-19 (LF, used for >= 800 MHz, incl. 868 MHz)
    22.0:(22,7,7), 21.5:(22,6,7), 21.0:(22,5,6), 20.5:(21,6,7),
    20.0:(21,5,6), 19.5:(21,4,7), 19.0:(20,5,7), 18.5:(20,4,7),
    18.0:(19,5,7), 17.5:(19,5,4), 17.0:(18,7,3), 16.5:(18,5,4),
    16.0:(17,7,3), 15.5:(18,4,3), 15.0:(17,4,5), 14.5:(17,4,3),
    14.0:(17,4,2), 13.5:(16,4,3), 13.0:(16,4,2), 12.5:(15,5,2),
    12.0:(15,4,2), 11.5:(14,5,2), 11.0:(15,2,4), 10.5:(17,4,0),
    10.0:(16,1,2),
}
PA_TABLE_490 = {   # Table 7-20 (LF, used for < 800 MHz)
    21.0:(22,7,7), 20.5:(22,7,4), 20.0:(21,7,7), 19.5:(21,7,4),
    19.0:(20,7,6), 18.5:(22,6,2), 18.0:(19,7,6), 17.5:(19,7,3),
    17.0:(19,6,5), 16.5:(18,6,7), 16.0:(18,6,5), 15.5:(17,6,7),
    15.0:(16,7,5), 14.5:(16,6,7), 14.0:(15,7,5), 13.5:(15,7,3),
    13.0:(15,7,2), 12.5:(15,6,3), 12.0:(15,6,2), 11.5:(15,5,3),
    11.0:(15,4,5), 10.5:(17,6,0), 10.0:(15,5,1),
}
PA_TABLE_2445 = {  # Table 7-21 (HF PA, 2445 MHz reference design)
    12.0:(12,16), 11.5:(12,21), 11.0:(12,27), 10.5:(11,21), 10.0:(12,30),
     9.5:(11,29),  9.0:(11,30),  8.5:(10,29),  8.0:(10,30),  7.5:(9,29),
     7.0:(8,27),   6.5:(8,29),   6.0:(7,27),   5.5:(8,31),   5.0:(6,27),
     4.5:(7,31),   4.0:(5,27),   3.5:(6,31),   3.0:(4,27),   2.5:(5,31),
     2.0:(3,27),   1.5:(4,31),   1.0:(2,28),   0.5:(1,24),   0.0:(1,28),
}

def set_tx_power(dbm, ramp=0x04, freq_hz=None):
    """
    Program the PA for the requested output power on the band that matches
    freq_hz. Returns (effective_dbm, characterised).
    `characterised` is False when the request is below the lowest datasheet
    table entry; the output is then only an estimate (0.5 dB/LSB steps from
    the lowest table entry) and should be measured.
    Requests above the maximum are clamped to the maximum table entry.
    """
    if freq_hz is None:
        freq_hz = FREQ_HZ

    if band_is_hf(freq_hz):
        lo, hi = min(PA_TABLE_2445), max(PA_TABLE_2445)
        if dbm >= lo:
            target = min(PA_TABLE_2445, key=lambda x: abs(x - min(dbm, hi)))
            raw, hf_duty = PA_TABLE_2445[target]
            set_pa_config(pa_sel=1, pa_lf_mode=0, duty=6, slices=7,
                          hf_duty=hf_duty)
            set_tx_params(raw, ramp)
            return target, True
        base_raw, hf_duty = PA_TABLE_2445[lo]
        raw = max(-39, min(24, base_raw + round((dbm - lo) * 2)))  # PA_HF range
        set_pa_config(pa_sel=1, pa_lf_mode=0, duty=6, slices=7,
                      hf_duty=hf_duty)
        set_tx_params(raw, ramp)
        return dbm, False

    table = PA_TABLE_490 if freq_hz < 800_000_000 else PA_TABLE_915
    lo, hi = min(table), max(table)
    if dbm >= lo:
        target = min(table, key=lambda x: abs(x - min(dbm, hi)))
        raw, duty, slices = table[target]
        set_pa_config(pa_sel=0, pa_lf_mode=0, duty=duty, slices=slices)
        set_tx_params(raw, ramp)
        return target, True
    base_raw, duty, slices = table[lo]
    raw = max(-19, min(44, base_raw + round((dbm - lo) * 2)))      # PA_LF range
    set_pa_config(pa_sel=0, pa_lf_mode=0, duty=duty, slices=slices)
    set_tx_params(raw, ramp)
    return dbm, False

# ==============================================================================
# LoRa MODEM
# ==============================================================================
def auto_ldro(sf, bw):
    # Datasheet recommendation (Section 9.9.1): LDRO on for SF11 if
    # BW <= 125 kHz, and for SF12 if BW <= 250 kHz; off otherwise.
    khz = BW_KHZ.get(bw, 125.0)
    if sf == 0xB and khz <= 125.0:
        return 1
    if sf == 0xC and khz <= 250.0:
        return 1
    return 0

def set_lora_modulation(sf=0x7, bw=0x4, cr=0x1, ldro=None):
    # Table 9-1: byte2 = sf(7:4)|bw(3:0), byte3 = cr(7:4)|rfu(3:2)|ldro(1:0)
    if ldro is None:
        ldro = auto_ldro(sf, bw)
    _cmd_write(0x02, 0x20, [((sf & 0x0F) << 4) | (bw & 0x0F),
                            ((cr & 0x0F) << 4) | (ldro & 0x03)])

def set_lora_packet_params(pbl=8, pld=0xFF, hdr=0, crc=1, inv=0):
    # Table 9-5: byte5 = rfu(7:3)|header_type(2)|crc(1)|invert_iq(0)
    # hdr: 0 = explicit, 1 = implicit. pld = max payload (0 = accept any in
    # explicit mode).
    b5 = ((hdr & 1) << 2) | ((crc & 1) << 1) | (inv & 1)
    _cmd_write(0x02, 0x21, [(pbl >> 8) & 0xFF, pbl & 0xFF, pld & 0xFF, b5])

def set_lora_syncword(sw=0x12):
    # Table 9-7: only 0x12 (private) and 0x34 (public) are valid.
    _cmd_write(0x02, 0x23, [sw & 0xFF])

def write_tx_fifo(data):
    # Direct write, opcode 0x0002 (Fig 5-5).
    if len(data) > 256:
        raise ValueError("Payload > 256 bytes")
    _cmd_write(0x00, 0x02, list(data))

def read_rx_fifo(n):
    return _fifo_read(n)

def get_rx_pkt_length():
    r = _cmd_read(0x02, 0x12, [], 4)
    return (r[2] << 8) | r[3]

def clear_rx_fifo():
    _cmd_write(0x01, 0x1E)

def clear_tx_fifo():
    _cmd_write(0x01, 0x1F)

def get_lora_packet_status():
    # Table 9-13 (bytes after the 2 Stat bytes):
    #   r[2] = rfu(7:5)|crc(4)|coding_rate(3:0)   r[3] = pkt_length
    #   r[4] = snr_pkt (x4, two's complement)
    #   r[5] = rssi_pkt(8:1)   r[6] = rssi_signal_pkt(8:1)
    #   r[7] = rfu(7:6)|detector(5:2)|rssi_pkt(0)(bit1)|rssi_signal_pkt(0)(bit0)
    #   r[8..10] = freq_offset (signed 24-bit, Hz)
    r = _cmd_read(0x02, 0x2A, [], 11)
    crc_on = (r[2] >> 4) & 1
    pkt_len = r[3]
    snr_raw = r[4]
    snr = (snr_raw - 256 if snr_raw & 0x80 else snr_raw) / 4.0
    rssi9 = (r[5] << 1) | ((r[7] >> 1) & 1)
    rssi = -rssi9 / 2.0
    sig9  = (r[6] << 1) | (r[7] & 1)
    sig   = -sig9 / 2.0
    fo = (r[8] << 16) | (r[9] << 8) | r[10]
    if fo & 0x800000:
        fo -= 0x1000000
    return {"rssi": rssi, "snr": snr, "sig": sig,
            "len": pkt_len, "crc": crc_on, "foff": fo}

# ==============================================================================
# INIT
# ==============================================================================
def hw_reset():
    NRST.value(0)
    time.sleep_ms(1)
    NRST.value(1)
    time.sleep_ms(10)
    wait_busy()

def get_version():
    r = _cmd_read(0x01, 0x01, [], 4)
    return r[2], r[3]

def lora_init(freq_hz=FREQ_HZ, use_tcxo=USE_TCXO,
              tcxo_v=TCXO_VOLTAGE, use_dcdc=USE_DCDC):
    hf = band_is_hf(freq_hz)
    print(f"[INIT] Band: {'HF (2.4 GHz path)' if hf else 'LF (sub-GHz path)'}")
    print("[INIT] Resetting...")
    hw_reset()

    set_standby(0x00)                 # Standby RC (needed for TCXO / RegMode)
    clear_errors()
    major, minor = get_version()
    print(f"[INIT] Chip FW v{major}.{minor:02d}")

    if use_tcxo:
        print(f"[INIT] Configuring TCXO (V={tcxo_v:#x})")
        set_tcxo(voltage=tcxo_v)

    set_reg_mode(0x02 if use_dcdc else 0x00)
    print(f"[INIT] Power mode: {'SIMO DC-DC' if use_dcdc else 'LDO'}")

    set_packet_type_lora()            # must precede modem configuration

    set_standby(0x01)                 # XOSC running
    time.sleep_ms(10)

    print("[INIT] Calibrating all blocks...")
    calibrate(CALIB_ALL)              # leaves chip in Standby RC
    time.sleep_ms(5)

    print(f"[INIT] Front-end cal @ {freq_hz/1e6:.3f} MHz "
          f"({'HF' if hf else 'LF'} path)...")
    calib_fe(f1=freq_hz, hf=hf)
    time.sleep_ms(5)

    set_standby(0x01)
    set_rf_frequency(freq_hz)

    clear_irq()
    clear_errors()

    errs = get_errors()
    if errs != ["No errors"]:
        print(f"[INIT] Post-init errors: {errs}")

    print(f"[INIT] Ready @ {freq_hz/1e6:.3f} MHz "
          f"({'TCXO' if use_tcxo else 'XTAL'})")

# ==============================================================================
# TX ONE PACKET (blocking)
# ==============================================================================
def lora_tx_once(payload, irq_dio=IRQ_DIO):
    set_packet_type_lora()
    set_lora_modulation(sf=SF, bw=BW, cr=CR, ldro=LDRO)
    set_lora_packet_params(pbl=8, pld=len(payload), hdr=0, crc=1, inv=0)
    set_lora_syncword(SYNCWORD)
    configure_irq_pin(dio=irq_dio, mask=IRQ_TX_DONE | IRQ_TIMEOUT)

    clear_tx_fifo()
    write_tx_fifo(payload)
    clear_irq()
    set_tx(0x000000)
    wait_busy()

    t0 = time.ticks_ms()
    while IRQ.value() == 0:
        if time.ticks_diff(time.ticks_ms(), t0) > TX_WAIT_MS:
            print("[TX] No TxDone within guard time")
            return False
        time.sleep_ms(1)

    irq32 = get_and_clear_irq_status()
    if irq32 & IRQ_TX_DONE:
        return True
    if irq32 & IRQ_TIMEOUT:
        print("[TX] Timeout!")
        return False
    print(f"[TX] Unexpected IRQ 0x{irq32:08X}")
    return False

# ==============================================================================
# RX ONE PACKET (blocking)
# ==============================================================================
def lora_rx_once(irq_dio=IRQ_DIO, timeout=0x000000):
    set_packet_type_lora()
    set_lora_modulation(sf=SF, bw=BW, cr=CR, ldro=LDRO)
    set_lora_packet_params(pbl=8, pld=0, hdr=0, crc=1, inv=0)
    set_lora_syncword(SYNCWORD)
    set_rx_path(hf=USE_HF, boost=RX_BOOST)
    configure_irq_pin(dio=irq_dio,
                      mask=IRQ_RX_DONE | IRQ_TIMEOUT |
                           IRQ_CRC_ERROR | IRQ_LEN_ERROR | IRQ_ADDR_ERROR)

    clear_rx_fifo()
    clear_irq()
    set_rx(timeout)
    wait_busy()

    while IRQ.value() == 0:
        time.sleep_ms(1)

    irq32 = get_and_clear_irq_status()

    if irq32 & IRQ_TIMEOUT:
        return None, None, "timeout"
    if irq32 & IRQ_CRC_ERROR:
        return None, None, "crc"
    if irq32 & IRQ_LEN_ERROR:
        return None, None, "len"
    if irq32 & IRQ_ADDR_ERROR:
        return None, None, "addr"

    if irq32 & IRQ_RX_DONE:
        n = get_rx_pkt_length()
        data = read_rx_fifo(n)
        stats = get_lora_packet_status()
        clear_rx_fifo()
        return data, stats, "ok"

    return None, None, f"irq=0x{irq32:08X}"

# ==============================================================================
# MAIN LOOPS
# ==============================================================================
def run_tx():
    print("[TX] Starting transmitter loop")
    # PA settings are static, so configure once before the loop.
    eff_dbm, characterised = set_tx_power(TX_POWER_DBM)
    if eff_dbm != TX_POWER_DBM and characterised:
        print(f"[TX] Requested {TX_POWER_DBM} dBm -> using nearest table "
              f"entry {eff_dbm} dBm")
    elif characterised:
        print(f"[TX] PA set for {eff_dbm} dBm (datasheet table values)")
    else:
        print(f"[TX] WARNING: {TX_POWER_DBM} dBm is below the datasheet "
              f"tables. PA setting is an estimate; measure real output.")

    counter = 0
    while True:
        msg = f"Hello World! {counter}".encode()
        print(f"[TX] Sending #{counter}: {msg!r}")
        ok = lora_tx_once(msg)
        if not ok:
            print(f"[TX] Send failed, errors: {get_errors()}")
            clear_errors()
        counter += 1
        time.sleep_ms(INTERVAL_MS)


def run_rx():
    print("[RX] Starting receiver loop")
    while True:
        data, stats, status = lora_rx_once()
        if status == "ok" and data is not None:
            try:
                text = data.decode('utf-8')
            except UnicodeDecodeError:
                text = repr(data)
            print(f"[RX] {text}   "
                  f"(RSSI={stats['rssi']:.1f} dBm, "
                  f"SNR={stats['snr']:.1f} dB, "
                  f"foff={stats['foff']} Hz, "
                  f"len={stats['len']})")
        else:
            if status != "timeout":
                print(f"[RX] No packet ({status})")


# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    lora_init()
    if ROLE == 'TX':
        run_tx()
    else:
        run_rx()
