"""
LR2021 LoRa TX/RX demo
-----------------------
TX side: sends "Hello World! <N>" every INTERVAL_MS milliseconds.
RX side: prints every received packet to the console.

Edit ROLE below to switch between 'TX' and 'RX'.

Changes in this version:
  1. TX power now uses the datasheet's optimal PA tables
     (Table 7-19 for 915 MHz reference design, used for >= 800 MHz, and
     Table 7-20 for 490 MHz reference design, used for < 800 MHz).
     Each table row gives a matched (tx_power, pa_lf_duty_cycle,
     pa_lf_slices) triple. All three must be applied together; tx_power
     alone does NOT give the targeted output power.
  2. Below 10 dBm (lowest table entry) there is no characterised
     combination in the datasheet. The code keeps the 10 dBm duty/slices
     and steps tx_power down by 0.5 dB/LSB (Table 7-23). The resulting
     output is an estimate -- measure it if the exact level matters.
  3. Earlier fixes retained: CALIB_AAF = bit 4 (Table 6-28), and
     tx_power register range clamp per Table 7-23.
"""

from machine import SPI, Pin
import time

# ==============================================================================
# USER CONFIG
# ==============================================================================
ROLE         = 'TX'          # 'TX' or 'RX'
FREQ_HZ      = 868_000_000   # must match on both ends
SF           = 0x7           # SF7
BW           = 0x4           # 125 kHz
CR           = 0x1           # 4/5
LDRO         = 0x0           # 0 for SF7/BW125
SYNCWORD     = 0x12          # 0x12 = private, 0x34 = public LoRaWAN
TX_POWER_DBM = 14            # 10..22 dBm uses datasheet tables; <10 is estimated
INTERVAL_MS  = 2000          # TX interval
USE_TCXO     = False         # True if your board has a TCXO on XTA
TCXO_VOLTAGE = 0x02          # 1.8 V (only used if USE_TCXO=True)
USE_DCDC     = True          # True = SIMO DC-DC, False = LDO

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
# ==============================================================================
CALIB_LF_RC  = (1 << 0)
CALIB_HF_RC  = (1 << 1)
CALIB_PLL    = (1 << 2)
# bit 3 is RFU. AAF is bit 4.
CALIB_AAF    = (1 << 4)
CALIB_MU     = (1 << 5)
CALIB_PA_OFF = (1 << 6)
CALIB_ALL    = CALIB_LF_RC | CALIB_HF_RC | CALIB_PLL | CALIB_AAF | CALIB_MU | CALIB_PA_OFF

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
    _cmd_write(0x01, 0x28, [mode])

def set_rx(timeout=0x000000):
    _cmd_write(0x02, 0x0C, [(timeout >> 16) & 0xFF,
                            (timeout >> 8) & 0xFF,
                             timeout & 0xFF])

def set_tx(timeout=0x000000):
    _cmd_write(0x02, 0x0D, [(timeout >> 16) & 0xFF,
                            (timeout >> 8) & 0xFF,
                             timeout & 0xFF])

def set_tcxo(voltage=0x02, start=0x0007A120):
    _cmd_write(0x01, 0x20, [voltage & 0x07,
                            (start >> 24) & 0xFF,
                            (start >> 16) & 0xFF,
                            (start >>  8) & 0xFF,
                             start        & 0xFF])

def set_reg_mode(simo=0x02):
    _cmd_write(0x01, 0x21, [simo & 0xFF])

def calibrate(blocks=CALIB_ALL):
    _cmd_write(0x01, 0x22, [blocks & 0xFF])
    wait_busy()

def calib_fe(f1=None, f2=None, f3=None):
    def enc(f):
        if f is None:
            return [0x00, 0x00]
        c = (f // 4_000_000) & 0x7FFF
        return [(c >> 8) & 0xFF, c & 0xFF]
    _cmd_write(0x01, 0x23, enc(f1) + enc(f2) + enc(f3))
    wait_busy()

# ==============================================================================
# RF / DIO / PA
# ==============================================================================
def set_rf_frequency(hz):
    _cmd_write(0x02, 0x00, _u32_bytes(hz))

def set_rx_path(lf=True, boost=0):
    _cmd_write(0x02, 0x01, [0x00 if lf else 0x01, boost & 0x07])

def set_dio_function(dio, func, pull=0x00):
    _cmd_write(0x01, 0x12, [dio & 0xFF, ((func & 0x0F) << 4) | (pull & 0x0F)])

def set_dio_irq_config(dio, mask):
    _cmd_write(0x01, 0x15, [dio & 0xFF] + _u32_bytes(mask))

def configure_irq_pin(dio=9, mask=0):
    set_dio_function(dio, func=0x1, pull=0x0)
    set_dio_irq_config(dio, mask)

def set_packet_type_lora():
    _cmd_write(0x02, 0x07, [0x00])

def set_pa_config(pa_sel=0, pa_lf_mode=0, duty=4, slices=4, hf_duty=16):
    # Table 7-18: byte2 = pa_sel(bit7) | rfu(bits6:2) | pa_lf_mode(bits1:0)
    b2 = ((pa_sel & 0x01) << 7) | (pa_lf_mode & 0x03)
    b3 = ((duty & 0x0F) << 4) | (slices & 0x0F)
    b4 = hf_duty & 0x1F
    _cmd_write(0x02, 0x02, [b2, b3, b4])

def set_tx_params(raw, ramp=0x04):
    if raw < 0:
        raw = 0x100 + raw
    _cmd_write(0x02, 0x03, [raw & 0xFF, ramp & 0xFF])

# ------------------------------------------------------------------------------
# PA optimal values, Semtech reference design
#   target dBm : (TX_POWER register, PA_LF_DUTY_CYCLE, PA_LF_SLICES)
# ------------------------------------------------------------------------------
PA_TABLE_915 = {   # Table 7-19 (used here for >= 800 MHz, incl. 868 MHz)
    22.0:(22,7,7), 21.5:(22,6,7), 21.0:(22,5,6), 20.5:(21,6,7),
    20.0:(21,5,6), 19.5:(21,4,7), 19.0:(20,5,7), 18.5:(20,4,7),
    18.0:(19,5,7), 17.5:(19,5,4), 17.0:(18,7,3), 16.5:(18,5,4),
    16.0:(17,7,3), 15.5:(18,4,3), 15.0:(17,4,5), 14.5:(17,4,3),
    14.0:(17,4,2), 13.5:(16,4,3), 13.0:(16,4,2), 12.5:(15,5,2),
    12.0:(15,4,2), 11.5:(14,5,2), 11.0:(15,2,4), 10.5:(17,4,0),
    10.0:(16,1,2),
}
PA_TABLE_490 = {   # Table 7-20 (used here for < 800 MHz)
    21.0:(22,7,7), 20.5:(22,7,4), 20.0:(21,7,7), 19.5:(21,7,4),
    19.0:(20,7,6), 18.5:(22,6,2), 18.0:(19,7,6), 17.5:(19,7,3),
    17.0:(19,6,5), 16.5:(18,6,7), 16.0:(18,6,5), 15.5:(17,6,7),
    15.0:(16,7,5), 14.5:(16,6,7), 14.0:(15,7,5), 13.5:(15,7,3),
    13.0:(15,7,2), 12.5:(15,6,3), 12.0:(15,6,2), 11.5:(15,5,3),
    11.0:(15,4,5), 10.5:(17,6,0), 10.0:(15,5,1),
}

def set_tx_power(dbm, ramp=0x04, freq_hz=None):
    """
    Program PA_LF for the requested output power.
    Returns (effective_dbm, characterised) where `characterised` is False
    if the value is outside the datasheet tables (estimate only).
    """
    if freq_hz is None:
        freq_hz = FREQ_HZ
    table = PA_TABLE_490 if freq_hz < 800_000_000 else PA_TABLE_915
    lo = min(table)
    hi = max(table)

    if dbm >= lo:
        target = min(table, key=lambda x: abs(x - min(dbm, hi)))
        raw, duty, slices = table[target]
        set_pa_config(pa_sel=0, pa_lf_mode=0, duty=duty, slices=slices)
        set_tx_params(raw, ramp)
        return target, True

    # Below the lowest table entry: keep the lowest entry's duty/slices and
    # step tx_power down at 0.5 dB per LSB (Table 7-23). Not characterised.
    base_raw, duty, slices = table[lo]
    raw = base_raw + round((dbm - lo) * 2)
    raw = max(-19, min(44, raw))       # PA_LF register range
    set_pa_config(pa_sel=0, pa_lf_mode=0, duty=duty, slices=slices)
    set_tx_params(raw, ramp)
    return dbm, False

# ==============================================================================
# LoRa MODEM
# ==============================================================================
def set_lora_modulation(sf=0x7, bw=0x4, cr=0x1, ldro=0x0):
    _cmd_write(0x02, 0x20, [((sf & 0x0F) << 4) | (bw & 0x0F),
                            ((cr & 0x0F) << 4) | (ldro & 0x03)])

def set_lora_packet_params(pbl=8, pld=0xFF, hdr=0, crc=1, inv=0):
    b5 = ((hdr & 1) << 2) | ((crc & 1) << 1) | (inv & 1)
    _cmd_write(0x02, 0x21, [(pbl >> 8) & 0xFF, pbl & 0xFF, pld & 0xFF, b5])

def set_lora_syncword(sw=0x12):
    _cmd_write(0x02, 0x23, [sw & 0xFF])

def write_tx_fifo(data):
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
    print("[INIT] Resetting...")
    hw_reset()

    set_standby(0x00)
    clear_errors()
    major, minor = get_version()
    print(f"[INIT] Chip FW v{major}.{minor:02d}")

    if use_tcxo:
        print(f"[INIT] Configuring TCXO (V={tcxo_v:#x})")
        set_tcxo(voltage=tcxo_v)

    set_reg_mode(0x02 if use_dcdc else 0x00)
    print(f"[INIT] Power mode: {'SIMO DC-DC' if use_dcdc else 'LDO'}")

    set_standby(0x01)   # XOSC running
    time.sleep_ms(10)

    print("[INIT] Calibrating all blocks...")
    calibrate(CALIB_ALL)
    time.sleep_ms(5)

    print(f"[INIT] Front-end cal @ {freq_hz/1e6:.3f} MHz...")
    calib_fe(f1=freq_hz)
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
def lora_tx_once(payload, irq_dio=9):
    set_lora_modulation(sf=SF, bw=BW, cr=CR, ldro=LDRO)
    set_lora_packet_params(pbl=8, pld=len(payload), hdr=0, crc=1, inv=0)
    set_lora_syncword(SYNCWORD)
    configure_irq_pin(dio=irq_dio, mask=IRQ_TX_DONE | IRQ_TIMEOUT)

    write_tx_fifo(payload)
    set_tx(0x000000)
    wait_busy()

    while IRQ.value() == 0:
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
def lora_rx_once(irq_dio=9, timeout=0x000000):
    set_lora_modulation(sf=SF, bw=BW, cr=CR, ldro=LDRO)
    set_lora_packet_params(pbl=8, pld=0, hdr=0, crc=1, inv=0)
    set_lora_syncword(SYNCWORD)
    set_rx_path(lf=True, boost=0)
    configure_irq_pin(dio=irq_dio,
                      mask=IRQ_RX_DONE | IRQ_TIMEOUT |
                           IRQ_CRC_ERROR | IRQ_LEN_ERROR | IRQ_ADDR_ERROR)

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
    if characterised:
        print(f"[TX] PA set for {eff_dbm} dBm (datasheet table values)")
    else:
        print(f"[TX] WARNING: {TX_POWER_DBM} dBm is below the datasheet tables "
              f"(min 10 dBm). PA setting is an estimate; measure real output.")

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
