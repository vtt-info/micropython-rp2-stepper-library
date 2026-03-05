"""
This is to be run on the target machine to print the call to emulate()
with parameters extracted from the PIO program tuple.
Example usage:

```
import generate_emulate_call
import smartstepper
pgm = smartstepper.pulseGenerator.PulseGenerator._pioCode
generate_emulate_call.print_emulate_call(pgm)
```
"""

from collections import namedtuple
from array import array
from uctypes import struct as ustruct, addressof, BFUINT32, BF_POS, BF_LEN, LITTLE_ENDIAN


# see ports/rp2/modules/rp2.py

PIOProgram = namedtuple('PIOProgram', [
    'data',
    'offset_pio0',
    'offset_pio1',
    'offset_pio2',
    'execctrl',
    'shiftctrl',
    'out_init',     # None, int, or list[int] for out pin initialization
    'set_init',     # None, int, or list[int] for set pin initialization
    'sideset_init'  # None, int, or list[int] for sideset pin initialization
])

# uctypes bitfield descriptors for PIO control registers (RP2040/RP2350)
# See RP2040 datasheet section 3.7

# fmt: off
EXECCTRL_FIELDS = {
    "side_en":       0 | BFUINT32 | 30 << BF_POS | 1 << BF_LEN,
    "side_pindir":   0 | BFUINT32 | 29 << BF_POS | 1 << BF_LEN,
    "jmp_pin":       0 | BFUINT32 | 24 << BF_POS | 5 << BF_LEN,
    "out_en_sel":    0 | BFUINT32 | 19 << BF_POS | 5 << BF_LEN,
    "inline_out_en": 0 | BFUINT32 | 18 << BF_POS | 1 << BF_LEN,
    "out_sticky":    0 | BFUINT32 | 17 << BF_POS | 1 << BF_LEN,
    "wrap_top":      0 | BFUINT32 | 12 << BF_POS | 5 << BF_LEN,
    "wrap_bottom":   0 | BFUINT32 |  7 << BF_POS | 5 << BF_LEN,
    "status_sel":    0 | BFUINT32 |  5 << BF_POS | 2 << BF_LEN,
    "status_n":      0 | BFUINT32 |  0 << BF_POS | 5 << BF_LEN,
}

SHIFTCTRL_FIELDS = {
    "fjoin_rx":      0 | BFUINT32 | 31 << BF_POS | 1 << BF_LEN,
    "fjoin_tx":      0 | BFUINT32 | 30 << BF_POS | 1 << BF_LEN,
    "pull_thresh":   0 | BFUINT32 | 25 << BF_POS | 5 << BF_LEN,
    "push_thresh":   0 | BFUINT32 | 20 << BF_POS | 5 << BF_LEN,
    "out_shiftdir":  0 | BFUINT32 | 19 << BF_POS | 1 << BF_LEN,
    "in_shiftdir":   0 | BFUINT32 | 18 << BF_POS | 1 << BF_LEN,
    "autopull":      0 | BFUINT32 | 17 << BF_POS | 1 << BF_LEN,
    "autopush":      0 | BFUINT32 | 16 << BF_POS | 1 << BF_LEN,
    "fjoin_rx_put":  0 | BFUINT32 | 15 << BF_POS | 1 << BF_LEN,
    "fjoin_rx_get":  0 | BFUINT32 | 14 << BF_POS | 1 << BF_LEN,
    "in_count":      0 | BFUINT32 |  0 << BF_POS | 5 << BF_LEN,
}
# fmt: on


def uint32_bitfields(value, fields):
    """Create a uctypes struct for accessing bitfields in a uint32 value.
    Returns (struct, buf) — caller must keep buf alive while using struct."""
    buf = array('I', [value])
    return ustruct(addressof(buf), fields, LITTLE_ENDIAN), buf


def _print_fields(name, value, fields):
    s, _buf = uint32_bitfields(value, fields)
    print(f"{name}: {hex(value)}")
    for field_name in fields:
        print(f"  {field_name}: {getattr(s, field_name)}")


def _print_shiftctrl(shiftctrl):
    _print_fields("shiftctrl", shiftctrl, SHIFTCTRL_FIELDS)


def _print_execctrl(execctrl):
    _print_fields("execctrl", execctrl, EXECCTRL_FIELDS)


# Add a dummy JMP 0 instruction at the end of the program to prevent out-of-bounds access during emulation.
def _format_opcodes_as_hex(opcodes: list[int]) -> str:
    return "[" + ", ".join(f"0x{opcode:04x}" for opcode in opcodes) + ", 0x0000]"


def print_emulate_call(_prg):
    """Print Python code to call emulate() with parameters extracted from a PIO program tuple."""
    prog = PIOProgram(*_prg)
    ec, _ecbuf = uint32_bitfields(prog.execctrl, EXECCTRL_FIELDS)
    sc, _scbuf = uint32_bitfields(prog.shiftctrl, SHIFTCTRL_FIELDS)

    opcodes = list(prog.data)
    pull_thresh = sc.pull_thresh if sc.pull_thresh != 0 else 32
    push_thresh = sc.push_thresh if sc.push_thresh != 0 else 32

    lines = ["emulate("]
    lines.append(f"    opcodes={_format_opcodes_as_hex(opcodes)},")
    lines.append( "    stop_when=...,  # TODO: provide stop condition (Callable[[opcode,state],bool])")
    if sc.autopull:
        lines.append( "    auto_pull=True,")
    if sc.autopush:
        lines.append( "    auto_push=True,")
    if pull_thresh != 32:
        lines.append(f"    pull_threshold={pull_thresh},")
    if push_thresh != 32:
        lines.append(f"    push_threshold={push_thresh},")
    if not sc.in_shiftdir:
        lines.append( "    shift_isr_right=False,")
    if not sc.out_shiftdir:
        lines.append( "    shift_osr_right=False,")
    if prog.out_init is not None:
        lines.append(f"    out_base=..., # TODO provide out base")
        lines.append(f"    out_count={len(prog.out_init) if isinstance(prog.out_init, list) else 1},")
    if prog.sideset_init is not None:
        lines.append(f"    side_set_base=..., # TODO provide side_set base")
        lines.append(f"    side_set_count={len(prog.sideset_init) if isinstance(prog.sideset_init, list) else 1},")
    if prog.set_init is not None:
        lines.append(f"    set_base=..., # TODO provide set base")
        lines.append(f"    set_count={len(prog.set_init) if isinstance(prog.set_init, list) else 1},")
    if ec.side_en:
        lines.append( "    side_set_opt=True,")
    if ec.jmp_pin:
        lines.append(f"    jmp_pin={ec.jmp_pin},")
    lines.append(f"    wrap_target={ec.wrap_bottom},")
    lines.append(f"    wrap_top={ec.wrap_top},")
    lines.append(")")
    print("\n".join(lines))


def print_pio_program(_prg):
    prog = PIOProgram(*_prg)
    print(f"data ({len(prog.data)} words): {_format_opcodes_as_hex(prog.data)}")
    print(f"offset_pio0: {hex(prog.offset_pio0)}")
    print(f"offset_pio1: {hex(prog.offset_pio1)}")
    print(f"offset_pio2: {hex(prog.offset_pio2)}")
    _print_execctrl(prog.execctrl)
    _print_shiftctrl(prog.shiftctrl)
    print(f"out_init: {prog.out_init}")
    print(f"set_init: {prog.set_init}")
    print(f"sideset_init: {prog.sideset_init}")
