from pioemu import conditions, emulate, State, disassemble


opcodes=[0x90a0, 0xa027, 0xa0c1, 0x8000, 0xc010, 0xa047, 0x80a0, 0x0060, 0xa047, 0xa0e1, 0xb827, 0x004b, 0xb027, 0x004d, 0x008a, 0x0000]

def stop_condition(opcode, state):
    """
    Return True if the emulation should stop after executing the instruction with the given opcode and state.
    """
    return len(state.transmit_fifo) == 0


def print_fifo_contents(fifo):
    return "[" + ", ".join(f"{item:#x}" for item in fifo) + "]"


def run_emulation(opcodes, pairs):
    # GPIO0 is an output.
    init = State(pin_directions=0x01)

    generator = emulate(
        opcodes=opcodes,
        stop_when=stop_condition,
        initial_state=init,
        shift_isr_right=False,
        shift_osr_right=False,
        side_set_base=0,  # GPIO0
        side_set_count=1,
        side_set_opt=True,
        wrap_target=0,
        wrap_top=14,
    )

    def data_generator():
        for pair in pairs:
            yield pair[0]
            yield pair[1]

    txgen = data_generator()

    # Ensure that the TX FIFO stays filled if possible.
    def fill_txfifo(state):
        while len(state.transmit_fifo) < 3:
            try:
                state.transmit_fifo.append(next(txgen))
                state.transmit_fifo.append(next(txgen))
            except StopIteration:
                return
    
    def print_status(opcode, before, after):
        print(
            f"{before.clock:04d} {after.clock:04d} PC={before.program_counter:02d} "
            f"{disassemble(opcode, side_set_count=1, side_set_opt=True):20s} "
            f"X={after.x_register:<8x} Y={after.y_register:<8x} "
            f"ISR={after.input_shift_register.contents:<8x} OSR={after.output_shift_register.contents:<8x} "
            f"IRQ={after.irq_flags:02b} "
            f"TX FIFO={print_fifo_contents(after.transmit_fifo)} RX FIFO={print_fifo_contents(after.receive_fifo)}"
        )

    fill_txfifo(init)

    pc = -1
    last_clock = 0
    final = init

    for before, after in generator:
        # print(f"    {before} -> {after}")
        if before.program_counter != pc:
            if before.clock != last_clock:
                print(f"--- {before.clock - last_clock} cycles ---")
            pc = before.program_counter
            opcode = opcodes[pc]
            print_status(opcode, before, after)
            last_clock = after.clock
        fill_txfifo(after)
        final = after

    pc = before.program_counter
    opcode = opcodes[pc]
    print_status(opcode, final, final)


def print_disassembly():
    print("\nDisassembly:")
    for i, opcode in enumerate(opcodes):
        print(
            f"{i:02d}: {opcode:04x} {disassemble(opcode, side_set_count=1, side_set_opt=True)}"
        )
    print("")


# (pulseLength, nbPulses-1)
pairs = [(5, 2), (1, 3), (0, 0)]

print_disassembly()
run_emulation(opcodes, pairs)
