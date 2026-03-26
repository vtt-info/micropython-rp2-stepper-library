# -*- coding: utf-8 -*-

"""Pulse generator."""

import array
import rp2
import machine
from micropython import const

SM_FREQ = const(10_000_000)  # Hz

# PIO0 register addresses for direct hardware manipulation.
_PIO0_XOR = const(0x50201000)  # XOR alias for atomic bit toggle
_SHIFTCTRL_OFF = const(0x0D0)  # SM0_SHIFTCTRL offset; +_SM_STRIDE per SM
_SM_STRIDE = const(0x18)
_FJOIN_RX_BIT = const(1 << 30)

# PIO0 TX DREQ indices for rp2.DMA.pack_ctrl(treq_sel=...)
# Values 0-3 are identical on RP2040 and RP2350.
_PIO0_TX_DREQ = (0, 1, 2, 3)

# PIO0 TX FIFO addresses (PIO0 base 0x50200000 + TXFn byte offset)
# Identical on RP2040 and RP2350.
_PIO0_TXF_ADDR = (0x50200010, 0x50200014, 0x50200018, 0x5020001C)


class PulseGenerator:
    """Pulses generator

    Uses State Machines 0-3 on PIO0. Each instance claims the next
    available SM (up to 4 total). The DMA channel is claimed from the
    MicroPython pool independently of the SM number.
    """

    _num = 0 - 1

    def __init__(self, pin):
        """ """
        PulseGenerator._num += 1
        if PulseGenerator._num > 3:
            raise RuntimeError("Too many PulseGenerator instances")

        self._pin = pin
        self._smNum = PulseGenerator._num  # capture at init; class var may change later

        self._pulseLength = 0

        self._dma = rp2.DMA()
        self._dma.active(False)

        self._sm = rp2.StateMachine(
            self._smNum, self._pioCode, freq=SM_FREQ, sideset_base=pin
        )
        self._sm.irq(self._pulseLengthISR)
        self._sm.active(1)

    @staticmethod
    @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW)
    def _pioCode():
        """

        In the routine, OSR contains pulseLength setpoint, and Y nbPulses counter.
        X is used as pulseLength counter for each pulse. Y is decremented until 0.
        """
        # fmt: off
        # Get input values (pulseLength, nbPulses)
        # Blocking
        label("top")

        pull(block).side(0)     # 0 pull pulseLength from TX FIFO to OSR; set output LOW
        mov(x, osr)             # 1 store pulseLength to X

        # Update freq
        mov(isr, x)             # 2 store pulseLength to ISR
        push(noblock)           # 3 push back pulseLength in RX FIFO and clear ISR
        irq(rel(0))             # 4 notify ARM a new pulseLength is available

        # Cancel if pulseLength is 0
        mov(y, osr)             # 5 store pulseLength to Y
        pull(block)             # 6 pull nbPulses from TX FIFO to OSR
        jmp(not_y, "top")       # 7 jump to 'top' if Y is zero (pulseLength = 0 → end-of-sequence sentinel)
        mov(y, osr)             # 8 store nbPulses to Y
        mov(osr, x)             # 9 store back pulseLength to OSR

        # Start pulsing (square)
        label("start")

        mov(x, osr).side(1)     # 10 put previously saved pulseLength to counter; set output HIGH
        label("loopHigh")
        jmp(x_dec, "loopHigh")  # 11 loop if pulseLength is non 0; decrement pulseLength counter in all cases

        mov(x, osr).side(0)     # 12 put previously saved pulseLength to counter; set output LOW
        label("loopLow")
        jmp(x_dec, "loopLow")   # 13 loop if pulseLength is non 0; decrement pulseLength counter in all cases

        jmp(y_dec, "start")     # 14 loop if nbPulses is non 0; decrement nbPulses counter in all cases

        wrap()                  # loop to 'top'
        # fmt: on

    def _pulseLengthISR(self, smId):
        """Drain the full RX FIFO and keep the most recent pulseLength.

        MicroPython coalesces multiple PIO IRQ events into a single handler
        call. Without draining, the FIFO can fill (max 4 entries) while the
        sentinel push(noblock) silently drops its value, leaving _pulseLength
        non-zero forever. Draining all entries on each call prevents overflow
        and ensures the sentinel 0 is always captured.
        """
        while self._sm.rx_fifo() > 0:
            self._pulseLength = self._sm.get()

    @property
    def moving(self):
        """True while the motor is running.

        DMA finishes writing to the PIO TX FIFO before the PIO has consumed
        all segments — up to 2 segments can be buffered.  Relying solely on
        dma.active() would declare the move finished while the motor is still
        pulsing, causing the next moveTo() to corrupt direction and position.

        _pulseLength is set to 0 by the ISR only when the PIO processes the
        end-of-sequence sentinel, so it is the true "motion complete" signal.
        Combining both covers the full timeline:
          - DMA active: dma.active() is True
          - DMA done, PIO draining FIFO: _pulseLength is still non-zero
          - Sentinel consumed: ISR sets _pulseLength = 0 → False
        After stop(), both are explicitly cleared, so moving returns False.
        """
        return self._dma.active() or self._pulseLength != 0

    @property
    def freq(self):
        """Return current frequency."""
        try:
            freq = round(SM_FREQ / self._pulseLength / 2)
        except ZeroDivisionError:
            freq = 0

        return freq

    def _buildSequence(self, points):
        """Build a DMA-ready word array from (freq, nbPulses) tuples.
        Appends a 0 pulseLength sentinel so the PIO knows when the sequence ends.
        """
        sequence = array.array("I", bytearray(len(points) * 8 + 8))
        i = 0
        for freq, nbPulses in points:
            if nbPulses < 1:  # guard: 0-pulse segments wrap to 0xFFFFFFFF → 2³² steps
                continue
            sequence[i] = round(SM_FREQ / freq / 2)
            sequence[i + 1] = nbPulses - 1
            i += 2
        return sequence

    def _configureDMA(self, sequence):
        """Configure DMA for the given sequence array without triggering.

        Saves sequence as self._sequence to prevent GC from reclaiming it
        while DMA is active. The array is passed directly as the read source;
        rp2.DMA resolves its address via the buffer protocol.
        """
        self._sequence = sequence  # keep alive for DMA
        ctrl = self._dma.pack_ctrl(
            size=2,  # word transfers
            inc_read=True,
            inc_write=False,
            treq_sel=_PIO0_TX_DREQ[self._smNum],
        )
        self._dma.config(
            read=sequence,
            write=_PIO0_TXF_ADDR[self._smNum],
            count=len(sequence),
            ctrl=ctrl,
            trigger=False,
        )

    def _triggerDMA(self):
        """Fire a previously configured DMA channel."""
        self._dma.active(True)

    def _startDMA(self, sequence):
        """Configure and immediately trigger DMA for the given sequence array."""
        self._configureDMA(sequence)
        self._triggerDMA()

    @property
    def dma_channel(self):
        """Return the integer DMA channel number (0–11)."""
        return self._dma.channel

    @staticmethod
    def trigger_channels(bitmask):
        """Simultaneously trigger multiple DMA channels.

        bitmask is a bitfield where bit N corresponds to DMA channel N.
        Uses the RP2040/RP2350 DMA_MULTI_CHAN_TRIGGER register (single AHB
        write) for hardware-guaranteed simultaneous start.
        """
        machine.mem32[0x50000430] = bitmask

    def prepare(self, points):
        """Configure DMA for the given points without triggering.

        Stops any prior DMA and programs the new sequence so that a subsequent
        call to _triggerDMA() (or trigger_channels()) will start it instantly.
        For multi-axis use: call prepare() on each axis, then fire all channels
        simultaneously via trigger_channels(bitmask).

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)  # stop any prior DMA
        self._configureDMA(self._buildSequence(points))

    def start(self, points):
        """

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)  # stop any prior DMA
        self._startDMA(self._buildSequence(points))

    def update(self, points):
        """Replace the remaining motion profile without stopping the motor.

        Non-blocking. Aborts the current DMA transfer and immediately starts a
        new one from the rebuilt sequence. Any data already in the PIO TX FIFO
        (at most 2 old segments) drains naturally before the new DMA data takes
        over, giving a seamless transition at the current speed.

        NOTE: only effective between PIO segments. If the current segment has
        many pulses remaining (e.g. a long cruise), the new profile will not
        start until that segment finishes. Use interrupt_with() when the new
        profile must start within one half-period.

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)
        self._startDMA(self._buildSequence(points))

    def interrupt_with(self, points):
        """Interrupt the current segment and immediately start a new profile.

        Zeroes the PIO Y register via sm.exec(), causing the current segment to
        end after the current half-period (at most 1/freq seconds). The new DMA
        is queued before the PIO reaches pull(block), so the transition is
        smooth with at most one extra half-period at the old speed.

        Use this instead of update() when the current segment may be long (e.g.
        a cruise phase) and the new profile must start promptly — for example,
        a smooth stop triggered mid-move.

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)
        self._sm.exec("mov(y, null)")  # end current segment after this half-period
        self._startDMA(self._buildSequence(points))

    def stop(self):
        """Stop pulse generation immediately.

        Freezes the state machine, clears both FIFOs (using the pico-sdk
        pio_sm_clear_fifos pattern: toggle SHIFTCTRL.FJOIN_RX twice), resets
        the SM program counter, and re-enables so it blocks at instruction 0
        (pull(block).side(0)) with the step pin LOW.

        Total time ~50 us vs ~80 ms for the old sm.exec()-string approach.
        """
        self._dma.active(False)  # 1. Stop DMA feeding FIFO

        self._sm.active(0)  # 2. Freeze SM — no more pulses

        # 3. Clear both FIFOs (toggle FJOIN_RX twice, per pico-sdk)
        xor_addr = _PIO0_XOR + _SHIFTCTRL_OFF + self._smNum * _SM_STRIDE
        machine.mem32[xor_addr] = _FJOIN_RX_BIT
        machine.mem32[xor_addr] = _FJOIN_RX_BIT

        self._sm.restart()  # 4. Reset PC to wrap_target (instr 0)

        self._sm.active(1)  # 5. Re-enable: pull(block).side(0)
        #    → pin LOW, blocks on empty FIFO

        self._pulseLength = 0  # 6. Mark as not moving

    @property
    def dma_count(self):
        """Return the number of DMA transfers currently in progress."""
        return self._dma.count
