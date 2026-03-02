# -*- coding: utf-8 -*-

""" Pulse generator.
"""

import array
import rp2

SM_FREQ = 10_000_000  # Hz

# PIO0 TX DREQ indices for rp2.DMA.pack_ctrl(treq_sel=...)
# Values 0-3 are identical on RP2040 and RP2350.
_PIO0_TX_DREQ = (0, 1, 2, 3)

# PIO0 TX FIFO addresses (PIO0 base 0x50200000 + TXFn byte offset)
# Identical on RP2040 and RP2350.
_PIO0_TXF_ADDR = (0x50200010, 0x50200014, 0x50200018, 0x5020001c)


class PulseGenerator:
    """ Pulses generator

    Uses State Machines 0-3 on PIO0. Each instance claims the next
    available SM (up to 4 total). The DMA channel is claimed from the
    MicroPython pool independently of the SM number.
    """
    _num = 0 - 1

    def __init__(self, pin):
        """
        """
        PulseGenerator._num += 1
        if PulseGenerator._num > 3:
            raise RuntimeError("Too many PulseGenerator instances")

        self._pin = pin
        self._smNum = PulseGenerator._num  # capture at init; class var may change later

        self._pulseLength = 0

        self._dma = rp2.DMA()
        self._dma.active(False)

        self._sm = rp2.StateMachine(self._smNum, self._pioCode, freq=SM_FREQ, sideset_base=pin)
        self._sm.irq(self._pulseLengthISR)
        self._sm.active(1)

    @staticmethod
    @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW)
    def _pioCode():
        """

        In the routine, ISR contains pulseLength setpoint, and Y nbPulses counter.
        X is used as pulseLength counter for each level. Y is decremented until 0.
        """

        # Get input values (pulseLength, nbPulses)
        # Blocking
        pull().side(0)          # pull pulseLength from TX FIFO to OSR; set output LOW
        mov(x, osr)             # store pulseLength to X

        # Update freq
        mov(isr, x)             # store pulseLength to ISR
        push()                  # push back pulseLength in RX FIFO
        irq(noblock, rel(0))    # notify ARM a new pulseLength is available

        # Cancel if pulseLength is 0
        mov(y, osr)             # store pulseLength to Y
        pull()                  # pull nbPulses from TX FIFO to OSR
        jmp(y_dec, "end")       # jump to 'end' if Y is non-zero (pulseLength > 0 → end-of-sequence sentinel)

        mov(y, osr)             # store nbPulses to Y
        mov(osr, x)             # store back pulseLength to OSR

        # Start pulsing (square)
        label("start")

        mov(x, osr).side(1)     # put previously saved pulseLength to counter; set output HIGH
        label("loopHigh")
        jmp(x_dec, "loopHigh")  # loop if pulseLength is non 0; decrement pulseLength counter in all cases

        mov(x, osr).side(0)     # put previously saved pulseLength to counter; set output LOW
        label("loopLow")
        jmp(x_dec, "loopLow")   # loop if pulseLength is non 0; decrement pulseLength counter in all cases

        jmp(y_dec, "start")     # loop if nbPulses is non 0; decrement nbPulses counter in all cases

        label("end")

    def _pulseLengthISR(self, smId):
        """
        """
        self._pulseLength = self._sm.get()

    @property
    def freq(self):
        """ Return current frequency.
        """
        try:
            freq = round(SM_FREQ / self._pulseLength / 2)
        except ZeroDivisionError:
            freq = 0

        return freq

    def _buildSequence(self, points):
        """ Build a DMA-ready word array from (freq, nbPulses) tuples.

        Appends a sentinel so the PIO knows when the sequence ends.

        The sentinel uses pulseLength=0xFFFFFFFF (not 0) so that the PIO's
        end-of-sequence check (jmp y_dec, "end") actually jumps — the jmp
        condition is true when Y is non-zero, so pulseLength=0 would fall
        through into the pulse loop and fire a spurious extra step.
        freq(0xFFFFFFFF) = round(SM_FREQ / 0xFFFFFFFF / 2) = 0, so the ISR
        still reports freq=0 and moving=False correctly.
        """
        sequence = array.array('I')
        for freq, nbPulses in points:
            sequence.append(round(SM_FREQ / freq / 2))
            sequence.append(nbPulses - 1)
        sequence.extend(array.array('I', (0xFFFFFFFF, 0)))
        return sequence

    def _startDMA(self, sequence):
        """ Configure and trigger DMA for the given sequence array.

        Saves sequence as self._sequence to prevent GC from reclaiming it
        while DMA is active. The array is passed directly as the read source;
        rp2.DMA resolves its address via the buffer protocol.
        """
        self._sequence = sequence  # keep alive for DMA
        ctrl = self._dma.pack_ctrl(
            size=2,         # word transfers
            inc_read=True,
            inc_write=False,
            treq_sel=_PIO0_TX_DREQ[self._smNum],
        )
        self._dma.config(
            read=sequence,
            write=_PIO0_TXF_ADDR[self._smNum],
            count=len(sequence),
            ctrl=ctrl,
            trigger=True,
        )

    def start(self, points):
        """

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)  # stop any prior DMA
        self._startDMA(self._buildSequence(points))

    def update(self, points):
        """ Replace the remaining motion profile without stopping the motor.

        Non-blocking. Aborts the current DMA transfer and immediately starts a
        new one from the rebuilt sequence. Any data already in the PIO TX FIFO
        (at most 2 old segments) drains naturally before the new DMA data takes
        over, giving a seamless transition at the current speed.

        points contains n tuples (freq, nbPulses) values.
        """
        self._dma.active(False)
        self._startDMA(self._buildSequence(points))

    def stop(self):
        """
        """
        self._dma.active(False)  # abort DMA

        self._sm.exec("mov(y, null)")    # Stop steps loop
        self._sm.exec("mov(y, null)")    # Stop steps loop
        self._sm.exec("mov(y, null)")    # Stop steps loop
        self._sm.exec("mov(y, null)")    # Stop steps loop

        self._sm.exec("nop().side(0)")   # ensure pulse is low

        self._pulseLength = 0  # needed?

