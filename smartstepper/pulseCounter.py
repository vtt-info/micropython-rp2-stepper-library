# -*- coding: utf-8 -*-

"""Pulse counter with direction sensing and DMA position output.

Uses a single PIO state machine (PIO1) that monitors step and direction
pins, maintaining a 32-bit position counter.  The current position is
written to a memory buffer after every step via DMA so the CPU can read
it without touching any FIFO.

The position can be set to any value while the motor is running by
writing to the SM's TX FIFO; the new value takes effect at the top of
the next step-loop iteration (i.e. after the current step edge is
processed).

DIR pin HIGH → increment (positive) direction
DIR pin LOW  → decrement (negative) direction

Up to 4 simultaneous PulseCounter instances are supported (PIO1
state machines 4-7).
"""

import array
import machine
import rp2

# PIO1 RX FIFO addresses (PIO1 base 0x50300000 + RXF0 offset 0x20).
# Identical on RP2040 and RP2350.
_PIO1_RXF_ADDR = (0x50300020, 0x50300024, 0x50300028, 0x5030002C)

# DREQ indices for PIO1 RX SM0–SM3 (RP2040/RP2350 Table 120).
_PIO1_RX_DREQ = (12, 13, 14, 15)


class PulseCounter:
    """Step/direction pulse counter with DMA-backed position output.

    Usage::

        counter = PulseCounter(step_pin=2, dir_pin=3)
        print(counter.value)   # signed position, updated after every step
        counter.value = 0      # set position while running
        counter.deinit()       # release SM and DMA resources
    """

    _num = 4 - 1  # next SM index; incremented before first use → SM 4

    _OFFSET = 1 << 31  # 2^31; raw X register value = position + _OFFSET

    def __init__(self, stepPin, dirPin):
        """Create a PulseCounter.

        *stepPin* is the step input (rising-edge counted); *dirPin* is the
        direction input (HIGH = increment, LOW = decrement).  Both may be
        GPIO numbers or ``machine.Pin`` instances.
        """
        PulseCounter._num += 1
        if PulseCounter._num > 7:
            raise RuntimeError("Too many PulseCounter instances (max 4)")

        sm_idx = PulseCounter._num  # 4–7
        pio1_sm = sm_idx - 4  # 0–3 within PIO1

        if not isinstance(stepPin, machine.Pin):
            stepPin = machine.Pin(stepPin, machine.Pin.IN)
        if not isinstance(dirPin, machine.Pin):
            dirPin = machine.Pin(dirPin, machine.Pin.IN)

        self._stepPin = stepPin
        self._dirPin = dirPin

        # One-word output buffer.  DMA writes the raw 32-bit counter value
        # here after every step.  Pre-filled with _OFFSET so value == 0
        # before any steps occur.
        self._pos_buf = array.array("I", [self._OFFSET])

        # ── State machine ────────────────────────────────────────────────────
        self._sm = rp2.StateMachine(
            sm_idx,
            PulseCounter._pioCode,
            in_base=stepPin,
            jmp_pin=dirPin,
        )

        # ─ DMA: two channels chained to each other for continuous output ──
        # Each transfers 0x7FFFFFFF words then triggers the other, giving
        # uninterrupted position updates to _pos_buf for the life of the
        # object.
        rx_addr = _PIO1_RXF_ADDR[pio1_sm]
        dreq = _PIO1_RX_DREQ[pio1_sm]

        self._dma_a = rp2.DMA()
        self._dma_b = rp2.DMA()

        ctrl_a = self._dma_a.pack_ctrl(
            size=2,  # 32-bit transfers
            inc_read=False,  # RX FIFO: fixed address
            inc_write=False,  # position buffer: single word, fixed address
            treq_sel=dreq,  # pace by PIO1 RX DREQ
            chain_to=self._dma_b.channel,
        )
        ctrl_b = self._dma_b.pack_ctrl(
            size=2,
            inc_read=False,
            inc_write=False,
            treq_sel=dreq,
            chain_to=self._dma_a.channel,
        )

        self._dma_a.config(
            read=rx_addr,
            write=self._pos_buf,
            count=0x7FFFFFFF,
            ctrl=ctrl_a,
            trigger=False,
        )
        self._dma_b.config(
            read=rx_addr,
            write=self._pos_buf,
            count=0x7FFFFFFF,
            ctrl=ctrl_b,
            trigger=False,
        )

        # ── Start ────────────────────────────────────────────────────────────
        # Pre-load the TX FIFO so the SM's very first pull(noblock) sets
        # X = _OFFSET (position 0) before any step edge arrives.
        self._sm.put(self._OFFSET)
        self._sm.active(1)
        self._dma_a.active(1)

    # ── PIO program ──────────────────────────────────────────────────────────

    @staticmethod
    @rp2.asm_pio()
    def _pioCode():
        """Single-SM step/direction counter with DMA-ready position output.

        Registers:
          X   — raw position (unsigned; signed position = X − 2^31)
          OSR — scratch / new position from TX FIFO
          ISR — scratch for push

        TX FIFO → optional new raw position (set-while-running)
        RX FIFO → updated raw position after each step (consumed by DMA)

        Program layout (12 instructions):

          0  pull(noblock)        ← wrap_target
          1  mov(x, osr)
          2  wait(0, pin, 0)
          3  wait(1, pin, 0)
          4  jmp(pin, "inc")      DIR HIGH → 7
          5  jmp(x_dec, "push")   decrement; X!=0 → 10
          6  jmp("push")          decrement underflow (X was 0) → 10
          7  mov(x, invert(x))    [inc]
          8  jmp(x_dec, "inc2")   always → 9
          9  mov(x, invert(x))    [inc2]  ~(~X-1) = X+1
         10  mov(isr, x)          [push]
         11  push(noblock)        → wrap to 0
        """
        # fmt: off
        wrap_target()

        # ── Position override check ──────────────────────────────────────────
        # pull(noblock): TX FIFO empty → copies X to OSR (mov becomes no-op).
        #                TX FIFO has data → loads it into OSR (mov updates X).
        pull(noblock)
        mov(x, osr)

        # ── Rising-edge detection on step pin (in_base pin 0) ────────────────
        wait(0, pin, 0)
        wait(1, pin, 0)

        # ── Direction branch (jmp_pin = dir pin) ─────────────────────────────
        jmp(pin, "inc")          # DIR HIGH → increment

        # ── Decrement path: X-- ──────────────────────────────────────────────
        # jmp(x_dec, …) always decrements X.  Branches when X was non-zero;
        # falls through when X was 0 (wraps to 0xFFFFFFFF).  Both reach "push".
        jmp(x_dec, "push")
        jmp("push")              # X was 0 → underflowed to 0xFFFFFFFF

        # ── Increment path: X++ via invert–dec–invert ────────────────────────
        label("inc")
        mov(x, invert(x))
        jmp(x_dec, "inc2")       # decrements ~X; always reaches "inc2"
        label("inc2")
        mov(x, invert(x))        # ~(~X − 1) = X + 1

        # ── Publish updated position to RX FIFO (read by DMA) ────────────────
        label("push")
        mov(isr, x)
        push(noblock)            # X is always correct; noblock drops the push
                                 # only if RX FIFO is full (DMA too slow),
                                 # which does not affect the X register.
        # fmt: on
        # implicit wrap → wrap_target

    # ── Python API ───────────────────────────────────────────────────────────

    @property
    def value(self):
        """Current position as a signed integer.

        Reads directly from the DMA-updated memory buffer — no FIFO access,
        no SM stall.  The value is updated after every step edge.
        """
        return self._pos_buf[0] - self._OFFSET

    @value.setter
    def value(self, val):
        """Set the position to *val* (signed integer).

        ``_pos_buf`` is updated immediately so reads reflect the new value
        at once.  The SM's X register is updated via the TX FIFO; if the
        motor is running the change takes effect at the top of the next
        step loop (after the current step is processed).  If the motor is
        stopped, X is updated when motion next starts.

        The TX FIFO is 4 entries deep.  Avoid calling the setter repeatedly
        in rapid succession while the motor is stopped, or only the last
        enqueued value will end up in X.
        """
        raw = self._OFFSET + val
        self._pos_buf[0] = raw  # immediate effect for reads
        self._sm.put(raw)  # enqueue for SM to load on next pull(noblock)

    def deinit(self):
        """Stop the DMA channels and state machine, freeing all resources."""
        self._dma_b.active(0)
        self._dma_a.active(0)
        self._sm.active(0)
