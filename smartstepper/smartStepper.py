# -*- coding: utf-8 -*-

""" Stepper control.

TODO:
    x update jog routine
    x allow several (max 4?) instances (share PIO)
    x generate accel slices in speed instead of time CANCELED
    x update position
    x use DMA class
    x move State Machine functions to dedicated module
    x implement timeout
    - implement error management -> status
    x implement enable pin
    x centralize accel computations
    - dynamically compute NB_ACCEL_PTS (which criteria?)

BUGS:
    -
"""

import time
import math
import machine

from . import pulseGenerator
from . import pulseCounter

NB_ACCEL_PTS = 100


class SmartStepperError(Exception):
    """ SmartStepperError class
    """
    pass


class SmartStepper:
    """ SmartStepper class
    """
    def __init__(self, stepPin, dirPin, enablePin=None, accelCurve='smooth2'):
        """ Init SmartStepper object

        accelCurve can be in ('linear', smooth1', 'smooth2', 'sine')
        """
        if not isinstance(stepPin, machine.Pin):
            stepPin = machine.Pin(stepPin, machine.Pin.OUT)

        if not isinstance(dirPin, machine.Pin):
            dirPin = machine.Pin(dirPin, machine.Pin.OUT)

        self._directionPin = dirPin

        if enablePin is not None and not isinstance(enablePin, machine.Pin):
            enablePin = machine.Pin(enablePin, machine.Pin.OUT)
        self._enablePin = enablePin
        if self._enablePin is not None:
            self._enablePin.high()  # active-low: start disabled

        self._stepsPerUnit = 1    # steps per unit
        self._minSpeed = 10       # minimum speed, in units per second
        self._maxSpeed = 100      # maximum speed, in units per second
        self._acceleration = 100  # acceleration, in units per second square
        self._reverse = False     # reverse dirPin level

        self._target = 0          # target position, in units
        self._direction = None    # current direction
        self._jogging = False     # True when in jog mode (no fixed target)
        self._moveDeadline = None # ticks_ms deadline for timeout, or None

        self._accelTable = []
        self._initAccelTable(accelCurve)

        self._pulseGenerator = pulseGenerator.PulseGenerator(stepPin)
        self._pulseCounter = pulseCounter.PulseCounter(stepPin)

    def __repr__(self):
        return f"SmartStepper(target={self._target}, direction={self._direction}, speed={self.speed}, jog={self._jogging})"

    def _initAccelTable(self, accelCurve='smooth2'):
        """ Init acceleration table
        """
        for i in range(NB_ACCEL_PTS+1):
            t = i / NB_ACCEL_PTS
            y = self._accel(t, accelCurve)
            self._accelTable.append(y)

    def _accel(self, x, accelCurve='smooth2'):
        """ Compute acceleration
        """
        if accelCurve == 'linear':
            return x

        elif accelCurve == 'smooth1':
            return x * x * (3 - 2 * x)

        elif accelCurve == 'smooth2':
            return x * x * x * (x * (x * 6 - 15) + 10)

        elif accelCurve == 'sine':
            return (math.cos((x + 1) * math.pi) + 1) / 2

        else:
            raise SmartStepperError(f"Unknown '{accelCurve}' acceleration curve")

    def _accelPoints(self, fromSpeed, toSpeed, accel=None):
        """ Compute acceleration (or deceleration) points from fromSpeed to toSpeed.

        For deceleration (fromSpeed > toSpeed), generates the reversed acceleration
        profile, which is valid because the smoothstep curves are point-symmetric.
        Floating-point overshoot is trimmed rather than raised.

        accel overrides self._acceleration for this call (used by triangular and
        accel_time moves that require a different effective acceleration).
        """
        if accel is None:
            accel = self._acceleration

        if fromSpeed > toSpeed:
            pts = self._accelPoints(toSpeed, fromSpeed, accel=accel)
            pts.reverse()
            return pts

        points = []

        accelTime = (toSpeed - fromSpeed) / accel
        accelDist = (toSpeed**2 - fromSpeed**2) / (2 * accel)
        accelSteps = round(accelDist * self._stepsPerUnit)

        realSteps = 0
        dt = accelTime / NB_ACCEL_PTS
        for i in range(NB_ACCEL_PTS):
            y = self._accelTable[i]
            speed = toSpeed * y + fromSpeed * (1 - y)
            pulses = round(speed * self._stepsPerUnit * dt)
            if pulses:
                points.append((speed * self._stepsPerUnit, pulses))
            realSteps += pulses

        if realSteps < accelSteps:
            points.append((toSpeed * self._stepsPerUnit, accelSteps - realSteps))
        elif realSteps > accelSteps:
            # Trim excess steps caused by floating-point rounding
            excess = realSteps - accelSteps
            while excess > 0 and points:
                freq, n = points[-1]
                trim = min(n, excess)
                if n - trim > 0:
                    points[-1] = (freq, n - trim)
                else:
                    points.pop()
                excess -= trim

        return points

    def _buildProfile(self, fromSpeed, remaining, triangular=False, forced_peak=None):
        """ Build a complete motion profile from fromSpeed over remaining distance.

        Computes the achievable peak speed given the distance, then produces
        accel → [const speed] → decel segments.

        triangular=True: no constant-velocity section; peak speed is the natural
        fit for the distance. If the natural peak would exceed maxSpeed, the
        effective acceleration is reduced so peak == maxSpeed exactly.

        forced_peak: skip peak computation and use this value directly (used by
        accel_time moves and MultiAxis). A const-speed section is added if needed.

        Returns a list of (freq_steps_per_s, nbPulses) tuples for the pulse generator.
        """
        if abs(remaining) < 1.0 / self._stepsPerUnit:
            return []

        accel = self._acceleration  # may be overridden below

        if forced_peak is not None:
            peakSpeed = min(forced_peak, self._maxSpeed)
            accelUpDist = (peakSpeed**2 - fromSpeed**2) / (2 * accel)
            decelDist   = (peakSpeed**2 - self._minSpeed**2) / (2 * accel)
            if accelUpDist + decelDist > abs(remaining):
                peakSpeed = math.sqrt(
                    (abs(remaining) * 2 * accel + fromSpeed**2 + self._minSpeed**2) / 2
                )
        else:
            # Peak speed achievable given remaining distance starting at fromSpeed.
            # Derived from: accelUpDist + decelDist = |remaining|
            #   (peak²-fromSpeed²)/(2a) + (peak²-minSpeed²)/(2a) = |remaining|
            # This formula is valid for all four accel curve types because every
            # curve has a mean value of 0.5, making distance = (v²-v0²)/(2a).
            peak = math.sqrt(
                (abs(remaining) * 2 * accel + fromSpeed**2 + self._minSpeed**2) / 2
            )

            if triangular and peak > self._maxSpeed:
                # Reduce acceleration so the move stays triangular at maxSpeed.
                # Solve peak=maxSpeed in the distance formula for accel:
                #   maxSpeed² = (|remaining|·2·accel_eff + fromSpeed² + minSpeed²) / 2
                accel = (2 * self._maxSpeed**2 - fromSpeed**2 - self._minSpeed**2) / (2 * abs(remaining))
                peakSpeed = self._maxSpeed
            else:
                peakSpeed = min(peak, self._maxSpeed)

        points = []

        # Accel phase: fromSpeed → peakSpeed
        if fromSpeed < peakSpeed - 1e-6:
            points.extend(self._accelPoints(fromSpeed, peakSpeed, accel=accel))

        # Const speed phase — omitted for triangular moves; present when forced_peak
        # is set (accel_time / MultiAxis) or when the move hits the speed ceiling.
        if not triangular and (forced_peak is not None or peakSpeed >= self._maxSpeed - 1e-6):
            accelUpDist = (peakSpeed**2 - fromSpeed**2) / (2 * accel)
            decelDist   = (peakSpeed**2 - self._minSpeed**2) / (2 * accel)
            constDist   = abs(remaining) - accelUpDist - decelDist
            constSteps  = round(constDist * self._stepsPerUnit)
            if constSteps > 0:
                points.append((peakSpeed * self._stepsPerUnit, constSteps))

        # Decel phase: peakSpeed → minSpeed
        points.extend(self._accelPoints(peakSpeed, self._minSpeed, accel=accel))

        return points

    def _profile_time(self, distance, forced_peak=None):
        """Return the actual total move duration for a given distance.

        Builds the full motion profile (same computation as prepare_move /
        moveTo) and sums the real PIO-segment durations.  This accounts for
        integer-step rounding in _accelPoints so the result matches what the
        hardware will actually execute, unlike a continuous-time approximation.

        Used by MultiAxis to get an accurate T_dominant and to drive the
        binary search for subordinate-axis peak speeds.
        """
        points = self._buildProfile(self._minSpeed, distance, forced_peak=forced_peak)
        if not points:
            return 0.0
        return sum(n / f for f, n in points)

    def _updateDirection(self, direction):
        """ Update dir pin / pulseCounter according to direction
        """
        if direction == 'up':
            self._directionPin.high()
            self._pulseCounter.direction = 'up'

        else:
            self._directionPin.low()
            self._pulseCounter.direction = 'down'

        self._direction = direction

        if self._reverse:
            self._directionPin.toggle()

    def enable(self):
        """ Enable the stepper driver (active-low enable pin).

        No-op if no enable pin was configured.
        """
        if self._enablePin is not None:
            self._enablePin.low()

    def disable(self):
        """ Disable the stepper driver (active-low enable pin).

        No-op if no enable pin was configured.
        """
        if self._enablePin is not None:
            self._enablePin.high()

    @property
    def minSpeed(self):
        """ Get the min speed

        minSpeed is in units per second.
        """
        return self._minSpeed

    @minSpeed.setter
    def minSpeed(self, value):
        """ Set the min speed

        minSpeed is in units per second.
        Can be changed while moving; triggers a motion replan.
        """
        if value == 0:
            raise SmartStepperError("min speed must be > 0")

        if value > self._maxSpeed:
            raise SmartStepperError("min speed must be <= max speed")

        self._minSpeed = value
        if self.moving:
            self._replan()

    @property
    def maxSpeed(self):
        """ Get the max speed

        maxSpeed is in units per second.
        """
        return self._maxSpeed

    @maxSpeed.setter
    def maxSpeed(self, value):
        """ Set the max speed

        maxSpeed is in units per second.
        Can be changed while moving; triggers a motion replan.
        """
        if value == 0:
            raise SmartStepperError("max speed must be > 0")

        if value < self._minSpeed:
            raise SmartStepperError("max speed must be >= min speed")

        self._maxSpeed = value
        if self.moving:
            self._replan()

    @property
    def speed(self):
        """ Get the current speed

        speed is in units per second.
        """
        return self._pulseGenerator.freq / self._stepsPerUnit

    @property
    def direction(self):
        """ Get the current direction
        """
        return self._direction

    @property
    def acceleration(self):
        """ Get the acceleration

        acceleration is in units per second square.
        """
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        """ Set the acceleration

        acceleration is in units per second square.
        Can be changed while moving; triggers a motion replan.
        """
        self._acceleration = value
        if self.moving:
            self._replan()

    @property
    def stepsPerUnit(self):
        """ Get the number of steps per unit
        """
        return self._stepsPerUnit

    @stepsPerUnit.setter
    def stepsPerUnit(self, value):
        """ Set the number of steps per unit
        """
        if self.moving:
            raise SmartStepperError("Can't change 'steps per unit' while moving")

        self._stepsPerUnit = value

    @property
    def reverse(self):
        """ get the reverse flag
        """
        return self._reverse

    @reverse.setter
    def reverse(self, value):
        """ Set the reverse flag
        """
        if self.moving:
            raise SmartStepperError("Can't change 'reverse' flag while moving")

        self._reverse = value

    @property
    def target(self):
        """ get the target

        Target is in units.
        """
        return self._target

    @property
    def position(self):
        """ Get the current position

        Position is in units.
        """
        return self._pulseCounter.value / self._stepsPerUnit

    @position.setter
    def position(self, value):
        """ Set the current position

        Can be used to reset the position.
        Position is in units.
        """
        if self.moving:
            raise SmartStepperError("Can't change 'position' while moving")

        self._pulseCounter.value = round(value * self._stepsPerUnit)

    @property
    def moving(self):
        """ Check if moving
        """
        return self._pulseGenerator.moving

    def jog(self, maxSpeed=None, direction='up'):
        """ Jog at given speed

        maxSpeed is in units per second.
        Handle acceleration.
        Non blocking.
        """
#         print("\nTRACE::jog()")

        if self.moving:
            raise SmartStepperError("Can't 'jog' while moving")

        if maxSpeed is None:
            maxSpeed = self._maxSpeed

        elif not self._minSpeed <= maxSpeed <= self._maxSpeed:
            raise SmartStepperError("'maxSpeed' is out of range")

        self._jogging = True
        self.enable()
        self._updateDirection(direction)

        points = []

        # Acceleration
        points.extend(self._accelPoints(self._minSpeed, maxSpeed))

        # Constant speed
#         print("\nDEBUG::Constant speed phase:")
        maxDist = 60 * maxSpeed  # distance for 1min run
        points.append((maxSpeed * self._stepsPerUnit, round(maxDist * self._stepsPerUnit)))

        # Start the generator
        self._pulseGenerator.start(points)

    def moveTo(self, target, relative=False, timeout=None, triangular=False, accel_time=None):
        """ Move to target

        target is in units.
        timeout is in seconds (None = no timeout).
        triangular=True: no constant-velocity section; motor accelerates then
            immediately decelerates. If the natural peak would exceed maxSpeed,
            acceleration is reduced so the move is triangular at maxSpeed.
        accel_time: duration of the acceleration phase in seconds. The peak speed
            is fromSpeed + acceleration * accel_time, clamped to maxSpeed. Any
            remaining distance is covered at constant speed.
        Handle acceleration.
        Non blocking.
        """
#         print("\nTRACE::moveTo()")

        if self.moving:
            raise SmartStepperError("Can't 'moveto' while moving")

        self._jogging = False
        self.enable()

        if timeout is not None:
            self._moveDeadline = time.ticks_add(time.ticks_ms(), round(timeout * 1000))
        else:
            self._moveDeadline = None

        if not relative:
            self._target = target
        else:
            self._target = self.position + target

        # Compute direction
        if self._target > self.position:
            self._updateDirection('up')

        else:
            self._updateDirection('down')

        remaining = abs(self._target - self.position)

        forced_peak = None
        if accel_time is not None:
            forced_peak = self._minSpeed + self._acceleration * accel_time
            forced_peak = min(forced_peak, self._maxSpeed)
            if forced_peak < self._minSpeed:
                raise SmartStepperError("accel_time too short: peak speed below minSpeed")

        points = self._buildProfile(self._minSpeed, remaining,
                                    triangular=triangular, forced_peak=forced_peak)

        # Start the generator
        self._pulseGenerator.start(points)

    def _prepare_move(self, target, relative=False, accel_time=None, triangular=False,
                      forced_peak=None):
        """ Set up a move and configure DMA without triggering it.

        Identical to moveTo() except the pulse generator is primed via
        prepare() rather than start(). Call pulseGenerator._triggerDMA() (or
        PulseGenerator.trigger_channels()) to start the motor afterwards.

        forced_peak: peak speed in units/s to use directly, bypassing the
            natural peak computation. Used by MultiAxis for T_total sync.

        Returns the DMA channel number for bitmask construction.
        """
        if self.moving:
            raise SmartStepperError("Can't prepare move while moving")

        self._jogging = False
        self.enable()
        self._moveDeadline = None

        if not relative:
            self._target = target
        else:
            self._target = self.position + target

        if self._target > self.position:
            self._updateDirection('up')
        else:
            self._updateDirection('down')

        remaining = abs(self._target - self.position)

        if forced_peak is None and accel_time is not None:
            forced_peak = self._minSpeed + self._acceleration * accel_time
            forced_peak = min(forced_peak, self._maxSpeed)
            if forced_peak < self._minSpeed:
                raise SmartStepperError("accel_time too short: peak speed below minSpeed")

        points = self._buildProfile(self._minSpeed, remaining,
                                    triangular=triangular, forced_peak=forced_peak)
        self._pulseGenerator.prepare(points)
        return self._pulseGenerator.dma_channel

    def stop(self, emergency=False):
        """ Stop the motor.

        With emergency=False (default), decelerates smoothly from the current
        speed to minSpeed, then halts. Non-blocking.
        With emergency=True, stops immediately (hard stop, may lose steps).
        """
#         print("\nTRACE::stop()")

        if not self.moving:
            raise SmartStepperError("Can't 'stop' while not moving")

        self._jogging = False
        self._moveDeadline = None

        if emergency:
            self._pulseGenerator.stop()
        else:
            points = self._accelPoints(self.speed, self._minSpeed)
            if points:
                self._pulseGenerator.interrupt_with(points)
            else:
                self._pulseGenerator.stop()

    def _replan(self):
        """ Recompute and update the motion profile from the current state.

        Called automatically when speed or acceleration is changed mid-move.
        Non-blocking: rebuilds the profile and hands it to the pulse generator,
        which performs a fast DMA abort+restart (~10µs).
        No-op during jog (no fixed target to replan toward).
        """
        if self._jogging:
            return

        currentSpeed = self.speed
        remaining = self._target - self.position

        if abs(remaining) < 1.0 / self._stepsPerUnit:
            self._pulseGenerator.stop()  # less than 1 step left: hard stop is fine
            return

        points = self._buildProfile(currentSpeed, remaining)
        if points:
            self._pulseGenerator.update(points)
        else:
            self._pulseGenerator.stop()  # already at or past target: hard stop

    @property
    def timedOut(self):
        """ True if the current move has exceeded its timeout.

        Always False when no timeout was set.
        """
        if self._moveDeadline is None:
            return False
        return time.ticks_diff(time.ticks_ms(), self._moveDeadline) >= 0

    def waitEndOfMove(self):
        """ Block until the move completes or times out.

        Raises SmartStepperError if a timeout was set and expires.
        """
        while self.moving:
            if self.timedOut:
                self.stop(emergency=True)
                raise SmartStepperError("Move timed out")
            time.sleep_ms(1)
        self._moveDeadline = None
