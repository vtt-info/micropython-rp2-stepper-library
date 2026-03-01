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

import pulseGenerator
import pulseCounter

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

    def _accelPoints(self, fromSpeed, toSpeed):
        """ Compute acceleration (or deceleration) points from fromSpeed to toSpeed.

        For deceleration (fromSpeed > toSpeed), generates the reversed acceleration
        profile, which is valid because the smoothstep curves are point-symmetric.
        Floating-point overshoot is trimmed rather than raised.
        """
        if fromSpeed > toSpeed:
            pts = self._accelPoints(toSpeed, fromSpeed)
            pts.reverse()
            return pts

        points = []

        accelTime = (toSpeed - fromSpeed) / self._acceleration
        accelDist = (toSpeed**2 - fromSpeed**2) / (2 * self._acceleration)
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

    def _buildProfile(self, fromSpeed, remaining):
        """ Build a complete motion profile from fromSpeed over remaining distance.

        Computes the achievable peak speed given the distance, then produces
        accel → [const speed] → decel segments.

        Returns a list of (freq_steps_per_s, nbPulses) tuples for the pulse generator.
        """
        if abs(remaining) < 1.0 / self._stepsPerUnit:
            return []

        # Peak speed achievable given remaining distance starting at fromSpeed.
        # Derived from: accelUpDist + decelDist = |remaining|
        #   (peak²-fromSpeed²)/(2a) + (peak²-minSpeed²)/(2a) = |remaining|
        peak = math.sqrt(
            (abs(remaining) * 2 * self._acceleration + fromSpeed**2 + self._minSpeed**2) / 2
        )
        maxSpeed = min(peak, self._maxSpeed)

        points = []

        # Accel phase: fromSpeed → maxSpeed
        if fromSpeed < maxSpeed - 1e-6:
            points.extend(self._accelPoints(fromSpeed, maxSpeed))

        # Const speed phase (only when we hit the maxSpeed ceiling and distance allows)
        if maxSpeed >= self._maxSpeed - 1e-6:
            accelUpDist = (maxSpeed**2 - fromSpeed**2) / (2 * self._acceleration)
            decelDist   = (maxSpeed**2 - self._minSpeed**2) / (2 * self._acceleration)
            constDist   = abs(remaining) - accelUpDist - decelDist
            if constDist > 0:
                points.append((maxSpeed * self._stepsPerUnit,
                               round(constDist * self._stepsPerUnit)))

        # Decel phase: maxSpeed → minSpeed
        points.extend(self._accelPoints(maxSpeed, self._minSpeed))

        return points

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
        return self.speed != 0

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

    def moveTo(self, target, relative=False, timeout=None):
        """ Move to target

        target is in units.
        timeout is in seconds (None = no timeout).
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
        points = self._buildProfile(self._minSpeed, remaining)

        # Start the generator
        self._pulseGenerator.start(points)

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
                self._pulseGenerator.update(points)
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


def jog(stepper):
    """
    """
    print("\nINFO::Testing 'jog'...")

    stepper.position = 0
    stepper.jog(50)
    t0 = time.ticks_ms()

    t = time.ticks_ms()
    while (time.ticks_ms() - t < 1500):
#         print("time={:6.3f}s speed={:+6.1f}mm/s pos={:+7.1f}mm".format((time.ticks_ms()-t0)/1000, stepper.speed, stepper.position))
        time.sleep(0.1)

    stepper.stop()

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def moveTo(stepper):
    """
    """
    print("\nINFO::Testing 'moveto'...")

    stepper.moveTo(150)
    t0 = time.ticks_ms()

#     while stepper.moving:
    t = time.ticks_ms()
    while (time.ticks_ms() - t < 1500):
        time.sleep(0.1)

    stepper.stop()

    while stepper.moving:
        time.sleep(0.1)

    stepper.moveTo(50)

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def moveToRel(stepper):
    """
    """
    print("\nINFO::Testing relative 'moveto'...")

    t = time.ticks_ms()
    stepper.position = 0
    stepper.moveTo(80, relative=True)

    while stepper.moving:
        time.sleep(0.1)

    stepper.moveTo(-50, relative=True)

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def debug(stepper):
    """
    """
    t = time.ticks_ms()
    stepper.moveTo(50)
    print("\nINFO::starting with target: {:.1f}mm / {:d}steps".format(stepper.target, round(stepper.target*stepper.stepsPerUnit)))

    while not stepper.moving:
        time.sleep_ms(1)
    print("INFO::Moving...")

#     print("  time  speed     pos")
    while stepper.moving:
#         print("{:6.3f} {:+6.1f} {:+7.1f}".format((time.ticks_ms()-t)/1000, stepper.speed, stepper.position))
#
#         if time.ticks_ms()-t >= 5000:
#             stepper.stop()
#             while stepper.moving:
#                 print("{:6.3f} {:+6.1f} {:+7.1f}".format((time.ticks_ms()-t)/1000, stepper.speed, stepper.position))

        time.sleep_ms(1)

    print("\nINFO::done: pulse counter: {:.1f}mm / {:d}steps".format(stepper.position, stepper._pulseCounter.value))

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def main():
    """
    """
    print("TRACE::main()")
    stepper = SmartStepper(27, 26, accelCurve='smooth2')
    stepper.stepsPerUnit = 96.
    stepper.minSpeed = 1
    stepper.maxSpeed = 50
    stepper.acceleration = 300

    jog(stepper)
    moveTo(stepper)
    moveToRel(stepper)
    debug(stepper)


if __name__ == "__main__":
    main()
