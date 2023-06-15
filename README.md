# Micropython lib

My collection of Micropython libs, based on existing libs, or entirely written from scratch.

(C) 2023 Frédéric Mantegazza

## General libs

### smartStepper

Stepper controller. Handle acceleration in both goto and jog modes.

### smartButton

Manage debouncing, click, double-click, long press...

### pidController

Simple PID controller.

### response

Time-based curve generator. Mainly used by the smartStepper lib to generate acceleration curve.

### servo

Simple servo driving lib.

### signal

### umenu2

µ-menu library based on xxx's umenu lib.

### uGEM

Micropython port of Arduino GEM lib by ???.

_Work in progress_

### baos

BAOS KNX ??? board handling. Based on original C code from ???.

### sht15

SHT15 sensor reader.

## RaspberryPi Pico dedicated libs

These libs require a RPi Pico as they use PIOs.

### ssi

SSI encoder reading.

## pyboard dedicated libs

These libs require a pyboard (v1.0 or v1.1) because they use the original LCD/touch shield.
