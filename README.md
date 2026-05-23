# PAMOSE-01

Portable Avionics Mistake  
Orbital Scooter Equipment

---

## Overview

PAMOSE-01 is a portable field electronics and mission support platform designed for RF observation, telemetry handling, environmental sensing and mobile operations.

The project combines radio monitoring, onboard sensing, telemetry tools and an interactive mission interface into a single expandable system.

Originally developed as a ground station platform, PAMOSE evolved into a modular engineering console intended for experimentation, field deployment and systems learning.

---

## Current Features

### RF Operations

- 2.4 GHz scan mode
- Channel activity observation
- Channel lock mode
- Packet transmission mode
- Telemetry reception

### Sensing

- Orientation sensing
  - Roll
  - Pitch
  - Yaw
  - Acceleration

- Environmental sensing
  - Temperature
  - Pressure
  - Humidity
  - Barometric altitude

### User Interface

- Mobile web interface
- Galaxy Ace display support
- Joystick navigation
- LCD feedback
- STELLAR navigation system

### Mission Modes

SCAN

LOCK

TX

ORNT

SENS

STELLAR

---

## Hardware

Primary controller:

ESP32

Sensors:

MPU6500

BME280

Radio:

nRF24L01+ PA+LNA

User input:

Dual joysticks

Display:

Galaxy Ace interface

LCD display

---

## Planned Expansion

PEWO

Portable Electromagnetic Wildlife Observatory

Planned capabilities:

- Passive WiFi observation
- RF ecology visualization
- Device mapping
- Historical observation logging

Additional roadmap items:

- Internal display upgrade
- Multi-band support
- Data logging
- External programming support

---

## Project Structure

01_REQUIREMENTS

02_ARCH

03_HARDWARE

04_SOFTWARE

05_ANALYSIS

06_TESTING

07_FAILURE

08_MEDIA

09_REFERENCE

ARCHIVE

---

## Documentation Philosophy

PAMOSE follows a systems engineering workflow.

Documentation priorities:

- Traceability
- Failure capture
- Design reasoning
- Testing records
- Knowledge retention

If the project is lost, documentation should permit reconstruction without internet access.

---

## Status

REV A

Operational

Current state:

RF system: Working

Telemetry: Working

Sensors: Working

STELLAR: Working

PEWO: In development

---
