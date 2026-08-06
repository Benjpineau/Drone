# Drone


## Why this project?

1. I wanted to work on a system that I find far more interesting and rewarding than the typical robotic arm project.
2. As cliché as it may sound, building a flying vehicle brings a unique sense of satisfaction—*"conquering the skies."*
3. I've always wanted to fly a drone.

## Project Overview

Beyond being a fun personal challenge, this project is also an opportunity to apply my existing skills to a real-world system while developing new ones.

To achieve this, I decided to complete the following tasks.

### What I wanted to build myself

* Model the drone dynamics.
* Design and build the drone frame.
* Design my own Flight Controller PCB.
* Develop the flight controller (PID-based control algorithms).
* Develop the embedded software (using the STM32 ecosystem).

### What I decided **not** to build

* Power Distribution Board (PDB)
* Electronic Speed Controllers (ESCs)
* Radio antenna
* Brushless motors
* Propellers

The components above were intentionally purchased rather than designed from scratch for both financial and time-related reasons. For example, a mass-produced PDB costs roughly five times less than manufacturing a custom one. Since the Flight Controller is essentially the brain of the drone, I considered it much more worthwhile to design it myself.

## Progress

* Drone dynamics modeling ✔
* Drone frame design and manufacturing ✔
* Flight Controller PCB ✔
* Flight control algorithms (PID controller) ✔
* Embedded software (STM32) ❌ *(only a few features remain: emergency stop, mixer, and Mahony filter)*
* Power Distribution Board (PDB) ✔ *(purchased)*
* Electronic Speed Controllers (ESCs) ✔ *(purchased)*
* Radio antenna ✔ *(purchased)*

At the moment, the controller has been fully designed and implemented from a theoretical perspective. The remaining step is tuning the controller gains. I'm currently waiting to replace one faulty motor before starting the flight tests.

## Repository Contents

* **Theory, derivations, explanations, and illustrations:** `drone_eng.pdf`
* **Source code:** `FlightController`
* **Gerber files and CAD models:** available upon request at **benjamin@[93160@gmail.com](mailto:93160@gmail.com)**

## Keywords

* **General:** Quadrotor, CAD (Fusion 360), PCB (EasyEDA), STM32 (CubeMX & STM32CubeIDE), ESP32 (Arduino), C, Cascaded Control
* **Embedded:** SPI, I²C, DMA, UART, USB
