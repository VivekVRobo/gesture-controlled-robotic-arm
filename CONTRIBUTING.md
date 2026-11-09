# Contributing

Thanks for considering a contribution to the Gesture-Controlled Robotic Arm project.

This repository is intended to stay reproducible and evidence-based. Contributions are welcome in firmware, documentation, testing, benchmarking, wiring documentation, and hardware validation.

## Good contribution areas

- Arduino compile/CI checks for the transmitter and receiver sketches
- safer receiver timeout and fail-safe behavior
- reproducible nRF24L01 packet-loss and latency measurement
- wiring/documentation improvements
- servo calibration and safe-limit documentation
- test coverage for packet parsing, mapping, and kinematics
- clearly documented hardware reproduction reports

## Local Firmware Compilation Verification (`arduino-cli`)

Before submitting firmware modifications, verify that both sketches compile cleanly using `arduino-cli`:

1. **Install Arduino CLI** and initialize board indices:
   ```bash
   arduino-cli core update-index
   arduino-cli core install arduino:avr@1.8.6
   ```

2. **Install Pinned Dependencies**:
   ```bash
   arduino-cli lib update-index
   arduino-cli lib install "RF24@1.4.9"
   arduino-cli lib install "Adafruit PWM Servo Driver Library@3.0.2"
   ```

3. **Compile Transmitter (Arduino Nano)**:
   ```bash
   arduino-cli compile --fqbn arduino:avr:nano src/transmitter
   ```

4. **Compile Hardened Receiver (Arduino Uno)**:
   ```bash
   arduino-cli compile --fqbn arduino:avr:uno src/receiver
   ```

## Before opening a pull request

1. Open or reference an issue describing the change.
2. Keep the change focused.
3. Document the exact board/library/tool versions used.
4. Do not turn an estimate into a measured hardware claim without preserving the measurement method and evidence.
5. If the change affects actuation, describe the safe test procedure.

## Pull request checklist

- [ ] The change has a clear technical purpose.
- [ ] Build/test steps are documented.
- [ ] Existing behavior is not silently changed.
- [ ] New performance or hardware claims include reproducible evidence.
- [ ] Documentation reflects the implementation.
- [ ] Safety-relevant behavior is called out explicitly.

## First-time contributors

Look for issues labeled `good first issue` or `help wanted`. Documentation, reproducibility, CI, and test improvements are valid engineering contributions and are encouraged.

## Evidence standard

Physical measurements should state:

- hardware revision and components
- firmware/commit SHA
- supply conditions
- test environment
- measurement method
- raw observations or logs
- number of trials when relevant

Simulation or analytical results should remain labeled as such.

## Questions

If you are unsure whether an idea fits the project, open an issue before implementing a large change.
