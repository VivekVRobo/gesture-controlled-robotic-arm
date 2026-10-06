# v0.1.0 Release Readiness

This checklist defines the minimum bar for the first tagged release of the gesture-controlled robotic arm.

The first release may present the project as a **physical hardware demo/reference implementation** because the repository already contains a continuous actuation video and hardware bench photography. It must not invent quantitative latency, repeatability, payload or reliability measurements that have not been collected.

## Required before tagging

- [ ] The exact release commit contains the current physical actuation video and hardware bench evidence.
- [ ] The transmitter and receiver firmware build/validation path is documented and reproducible.
- [ ] README links to the physical demo and bench image resolve from GitHub.
- [ ] `CONTRIBUTING.md` and issue templates remain present.
- [ ] Mechanical angle clamping and servo interpolation behavior remain documented.
- [ ] The dedicated power-rail / common-ground wiring guidance remains visible.
- [ ] No unmeasured latency value is promoted as measured evidence.
- [ ] `LICENSE` is present and accurate.
- [ ] Release notes state exactly which physical behaviors are directly visible in the committed demonstration.

## Allowed v0.1.0 claims

The release may claim:

- a real multi-joint robotic arm controlled from a wearable MPU6050 gesture glove;
- nRF24L01 wireless command transport;
- Arduino Nano/Uno control path;
- PCA9685 servo PWM output;
- physical actuation evidence committed in the repository;
- dedicated servo power-rail guidance;
- software angle clamping and interpolation safeguards;
- contributor-facing wiring and calibration documentation.

## Claims that remain blocked

Do not claim any of the following without new measurement evidence:

- quantified end-to-end control latency;
- packet-loss rate under defined RF conditions;
- servo positional accuracy/repeatability;
- payload capacity;
- endurance/reliability over extended operation;
- formal electrical or machinery safety certification.

## Promotion rule

Tag `v0.1.0` only from a clean commit satisfying this checklist. New quantitative hardware claims should be added in later releases only when the corresponding measurement protocol and raw evidence are committed.