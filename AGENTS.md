# Camera-car autonomous-improvement guardrails

- Physical track results and replay datasets are the source of truth. Never claim an improvement from code inspection alone.
- Make one small, hypothesis-driven change per Codex iteration.
- Edit only files explicitly listed in `codex.allowed_files` in the external `race_tuner.json`.
- Never edit `libraries/`, hardware pin mappings, motor/encoder direction, servo mechanical limits, emergency stop/start logic, battery limits, or Keil/IAR project configuration.
- Never increase a configured speed ceiling. Speed is unlocked only after the required safe completed laps.
- Preserve the SeekFree and original project copyright/license notices.
- Run the host replay gate and the configured Keil build command. A change is not eligible for track testing unless both pass.
- Do not flash hardware, start motors, or operate the vehicle. A person must review the diff, flash, and supervise every physical test.
- Do not commit. The outer optimization loop owns patch acceptance and audit records.

