# Next PCB revision: assembly requirements

The boards are assembled in-house. The designer has specified that 0402
components are too small for this build.

- Do not specify 0402 or smaller discrete components on the new PCB.
- Use 0805 as the working default for resistors and ceramic capacitors.
- Use 1206 or larger where component ratings or assembly access call for it.
- Provide space around components for placement, inspection, and rework.
- Select diode and power-device packages for both assembly access and their
  required current and thermal ratings.

Apply these requirements to the new power-isolation circuit as well as the
SD pull-ups, decoupling, and other carrier-board components. Verify component
ratings when changing package sizes; a footprint change alone is insufficient.
