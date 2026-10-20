# Repository Guidelines

## Project Structure & Module Organization

This workspace is a collection of related repositories and hardware artifacts:

- `ARDUINO_PIPELINE_FIRMWARE/` contains the current Arduino firmware; `ARDUINO_COMBINED_CHECK/` contains bench diagnostics.
- `opensurgbot_pipeline/` provides the Python hardware/control API, while `opensurgbot_SurRoL/` integrates it with SurRoL.
- `opensurgbot_kinevizu/` contains the kinematic visualization and study materials.
- `SurRoL/` contains simulation, training, benchmark, and haptic-guidance code.
- `web/` is the static GitHub Pages site (`index.html`, `scripts.js`, `styles.css`, and `assets/`).
- `archives/` preserves historical PCB and 3D-print designs. Treat archived files as reference material, not current build inputs.

Software components are tracked as ordinary folders in this repository. Run component-specific build and test commands from the relevant component directory.

## Build, Test, and Development Commands

Arduino sketches are opened, compiled, and uploaded with Arduino IDE; follow the wiring and startup guidance in `ARDUINO_FULL_PIPELINE_README.md`. For Python components, create a Python 3.7+ virtual environment and install that component’s documented dependencies, then run its entry point, for example:

```bash
python opensurgbot_kinevizu/kinevisu/kinevisu.py
```

SurRoL experiments and tests are component-specific. From the applicable `SurRoL/` subproject, use its `setup.py`, `requirements.txt`, or README instructions. The web site is static; preview it with any local HTTP server, such as `python -m http.server` from `web/`.

## Coding Style & Naming Conventions

Use four spaces for Python indentation and clear `snake_case` names for Python modules, functions, and variables. Keep Arduino sketches readable with descriptive `UPPER_SNAKE_CASE` constants and comments around pin assignments, serial protocols, and safety limits. Preserve existing formatting in HTML, CSS, and JavaScript. Use ASCII, descriptive filenames where practical; retain established hardware names and archival filenames when changing those assets.

## Testing Guidelines

Add or update tests beside the component they exercise, following existing `test_*.py` naming. Run the relevant component’s test command (commonly `pytest`) from that component’s directory. For firmware, compile the sketch and perform the documented bench checks before connecting loads; verify serial commands and safe motion limits.

## Commit & Pull Request Guidelines

Component histories favor short, imperative summaries such as `Updated README.md` and `Fixed lib name`. Keep commits focused on one component or concern. Pull requests should explain the affected component, hardware or simulation impact, setup and validation commands, and any wiring or safety implications. Include screenshots for web or visualization changes and link related issues when available.

## Security & Configuration Tips

Never commit serial-port credentials, private keys, machine-local paths, or generated experiment outputs. Review power, grounding, current limits, and motion bounds before testing firmware on connected hardware.
