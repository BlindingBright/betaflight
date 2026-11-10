> [!IMPORTANT]
> **This is a testing fork for 3D (bidirectional / reversible motor) mode.**
> It exists to try out and flight-test changes and improvements to Betaflight's 3D mode,
> with the hope that the ones that prove themselves get proposed for the main
> [betaflight/betaflight](https://github.com/betaflight/betaflight) branch.
> It is experimental: use the official releases for normal flying.
>
> **Pilots: download, testing guide and log upload are on the project page:
> [BetaFlight 3D Edition](https://github.com/BlindingBright/Drones/tree/main/BetaFlight%203D%20Edition)**
> (ready-to-flash boards on the [v0.3.2-test release](https://github.com/BlindingBright/betaflight/releases/tag/v0.3.2-3d-test); v0.3.3 hexes follow).
>
> Base: **Betaflight 2026.6.2** (`e0b7bb01b`, latest stable). The previous 4.5.1-based v0.3.1-test stays on branch `3d-iterm-memory`. Test craft: a 3D whoop on a TBS Lucid AIO (AT32F435) with AM32 ESCs.
> Every change is flight-tested against stock behaviour with blackbox logs before it is considered.
>
> **Current version: v0.3.3-test** (branch `3d-v0.3.3`, tag `v0.3.3-3d-test`): v0.3.2 (the v0.3.1 3D changes on Betaflight 2026.6.2)
> plus two 3D fixes. No new settings, saved settings are kept. Flown on a 6S 5-inch (Hobbywing XRotor F7, AM32): both fixes confirmed
> in blackbox logs (frozen I-term unchanged through every reversal hold; reverse motor outputs follow the corrected bounds).
> - with `iterm_rotation = ON`, stock kept rotating the I-term while it is frozen after a motor reversal, so the I-term loaded for the
>   new direction moved between axes during the roll-out (e.g. yaw I into roll). Frozen now means frozen; stock rotation resumes at release.
> - in reverse the mixer applies the mix negated, but airmode / the throttle constraint used the bounds of the un-negated mix (every
>   mixer type), so with roll + pitch + yaw together a motor could clip or full reverse throttle was pulled down too far. Same code in stock
>   Betaflight. Upright and non-3D flight are bit-identical.
>
> The 3D settings default to ZoeFPV's best flying setup (Pass 53/56); set the stock values from the table to get stock Betaflight 3D behaviour.
>
> | Setting | Default (stock) | Values | What it does |
> |---|---|---|---|
> | `3d_iterm_mode` | `PER_DIRECTION` (`RESET`) | `RESET` / `HOLD` / `PER_DIRECTION` | What happens to I-term when the motors reverse. `PER_DIRECTION` keeps a learned I-term per motor direction and restores it on reversal, so a direction-dependent trim isn't thrown away on every flip. `HOLD` / `PER_DIRECTION` freeze integration while the props cross zero rpm. |
> | `3d_iterm_release` | `RPM` (`TIME`) | `TIME` / `RPM` | End the post-reversal hold after `3d_iterm_reset_ms` (stock: hard-coded 250 ms), or as soon as every motor's bidirectional-DShot eRPM has dipped below and recovered above `3d_iterm_rpm_pct` % of the in-flight average, within `3d_iterm_min_ms`…`3d_iterm_max_ms` (50…400). Implausible eRPM jumps are filtered. Without RPM telemetry, `RPM` falls back to `3d_iterm_reset_ms`. |
> | `3d_iterm_load` | `FLIP` | `FLIP` / `RELEASE` | Load the learned I-term at the reversal or when the hold ends. |
> | `3d_iterm_learn_thr`, `3d_iterm_learn_hz_x10` | 50 %, 0.5 Hz | | Learning only in steady, unsaturated flight above 50 % of the quad's own average throttle, PT1 at 0.5 Hz. Memory persists across disarms. |
> | `3d_iterm_axes` | `ALL` | `ALL` / `PITCH` / `ROLL_PITCH` | Axes that use the memory. |
> | `3d_anti_gravity` | `GATED` (`STOCK`) | `STOCK` / `GATED` | `GATED`: the anti-gravity filter is reset at each reversal and ignores the throttle change while I-term is held, so a pass through center isn't a throttle chop. |
> | `3d_ag_approach_pct` | 15 (0 = off) | 0 … 50 | With `GATED`, also ignore the throttle falling below this % on the way into a reversal. |
> | `3d_dyn_idle` | `ON` (`OFF`) | `OFF` / `ON` | Runs the dynamic idle controller in 3D (stock skips it) and raises the idle of the current motor direction (DShot); no 1 % zero-throttle offset; I falls fast once every motor is >25 % above `dyn_idle_min_rpm`. Only active with bidirectional DShot and `dyn_idle_min_rpm` > 0 (stock default 0; 50 on 1S/2S whoops). |
> | `debug_mode = 3D_ITERM` | | | [0] motor direction, [1] ms since reversal while holding (0 once released), [2-4] upright memory x10, [5-7] inverted memory x10 |
>
> **3D fixes (always on in normal 3D; no settings):** stock Betaflight treats the bottom of the stick as "off", which in 3D is full
> reverse. v0.3.1 fixes the places where that leaks into 3D:
> - low-voltage cutoff (`vbat_cutoff_percent` < 100) scales the throttle toward stick center, not toward full reverse;
> - failsafe stage 2 never commands reverse thrust (`failsafe_throttle` ≤ 1500 → zero thrust; above → upright thrust);
> - `thr_mid` / `thr_hover` (2026.6 throttle curve) no longer move zero thrust off stick center;
> - `vbat_sag_compensation` works in 3D (stock skips it), as a fraction of the active direction's band;
> - (new in v0.3.2) when the autopilot sets the throttle (altitude hold, which 2026.6 also engages for failsafe landing, or GPS
>   rescue), the motors run upright: stock kept the last direction, so after a link loss in reverse it drove them in reverse.
>
> With the 3D mode switch (`3D DISABLE / SWITCH`) active or `3d_switched_mode = ON`, stock behaviour applies.
>
> The blackbox header records `3d_deadband_throttle` and all `3d_*` settings. Unit tests: `src/test/unit/mixer_3d_unittest.cc`
> (real mixer + DShot code, both motor directions), `rc_3d_unittest` and `pid_unittest`.
>
> 2026.6 renames: the D boost is now `d` (base) + `d_max_*` (max) with `d_max_gain` / `d_max_advance` (4.5: `d_min_*` / `d`), and
> `dshot_idle_value` is `motor_idle`, so a 4.5 `diff all` doesn't paste as-is.
>
> Build: `make CONFIG=<board>` with the [betaflight/config](https://github.com/betaflight/config) submodule (`src/config`, commit `3a406c5`).

![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo.svg#gh-light-mode-only)
![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo_dark.svg#gh-dark-mode-only)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build](https://img.shields.io/github/actions/workflow/status/betaflight/betaflight/push.yml?branch=master)](https://github.com/betaflight/betaflight/actions/workflows/push.yml) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Join us on Discord!](https://img.shields.io/discord/868013470023548938)](https://discord.gg/n4E6ak4u3c)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft. Betaflight focuses on flight performance, leading-edge feature additions, and wide target support.

## Release Schedule

| Date       | Release | Stage             | Status    |
| ---------- | ------- | ----------------- | --------- |
| 26-12-2025 | 2025.12 | Release           | Completed |
| 01-04-2026 | 2026.6  | Beta              |           |
| 01-05-2026 | 2026.6  | Release Candidate |           |
| 01-06-2026 | 2026.6  | Release           |           |
| 01-10-2026 | 2026.12 | Beta              |           |
| 01-11-2026 | 2026.12 | Release Candidate |           |
| 01-12-2026 | 2026.12 | Release           |           |

## News

### 📣 Announcement: New Versioning Scheme & Release Cadence 📣

To create a more predictable release schedule, we're moving to a new versioning system and development cycle, starting with the next release.

**New Format**: `YYYY.M.PATCH` (e.g., `2025.12.1`)

**Release Cadence**: Two major releases per year.

**Target Months**: June and December.

This means the successor to our current `4.x` series will be Betaflight `2025.12.x`, followed by Betaflight `2026.6.x`. We will also align the Betaflight App and Firmware to the same `YYYY.M.PATCH` releases (and cadence).

**Our New Release Cycle**

To support this schedule, our development phases will be structured as follows:

**Alpha**: For new feature development. Alpha builds for the next version will be available shortly after a stable release is published.

**Beta**: A one-month feature freeze for bug fixes only, and existing pull requests currently being reviewed, starting approximately two months before a release.

**Release Candidate (RC)**: A one-month period for final stabilization and testing before the official release.

### Requirements for the submission of new and updated target configuration

The requirements for pull requests adding new targets or modifying existing targets are available on the [betaflight.com website](https://www.betaflight.com/docs/development/manufacturer/requirements-for-submission-of-targets).

## Features

Betaflight has the following features:

- Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
- DShot (150, 300 and 600), Multishot, Oneshot (125 and 42) and Proshot1000 motor protocol support
- Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
- Support for targets that use the STM32 F4, G4, F7 and H7 processors
- PWM, PPM, SPI, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
- Multiple telemetry protocols (CRSF, FrSky, HoTT smart-port, MSP, etc)
- RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
- OSD support & configuration without needing third-party OSD software/firmware/comm devices
- OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
- In-flight manual PID tuning and rate adjustment
- PID and filter tuning using sliders
- Rate profiles and in-flight selection of them
- Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
- VTX support for Unify Pro and IRC Tramp
- and MUCH, MUCH more.

## Installation & Documentation

See: https://betaflight.com/docs/wiki

## Support and Developers Channel

There's a dedicated [Discord server](https://discord.gg/n4E6ak4u3c) for help, support and general community.

## Betaflight Application

To configure Betaflight you should use the [Betaflight App](https://app.betaflight.com). It is a progressive web app, so should always be the latest version.

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

- implement a new feature in the firmware or in the app (see [below](#developers));
- documentation updates and corrections;
- How-To guides - received help? Help others!
- bug reporting & fixes;
- new feature ideas & suggestions;
- provide a new translation for the app, or help us maintain the existing ones (see [below](#translators)).

The best place to start is the Betaflight Discord (registration [here](https://discord.gg/n4E6ak4u3c)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](https://www.betaflight.com/docs/development) and [coding style definition](https://www.betaflight.com/docs/development/CodingStyle).

GitHub actions are used to run automatic builds.

### Building with Docker/Devcontainers

A preconfigured [devcontainer](.devcontainer/README.md) is included for a consistent build environment across all platforms. This is the recommended approach for Windows developers:

```bash
# With VS Code: Install "Dev Containers" extension, open folder, and select "Reopen in Container"

# Or command-line only:
docker build -t betaflight-dev -f .devcontainer/containerfile .devcontainer/
docker run --rm -v "${PWD}:/workspace" -w /workspace betaflight-dev make TARGET=SPEEDYBEEF405WING
```

See the [devcontainer documentation](.devcontainer/README.md) for detailed setup instructions including hardware flashing.

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 21 languages for Betaflight Configurator: Català, Dansk, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文, 繁體中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:

- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Discord chat (registration [here](https://discord.gg/n4E6ak4u3c)), and join the ['translation'](https://discord.com/channels/868013470023548938/1057773726915100702) channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

Betaflight does not manufacture or distribute their own hardware. While we are collaborating with and supported by a number of manufacturers, we do not do any kind of hardware support.

If you encounter any hardware issues with your flight controller or another component, please contact the manufacturer or supplier of your hardware, or check [Discord](https://discord.gg/n4E6ak4u3c) to see if others with the same problem have found a solution.

## Betaflight Releases

You can find our release [here](https://github.com/betaflight/betaflight/releases) on Github and we also have more detailed [release notes](https://www.betaflight.com/docs/category/release-notes) at [betaflight.com](https://www.betaflight.com).

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

For a complete list of contributors (past and present) see [Github](https://github.com/betaflight/betaflight/graphs/contributors).
