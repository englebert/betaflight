# BAROLAND

A pilot-invoked controlled descent on the barometer alone, ending in a disarm.
Added for the LittleBee 1.0 build on a JHEMCU GF30F722-ICM (`JHEF7DUAL`,
STM32F722), where a GPS fix is not always available.

## Why

Betaflight can already fly a baro-only descent, but only as a *failsafe*
reaction — `failsafe_procedure = AUTO-LAND` engages it when the RX link drops.
There was no way for the pilot to ask for the same descent deliberately. The
GPS RESCUE switch is not a substitute: it runs the rescue controller, which
disarms outright if it starts below `ap_landing_altitude_m` with no valid
distance-to-home (`gps_rescue_multirotor.c:346`), and which explicitly disables
alt hold and pos hold while it owns the craft.

BAROLAND is that missing switch.

## What it does

The descent itself is not new code. It reuses the failsafe auto-landing path
verbatim, so the behaviour in the air is the behaviour already flown today:

- **Descent profile** — BAROLAND's own, in absolute cm/s, set in
  `baroLandDescentRateCmS()`. Flat `baroland_descent_rate` at and below
  `baroland_flare_altitude`, ramping linearly with height above it to
  `baroland_max_descent_rate` ten flare-heights up. The throttle stick is
  ignored throughout.

  It does *not* reuse the failsafe curve, for two reasons. That curve has no
  flare: below 2 m it clamps to a flat 1x `alt_hold_climb_rate` and the
  altitude target marches below zero regardless, so whatever rate is commanded
  near the ground is the rate it touches down at. And tying the landing rate to
  `alt_hold_climb_rate` means softening the touchdown also cripples the pilot's
  throttle-stick authority in ordinary alt hold.

  | Height | Rate on defaults |
  | --- | --- |
  | at or below 1.5 m | 0.40 m/s |
  | 3 m | 0.63 m/s |
  | 5 m | 0.94 m/s |
  | 10 m | 1.72 m/s |
  | 15 m and above | 2.50 m/s |
- **Position** — pos hold engages alongside alt hold, as it does under
  failsafe. With a valid XY estimate the craft holds its spot; with no GPS fix
  `updatePosHold()` zeroes its output and the craft descends level, drifting
  with the wind. Roll and pitch sticks still nudge it within
  `pos_hold_deadband`, so the touchdown spot can be shifted — unlike failsafe,
  the pilot has a live link here.
- **Attitude** — ANGLE mode is forced, as for any autopilot mode.

What is new is how it ends. A failsafe landing disarms on a timer; a mode the
pilot can hold indefinitely needs to recognise the ground:

```
isAltHoldActive()                                  the controller owns throttle
  AND  below ap_landing_altitude_m                 (4 m default)
  AND  |vertical velocity| <= baroland_touchdown_velocity
  AND  autopilot throttle < midway(ap_throttle_min, hover)
  held for baroland_touchdown_time                 -> disarm(LANDING)
```

All four must hold together, because each has a benign in-flight explanation on
its own — low altitude on a low pass, a momentarily zero vertical speed at the
top of a baro wobble, low throttle in a dive. The ground is what stops a
commanded descent, so a descent that has stalled at low throttle near the
ground is the inference. A 2 s settling window after engaging suppresses the
test entirely while the altitude controller winds its integrator down.

`baroland_timeout` is the backstop, in the same spirit as
`failsafe_landing_time`: the detector needs a working baro to fire at all, so
the mode cannot hold the craft in a descent it will never finish.

## Enabling it

Compiled in by the board config, and needs no GPS:

```c
#define USE_ALTITUDE_HOLD   // BAROLAND flies its descent through alt hold
#define USE_BARO_LAND
```

`common_post.h` drops `USE_BARO_LAND` on a build without `USE_ALTITUDE_HOLD` or
on a wing, rather than failing to link, so the opt-in is safe to leave in a
shared config. Cost on the LittleBee build is about 700 bytes of flash.

Then assign an aux switch to **BAROLAND** in Configurator's Modes tab.

## Settings

| Setting | Default | Range | Meaning |
| --- | --- | --- | --- |
| `baroland_descent_rate` | 40 | 10-200 | cm/s; the final approach rate, flown through touchdown. |
| `baroland_max_descent_rate` | 250 | 50-1000 | cm/s; the rate the profile tops out at when high. |
| `baroland_flare_altitude` | 150 | 20-1000 | cm; at and below this height the descent holds `baroland_descent_rate`. |
| `baroland_timeout` | 60 | 0-250 | Seconds in the mode before the backstop disarm. 0 disables it. |
| `baroland_touchdown_time` | 10 | 1-50 | Tenths of a second the touchdown conditions must hold. |
| `baroland_touchdown_velocity` | 50 | 10-200 | cm/s; vertical speed at or below this counts as "no longer descending". |

The descent rate is BAROLAND's own and independent of `alt_hold_climb_rate`,
so that can stay at whatever suits the pilot's throttle stick. How well the
craft *tracks* the commanded rate still comes from the shared autopilot gains
(`ap_altitude_*`, `ap_hover_throttle`). `ap_hover_throttle` is the single
biggest knob for descent quality; set it to the craft's real hover throttle.

Note that no rangefinder means no ground-effect compensation
(`position_estimator.c:897` returns 1.0 without one), so the barometer is least
trustworthy in exactly the last half metre. A slow `baroland_descent_rate` is
what keeps that from mattering much.

## Behaviour notes

- **Arming** — `ARMING_DISABLED_ALTHOLD` is asserted while the switch is on, so
  the craft cannot be armed into a landing, and cannot re-arm after a BAROLAND
  disarm until the switch is released.
- **Priority** — failsafe and GPS rescue both outrank it. If either engages
  mid-descent, BAROLAND drops out and they take over.
- **Exiting** — release the switch at any point before touchdown and normal
  control returns immediately, throttle stick live.
- **Baro loss** — if `isAltitudeAvailable()` goes false mid-descent the mode
  disengages and alt hold drops with it, which hands the throttle stick back
  abruptly wherever it happens to be sitting. Worth knowing before relying on
  the mode over a shared I2C bus.
- **OSD** — annunciates `LAND` (ahead of `POSH`/`ALTH`, both of which are on
  underneath it). Not visible on the LittleBee build, which sets `EXCLUDE_OSD`.

## Files

| File | Role |
| --- | --- |
| `src/main/flight/baro_land.c` / `.h` | Mode state, touchdown detector, disarm |
| `src/main/pg/baro_land.c` / `.h` | `baroLandConfig` parameter group |
| `src/main/fc/rc_modes.h` | `BOXBAROLAND` box id |
| `src/main/fc/runtime_config.h` | `BARO_LAND_MODE` flight mode flag, box map |
| `src/main/msp/msp_box.c` | Box name `BAROLAND`, permanent id 58 |
| `src/main/fc/core.c` | Mode engage, arming gate, alt/pos hold coupling |
| `src/main/flight/alt_hold_multirotor.c` | Descent profile hook |
| `src/main/osd/osd_elements.c` | `LAND` annunciation |
