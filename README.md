[![PathPlanner](https://github.com/mjansen4857/pathplanner/actions/workflows/pathplanner-ci.yaml/badge.svg)](https://github.com/mjansen4857/pathplanner/actions/workflows/pathplanner-ci.yaml)
[![codecov](https://codecov.io/gh/mjansen4857/pathplanner/branch/main/graph/badge.svg?token=RRJY4YR69W)](https://codecov.io/gh/mjansen4857/pathplanner)
[![PathPlannerLib](https://github.com/mjansen4857/pathplanner/actions/workflows/pplib-ci.yml/badge.svg)](https://github.com/mjansen4857/pathplanner/actions/workflows/pplib-ci.yml)

# PathPlanner
<a href="https://www.microsoft.com/en-us/p/frc-pathplanner/9nqbkb5dw909?cid=storebadge&ocid=badge&rtc=1&activetab=pivot:overviewtab"><img src="https://mjansen4857.com/badges/windows.svg" height=50></a>
&nbsp;&nbsp;&nbsp;

Download from the Microsoft Store to receive auto-updates for stable releases. Manual installs and pre-releases can be found [here](https://github.com/mjansen4857/pathplanner/releases).

## About
![PathPlanner](https://github.com/user-attachments/assets/5b87c1a4-8fdb-4eb9-bf29-71f79a826a82)


PathPlanner is a motion profile generator for FRC robots created by team 3015. The main features of PathPlanner include:
* Each path is made with Bézier curves, allowing fine tuning of the exact path shape.
* Holonomic mode supports decoupling the robot's rotation from its direction of travel.
* Real-time path preview
* Allows placing "event markers" along the path which can be used to trigger other code while path following.
* Build modular autonomous routines using other paths.
* Automatic saving and file management
* Robot-side vendor library for path generation and custom path following commands/controllers
* Full autonomous command generation with PathPlannerLib auto builder
* Real time path following telemetry
* Hot reload (paths and autos can be updated and regenerated on the robot without redeploying code)
* Automatic pathfinding in PathPlannerLib with AD*

## Usage and Documentation
### [pathplanner.dev](https://pathplanner.dev)

<br/>

Make sure you [install PathPlannerLib](https://pathplanner.dev/pplib-getting-started.html) to generate your paths.
```
https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
```

[Java API Docs](https://pathplanner.dev/api/java/)

[C++ API Docs](https://pathplanner.dev/api/cpp/)

[Python API Docs](https://pathplanner.dev/api/python/)

## How to build manually:
* [Install Flutter](https://flutter.dev/docs/get-started/install)
* Open the project in a terminal and run the following command: `flutter build <PLATFORM>`
   * Valid platforms are:
      * windows
      * macos
      * linux
* The built app will be located here:
    * Windows: `<PROJECT DIR>/build/windows/runner/Release`
    * macOS: `<PROJECT DIR>/build/macos/Build/Products/Release`
    * Linux: `<PROJECT DIR>/build/linux/x64/release/bundle`
* OR `flutter run` to run in debug mode

---

## Ghost Auto Overlay System (Fork Addition)

This fork adds a **Ghost Auto Overlay System** that does not exist in the upstream [mjansen4857/pathplanner](https://github.com/mjansen4857/pathplanner) repository. It lets you visualize other robots' autonomous routines as translucent reference overlays while editing your own paths — useful for alliance coordination and avoiding on-field collisions.

### Features

1. **Ghost Auto Data Model** — A ghost auto captures a simulated trajectory plus the full robot config (bumper size, offset, swerve module locations, holonomic flag) and a team name label. Stored as `.ghostauto` JSON files.

2. **Import/Export** — Import individual `.ghostauto` files via file picker, or batch-import from another PathPlanner project folder. Batch-export all autos in your current project as ghosts to a chosen directory.

3. **Multi-Ghost Rendering** — Up to 10 simultaneous ghosts with distinct color-coded trajectories, robot outlines with swerve modules, and start/end indicators. Ghosts animate in sync with the main trajectory preview.

4. **Collision Detection** — Automatic time-sampled bounding-circle collision checks between the main trajectory and each ghost, and between ghost pairs. Collisions render as red warning circles on the field.

5. **LAN Sync (Real-Time Multi-Peer)** — Zero-configuration peer discovery via UDP broadcast. Deterministic WebSocket connections with automatic port fallback (5811–5821). Multiple PathPlanner instances on the same network see each other's current auto as a ghost in real-time. Includes keepalive, reconnect logic, backpressure, and graceful ghost fade on disconnect.

6. **Ghost UI Controls** — Per-ghost visibility toggle, pin network ghost as local on disconnect, team number / display name field with persistence, sync enable/disable toggle, manual IP:port connect dialog.

7. **Visual Polish** — Waypoint robot copies rendered at reduced opacity so they don't compete with the animated preview. The animated preview robot uses a thicker outline for clear visual distinction.

### Files Added or Modified

| File | What Changed |
|---|---|
| `lib/auto/ghost_auto.dart` | **New.** Ghost auto data model, serialization, import/export, trajectory sampling, 10-color palette. |
| `lib/services/ghost_sync_service.dart` | **New.** LAN sync service (~740 lines): UDP discovery, WebSocket data, multi-peer N:N topology, backpressure, identity re-keying, ghost fade timers. |
| `lib/main.dart` | Instantiates `GhostSyncService` at startup, restores sync preference. |
| `lib/util/prefs.dart` | Added ghost-related preference keys and defaults. |
| `lib/util/path_painter_util.dart` | Added optional `bumperStrokeWidth` parameter to `paintRobotOutline()`. |
| `lib/pages/home_page.dart` | Export/Import Ghosts toolbar actions; batch operations; wires sync service. |
| `lib/pages/auto_editor_page.dart` | Passes ghost data and sync service through to child editors. |
| `lib/widgets/editor/split_auto_editor.dart` | Manages local + network ghost state, visibility toggle, pin-on-disconnect, sync listeners. |
| `lib/widgets/editor/split_path_editor.dart` | Forwards ghost data to painter; renders color-coded ghost legend. |
| `lib/widgets/editor/path_painter.dart` | Renders ghost trajectories, ghost robots, endpoint indicators, collision warnings; visual polish. |
| `lib/widgets/editor/tree_widgets/auto_tree.dart` | Ghost legend sidebar with visibility/pin/remove controls; sync toggle panel with team # field. |

---

## A note on AI

To be upfront, I used Github Copilot to reference, plan, and build changes for this fork. It's not 100% of the changes, but there are many areas that I let AI write, followed by reviewing and debugging.