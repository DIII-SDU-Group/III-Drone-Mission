# III-Drone-Mission

`iii_drone_mission` contains the mission-layer logic that sits above control primitives and below operator workflows. It is responsible for turning mission specifications and behavior definitions into concrete runtime actions.

## Package Role

This package owns:

- mission specification loading and lookup
- mission execution orchestration
- behavior-tree related type integration
- PX4-facing mission support code used by the mission layer

## Module Map

### `src/mission`

- `mission_specification.cpp`: loads YAML mission definitions, expands paths, and exposes mission entries by key
- `mission_executor.cpp`: runtime mission execution/orchestration logic

This is the center of the package. If you are changing mission sequencing or how operators describe missions, start here.

### `src/behavior`

- `port_types.cpp`: behavior-tree port type integration and conversion helpers

This module keeps the mission package compatible with the behavior-tree layer without leaking those details into the rest of the runtime.

### `src/px4`

- `px4/px4_handler.cpp`: PX4 integration helpers used by the mission layer
- `px4_mode_test.cpp`: local PX4 mode experimentation/test utility source

These files handle the boundary between mission intent and PX4-specific runtime interaction.

## Mission Specification Format

The package currently expects mission YAML with:

- `executor_owned_mode`: the mode owned by the mission executor itself
- `entries`: a list of named mission entries

Each entry can include:

- `key`
- `mode_name`
- `behavior_tree_xml_file`
- `next_mode` (optional)
- `allow_activate_when_disarmed` (optional)

The loader expands `~` in behavior-tree paths and defaults omitted optional fields to safe values.

Point-queue ports (for example `FollowWaypointPath` `waypoints`, `LoopPoint`
`queue`) also accept a literal list `"x,y,z;x,y,z;..."` in the tree XML or
through `SetBlackboard`. Whitespace around numbers and separators is ignored;
an empty list, an empty entry, a point without exactly three coordinates or a
non-finite coordinate fails when the tree is loaded.

## Runtime Profiles and the opti_track Rules

A node's runtime profile is its `iii_runtime_profile` ROS parameter when that
is set, else `III_SYSTEM_PROFILE` (set by supervision for every entity). An
empty or unknown profile restricts nothing.

`opti_track` is the OptiTrack-lab profile: flight basics without cable,
gripper or perception. It is commissioned and onboard, so every catalog that
carries it needs exactly one default mission: `opti-track-hover`.

Under `opti_track` only these III behavior nodes are available, besides the
BehaviorTree.CPP built-ins: `FlyToPosition`, `FollowWaypointPath`, `Hover`,
`ModeExecutorAction`, `WaitForPX4Airborne`, `VerifyDisarmed`,
`StoreCurrentState`, `LogMessage`, `SetBlackboardBool`, `BlackboardBool`,
`SetBlackboardString`, `BlackboardStringEquals`, `StringEquals`,
`ApplyPendingIntentUpdates`, `RetryUntilSuccessfulOnAborted`,
`RosbagRecordingScope`, `StartRosbagRecording`, `StopRosbagRecording`,
`LoopPoint`, `SplitPointQueue`, `PartitionPointQueue`, `QueueHasPoints`
(`src/mission/profile_restrictions.cpp`). Every layer rejects the rest with
"<thing> is not available in the opti_track profile":

- the catalog build fails when a tree of a mission registered for
  `opti_track` uses another node or includes other tree files; the
  behavior-node contract exporter publishes the allowlist the build checks;
- the mission executor refuses to configure or select such a mission;
- the custom operation node rejects every operation except `hover`,
  `fly_to_position` and `follow_waypoint_path` before it sends a goal; the
  reason is on `/mission/custom_operation/mode_status`.

## OptiTrack Missions

Registered for `opti_track`, `sim` and `hil`. World targets use the III world
frame `world`, yaw is in radians, and no world target lies below 1.1 m
(FlyToPosition rejects targets below
`/control/maneuver_controller/minimum_target_altitude`, 1.0 m in the real
parameter set). Each tree defines its geometry with `SetBlackboard` at the top,
to adapt to the cage. Each run records one rosbag (owner `mission`) with the
flight analysis topics plus the OptiTrack/external-vision ones. A mission that
ends selects `/mission/mission_done_select_mode` (PX4 Hold).

| Mission | Classification | Modes | Flight |
| --- | --- | --- | --- |
| `opti-track-hover` | production, default | OT Hover | hold 10 s, climb 0.3 m, return, hover |
| `opti-track-maneuvers` | experimental | OT Maneuvers | centre at 1.2 m, stop-and-go and blended 1 m box, yaw steps, height step to 1.5 m, waypoint path |
| `opti-track-cycle` | experimental | OT Takeoff, OT Shuttle, OT Land | take off, hover until Proceed, shuttle across the cage, land |
| `opti-track-mode-loop` | experimental | OT Loop Takeoff, four rectangle modes | take off, then loop stop-and-go and blended laps at 1.2 m and 1.6 m |

`opti-track-hover` is production because the qualified and field-candidate
catalogs carry no test entries (and no experimental ones unless explicitly
included), yet need a default for every commissioned onboard profile.

- **opti-track-hover and opti-track-maneuvers**: select them only while the
  vehicle hovers; the pilot hands over from a steady hover (at least 1.1 m for
  the hover mission). The mode executor arms a disarmed vehicle when its owned
  mode is selected, whatever the mission specification says.
- **opti-track-cycle**: OT Takeoff takes off unless PX4 reports the vehicle
  airborne, flies to the centre and hovers until the operator calls
  `/mission/opti_track/proceed` (`std_srvs/SetBool`, accepted in OT Takeoff
  only), for at most 60 s. Without Proceed it sets
  `opti_track.cycle_land_now`; OT Shuttle then skips the shuttle and OT Land
  lands.
- **opti-track-mode-loop**: loops OT Lower Stop-and-Go, OT Upper Stop-and-Go,
  OT Lower Blended, OT Upper Blended until the pilot or the ground control
  changes mode.
- Without a global position PX4 takes off to its default takeoff altitude
  (`MIS_TAKEOFF_ALT`) instead of the requested 1.2 m: set it for the cage.

## Mode Executor Behavior

- A pilot takeover is stick movement: some axis moved by more than
  `/mission/manual_stick_input_threshold` from where it was when the executor
  became active. A stick resting away from centre (PX4 reports throttle -1 at
  the bottom) does not end the mission; samples PX4 marks invalid are
  ignored. The custom operation mode uses the same rule.
- PX4 failsafes are deferred (5 s each) only during a mode handoff: from
  scheduling a mode (activation, next mode, land, takeoff) until PX4 runs it.
- A takeoff without a global ground altitude estimate uses PX4's default
  takeoff altitude and says so in the log.

## Tests

The package test suite currently validates:

- mission file loading
- home-directory expansion
- optional field defaults
- lookup failures for missing mission keys
- iterator behavior across loaded entries
- the profile allowlists in the catalog build (`test_mission_catalog.py`), the
  executor and the custom operation node
- the OptiTrack missions: registration, mode names, trees loading against the
  node registry and world targets at or above 1.1 m
  (`opti_track_missions_test.cpp`)
- literal waypoint lists, stick takeover and handoff-scoped failsafe deferral

Typical package-only commands:

```bash
colcon build --packages-select iii_drone_mission
colcon test --packages-select iii_drone_mission --ctest-args --output-on-failure
colcon test-result --verbose
```

## Extension Guidelines

- keep parsing and validation close to `mission_specification.cpp`
- prefer explicit defaults for optional mission fields
- add tests for every new mission-file field, because malformed mission config is expensive to debug at runtime
