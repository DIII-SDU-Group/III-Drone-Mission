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

## Tests

The package test suite currently validates:

- mission file loading
- home-directory expansion
- optional field defaults
- lookup failures for missing mission keys
- iterator behavior across loaded entries

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
