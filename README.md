# robstride_hardware_interface

## 1. Introduction

ROS 2 package providing a [ros2_control](https://github.com/ros-controls/ros2_control) `hardware_interface::SystemInterface` plugin for [RobStride](https://www.robstride.com/) CAN actuators (RS00/RS01/RS02/RS03/RS04/RS05/RS06), built on top of [robstride_sdk](https://github.com/kmj060703/RobstrideSDK) and [robstride_interfaces](https://github.com/kmj060703/robstride_interfaces) — the RobStride equivalent of ROBOTIS's `dynamixel_hardware_interface` for Dynamixel servos.

Sustains a **≥300 Hz** control loop: `read()`/`write()` never block on CAN I/O (they only touch `robstride_sdk`'s lock-free per-motor state and batch-send via `sendmmsg()`), so the update rate is limited by real CAN bus bandwidth, not by request/response round trips. See [robstride_sdk's README](https://github.com/kmj060703/RobstrideSDK/blob/main/README.md#1-introduction) for the I/O design.

## 2. Prerequisites

- ROS 2 Jazzy (this workspace).
- Hardware: a SocketCAN-compatible adapter (e.g. a canable/candlelight USB-CAN adapter) per bus, RobStride motors wired to it, and a power supply for the motors.
- `robstride_sdk`, `robstride_interfaces` built in the same workspace.

## 3. Installation

1. Clone the repositories into your ROS workspace:

   ```bash
   cd ~/${WORKSPACE}/src
   git clone https://github.com/kmj060703/RobstrideSDK.git
   git clone https://github.com/kmj060703/robstride_hardware_interface.git
   git clone https://github.com/kmj060703/robstride_interfaces.git
   ```

2. Build the packages:

   ```bash
   cd ~/${WORKSPACE}
   colcon build --packages-select robstride_sdk robstride_interfaces robstride_hardware_interface
   ```

3. Source your workspace:

   ```bash
   source ~/${WORKSPACE}/install/setup.bash
   ```

## 4. CAN interface setup

RobStride motors run at 1 Mbps. Bring each interface up before launching:

```bash
./scripts/setup_can.sh can0                 # defaults: 1 Mbps, txqueuelen 1000
./scripts/setup_can.sh can0 1000000 1000    # explicit bitrate / txqueuelen
```

Some USB-CAN controllers (including many gs_usb/candlelight-based canable adapters) don't support automatic bus-off recovery (`restart-ms`) — the script detects this and falls back to configuring without it, logging a warning. If that happens, a bus-off event needs a manual `down`/`up` cycle to recover.

## 5. Configuration (`ros2_control` xacro)

Hardware-level params:

| Param | Meaning | Default |
|---|---|---|
| `master_id` | Host CAN id used as source address in outgoing frames | `253` (`0xFD`) |
| `error_timeout_ms` | Max time without fresh feedback before a joint is treated as stale/faulted (also arms each motor's own `CAN_TIMEOUT` watchdog, param `0x7028`) | `4000` |

Per-joint params:

| Param | Meaning |
|---|---|
| `id` | Motor's CAN id |
| `can_interface` | SocketCAN interface name this joint's motor is on (e.g. `can0`) — different joints may use different buses, e.g. one per quadruped leg |
| `actuator_type` | `"00"`..`"06"` / `"0"`..`"6"` / `"RS00"`..`"RS06"` |
| `kp`, `kd` | Position/velocity gains sent with every motion-control frame |

Example (see the `qdd_description` package's `robstride_ros2_control.xacro` for the full macro):

```xml
<ros2_control name="robstride" type="system">
  <hardware>
    <plugin>robstride_hardware_interface/RobstrideHardware</plugin>
    <param name="master_id">253</param>
    <param name="error_timeout_ms">4000</param>
  </hardware>
  <joint name="rs_joint1">
    <param name="id">1</param>
    <param name="can_interface">can0</param>
    <param name="actuator_type">00</param>
    <param name="kp">5.0</param>
    <param name="kd">0.3</param>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... -->
</ros2_control>
```

For a bench/CI test without real hardware, swap the `<hardware>` block for `mock_components/GenericSystem` (see `qdd_bringup`'s `use_mock_robstride:=true` launch arg).

## 6. Activation behavior (safety)

`on_activate()` does not blindly seed command interfaces at 0:

1. Opens each CAN bus and starts its read thread.
2. Sends an Enable frame to every joint, then waits (up to `error_timeout_ms`) for each motor to actually report `run_state == MOTOR` — resending Enable every 200 ms in case the first frame was dropped, since CAN is fire-and-forget.
3. Only once every joint confirms, seeds each command interface from that joint's real measured position — not 0 — so `write()`'s first cycle holds position instead of snapping toward a stale target.
4. If any joint fails to confirm within the timeout, disables whatever did confirm, tears down the buses, and returns `ERROR` instead of `SUCCESS`.

`read()` and `write()` both check each joint's `last_update_ns` against `error_timeout_ms`: `read()` returns `ERROR` (triggering `controller_manager` to deactivate) if any joint has gone stale, and `write()` independently substitutes a Disable frame for any stale or faulted joint instead of continuing to send a motion-control command toward a possibly outdated target.

## 7. Topics and services

All under the hardware component's own node namespace (`~`):

| Name | Type | Description |
|---|---|---|
| `~/robstride_state` | `robstride_interfaces/msg/RobstrideState` | Per-joint id/enabled/run_state/fault_bits, published every `read()` cycle |
| `~/get_data_from_robstride` | `robstride_interfaces/srv/GetDataFromRobstride` | Read live telemetry (position/velocity/torque/temperature/run_state/fault_bits) for one motor |
| `~/set_data_to_robstride` | `robstride_interfaces/srv/SetDataToRobstride` | Set `kp`/`kd`, or write an arbitrary RobStride parameter index |
| `~/set_zero_robstride` | `robstride_interfaces/srv/SetZeroRobstride` | Set a motor's mechanical zero |
| `~/set_torque` | `std_srvs/srv/SetBool` | Enable/disable all configured motors |

See [robstride_interfaces' README](https://github.com/kmj060703/robstride_interfaces/blob/main/README.md) for full field descriptions.

## 8. Bus bandwidth

A single 1 Mbps CAN bus has a hard throughput ceiling, and it's shared across every motor on that bus. Each `read`+`write` cycle needs `2 · N_motors` frames round-tripped (one motion-control command out, one feedback frame back, per motor); an 8-byte extended CAN frame takes roughly 130–190 µs on the wire at 1 Mbps depending on bit-stuffing, so a rough estimate of bus utilization is:

```
utilization ≈ 2 · N_motors · update_rate_hz · frame_time_s
```

This is only an estimate — actual overhead depends on your specific adapter, motor count, and any other traffic sharing the bus (parameter services, etc). **Always measure your own setup** before trusting a given `update_rate`:

```bash
# 1) bring the interface up (see section 4) and activate the hardware
# 2) count real frames/sec on the wire for a short window:
timeout 1 candump can0 | wc -l
```

Compare that count against `2 · N_motors · update_rate_hz` — if it's meaningfully short of that target, or if you see `read()`/`write()` staleness errors (section 6) under load, the bus is saturated and `update_rate` (or the number of motors on that bus) needs to come down. There's no universal safe number: more motors per bus, a slower/higher-overhead adapter, or added service traffic all lower the ceiling. Leave headroom rather than tuning right up to the saturation point — a bus running near 100% utilization has no margin for retries, parameter reads/writes, or momentary bus noise, and can start intermittently missing the `error_timeout_ms` deadline.

## 9. License

Apache License 2.0.
