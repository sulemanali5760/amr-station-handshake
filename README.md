# amr-station-handshake

[![CI](https://github.com/sulemanali5760/amr-station-handshake/actions/workflows/ci.yml/badge.svg)](https://github.com/sulemanali5760/amr-station-handshake/actions/workflows/ci.yml)

A mobile robot hands over a load at a PLC-controlled conveyor station.
*Ein fahrerloses Transportfahrzeug übergibt eine Last an eine SPS-gesteuerte Förderstation.*

VDA 5050 standardises fleet controller to vehicle and expressly leaves out
"interfaces to peripheral equipment". The signal handshake between vehicle and
conveyor is therefore invented again in every project. This repository writes
it down once and makes it testable:

| | |
|---|---|
| [**Specification**](docs/SPEC.md) | Eight signals and ten rules (R1 to R10) for the station, each with a time limit |
| [**`handover-check`**](tools/handover-check) | `pip install handover-check`. Plays the vehicle side over OPC UA against a station PLC, also the ways a vehicle fails, and reports rule by rule. No ROS, no robot; Windows or Linux |
| [**`FB_StationHandshake`**](plc/twincat) | The station side for TwinCAT 3 in Structured Text: rules R1 to R10, heartbeat watchdog, conveyor time-out, the signal GVL and a TcUnit suite. 7 unit tests pass on a TwinCAT 3.1.4026 runtime |
| **ROS 2 reference** (below) | The vehicle side as a `Dock` action server with the handshake state machine, a simulated AMR and a simulated station |

```bash
pip install handover-check
handover-check station &                                  # reference station, no PLC needed
handover-check run opc.tcp://127.0.0.1:4860/station/      # ten rules, pass or fail
```

## The ROS 2 reference

The robot side is ROS 2, the station side is a PLC, and between them sits the
part that goes wrong on real sites: a signal handshake with a timeout on every
step and a heartbeat the station watches.

```
amr_node  ──Dock action──►  station_bridge  ──OPC UA──►  station PLC
(ROS 2)                     (ROS 2)                      (simulated, 50 ms cycle)
```

1. The AMR drives to the station and sends a `Dock` goal.
2. The bridge starts a heartbeat, keeps its request bits low and waits until
   the station is idle and fault-free.
3. It sets `AmrDockRequest` and waits for `StationReady`.
4. It sets `AmrInPosition` and waits for `TransferActive`, then `TransferDone`.
5. It drops both request bits and waits for the station to clear `StationReady`.
6. The goal succeeds and the AMR drives home.

Every phase has its own timeout. A station fault, a withdrawn `StationReady`
or a broken OPC UA link ends the handover, and the request bits are written
back to FALSE. If the robot side dies without doing that, the station's
heartbeat watchdog stops the transfer on its own.

Full design: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Run it in the browser

Open the repo in **GitHub Codespaces** (Code → Codespaces → Create codespace).
The container builds the workspace on first start; then in the terminal:

```bash
ros2 launch amr_station_handshake demo.launch.py
```

## Run it locally

Ubuntu 24.04 with ROS 2 Jazzy:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/sulemanali5760/amr-station-handshake.git
cd ~/ws
rosdep install --from-paths src --ignore-src -y
pip install --break-system-packages asyncua==2.0.1
colcon build && source install/setup.bash

ros2 launch amr_station_handshake demo.launch.py
```

## Break the station on purpose

```bash
ros2 launch amr_station_handshake demo.launch.py scenario:=never_ready
ros2 launch amr_station_handshake demo.launch.py scenario:=fault_during_transfer
ros2 launch amr_station_handshake demo.launch.py scenario:=crash_during_transfer
```

| Scenario | Station | Handover ends |
|---|---|---|
| `normal` | Ready after 0.5 s, 2 s transfer | `succeeded` |
| `never_ready` | ignores the request | `timed_out` after 5 s |
| `fault_during_transfer` | faults with code 1 halfway | `faulted` |
| `crash_during_transfer` | process exits halfway | `link_error` |
| any, robot side dies | watchdog faults with code 2 after 1 s | station stops on its own |
| any, robot leaves its position mid-transfer | station faults with code 3 | station stops on its own |

Watch the robot and the handover from a second terminal:

```bash
ros2 topic echo /amr/position
ros2 action send_goal /station/dock amr_handshake_interfaces/action/Dock "{amr_id: manual}" --feedback
```

## Connect a real PLC

The bridge reads the namespace URI and node IDs of the eight signals from a
signal map. Copy one of the examples, check the node IDs in UaExpert, and
start the bridge with it:

```bash
ros2 run amr_station_handshake station_bridge --ros-args \
  -p endpoint:=opc.tcp://192.168.0.10:4840 \
  -p signal_map:=/path/to/my_station.yaml \
  -p timeout.transfer:=30
```

| Example | PLC |
|---|---|
| [`s7_1500.example.yaml`](src/amr_station_handshake/config/s7_1500.example.yaml) | Siemens S7-1500, data block `DB_Handshake` |
| [`twincat3_tf6100.example.yaml`](src/amr_station_handshake/config/twincat3_tf6100.example.yaml) | Beckhoff TwinCAT 3 with TF6100, `GVL_Handshake` |

The PLC program has to implement the station side of the handshake, including
the heartbeat watchdog. For TwinCAT 3 that is [`FB_StationHandshake`](plc/twincat)
with `GVL_Handshake`, whose names match the TwinCAT example map. The example
maps have not been tested against hardware.

## Tests

CI builds both packages in the `ros:jazzy-ros-base` container and runs:

- **`test_handshake.py`**: every transition, timeout, fault, reset and cancel
  path of the state machine, with no ROS and no network.
- **`test_flake8.py`, `test_pep257.py`**: ament code style and docstrings.
- **`test_demo_launch.py`**: `launch_testing` with real processes. A full AMR
  mission; a faulting station, a silent station and a cancel mid-transfer,
  each checked by reading the request bits back from the station; a station
  that crashes mid-transfer; and the heartbeat watchdog.

It also builds the Codespaces container image.

## Layout

```
plc/twincat/                  FB_StationHandshake, GVL_Handshake, TcUnit suite
tools/handover-check/         the conformance tester (PyPI: handover-check)
src/
  amr_handshake_interfaces/   Dock.action
  amr_station_handshake/
    handshake.py              pure state machine, no ROS or OPC UA
    station_bridge.py         Dock action server, OPC UA client, heartbeat
    amr_node.py               simulated drive + Dock action client
    station_sim.py            OPC UA server acting as the station PLC
    config/                   signal maps: simulator, S7-1500, TwinCAT 3
    launch/demo.launch.py
    test/
```

## Scope

Driving is a 1-D position ramp, not Nav2 in Gazebo: navigation is solved
elsewhere, the handover to a machine is the integration work. Nothing here is
a safety function: on a real station the interlocks are sensors and a safety
PLC, and OPC UA only coordinates. The simulator listens on 127.0.0.1 without
OPC UA security. Next steps are in the [plan](docs/ARCHITECTURE.md#plan).

Tested on Ubuntu 24.04 in CI. ROS 2 core also runs on Windows, for example
through RoboStack, but this repo has not been tried there.

## Licence

MIT
