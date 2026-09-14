# amr-station-handshake

[![CI](https://github.com/sulemanali5760/amr-station-handshake/actions/workflows/ci.yml/badge.svg)](https://github.com/sulemanali5760/amr-station-handshake/actions/workflows/ci.yml)

A mobile robot hands over a load at a conveyor station. The robot side is
ROS 2, the station side is a PLC, and between them sits the part that goes
wrong on real sites: a signal handshake with a timeout on every step.

```
amr_node  ──Dock action──►  station_bridge  ──OPC UA──►  station PLC
(ROS 2)                     (ROS 2)                      (simulated, 50 ms cycle)
```

1. The AMR drives to the station and sends a `Dock` goal.
2. The bridge sets `AmrDockRequest` and waits for `StationReady`.
3. It sets `AmrInPosition` and waits for `TransferActive`, then `TransferDone`.
4. It drops both request bits and waits for the station to clear `StationReady`.
5. The goal succeeds and the AMR drives home.

Every phase has its own timeout. A station fault, a withdrawn `StationReady`
or a lost OPC UA session ends the handover, and the request bits are always
written back to FALSE so nothing stays latched on the PLC.

Full design: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Run it

Ubuntu 24.04 with ROS 2 Jazzy:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/sulemanali5760/amr-station-handshake.git
cd ~/ws
rosdep install --from-paths src --ignore-src -y
pip install --break-system-packages asyncua
colcon build && source install/setup.bash

ros2 launch amr_station_handshake demo.launch.py
```

Break the station on purpose:

```bash
ros2 launch amr_station_handshake demo.launch.py scenario:=never_ready
ros2 launch amr_station_handshake demo.launch.py scenario:=fault_during_transfer
```

| Scenario | Station | Handover ends |
|---|---|---|
| `normal` | Ready after 0.5 s, 2 s transfer | `succeeded` |
| `never_ready` | ignores the request | `timed_out` after 5 s |
| `fault_during_transfer` | faults with code 1 halfway | `faulted` |

Point the bridge at a real PLC instead of the simulator:

```bash
ros2 run amr_station_handshake station_bridge --ros-args \
  -p endpoint:=opc.tcp://192.168.0.10:4840/ -p timeout.transfer:=30.0
```

The PLC has to expose the seven signals listed in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#signals) under the namespace
`urn:amr-station-handshake:station`.

Watch the robot and the handover from a second terminal:

```bash
ros2 topic echo /amr/position
ros2 action send_goal /station/dock amr_handshake_interfaces/action/Dock "{amr_id: manual}" --feedback
```

## Tests

CI builds both packages in the `ros:jazzy-ros-base` container and runs:

- **`test_handshake.py`**: every transition, timeout, fault and cancel path
  of the state machine, with no ROS and no network.
- **`test_demo_launch.py`**: `launch_testing` with real processes. A normal
  station plus an AMR running two missions must exit 0; a station that faults
  mid-transfer must abort a Dock goal with the fault code.

## Layout

```
src/
  amr_handshake_interfaces/   Dock.action
  amr_station_handshake/
    handshake.py              pure state machine, no ROS or OPC UA
    station_bridge.py         Dock action server + OPC UA client
    amr_node.py               simulated drive + Dock action client
    station_sim.py            OPC UA server acting as the station PLC
    launch/demo.launch.py
    test/
```

## Scope

Driving is a 1-D position ramp, not Nav2 in Gazebo: navigation is solved
elsewhere, the handover to a machine is the integration work. The simulator
listens on 127.0.0.1 without OPC UA security. Next steps are in the
[plan](docs/ARCHITECTURE.md#plan): a browser demo, a C++ `amr_node`, and the
station as a TwinCAT 3 function block.

Tested on Ubuntu 24.04 in CI only. ROS 2 core also runs on Windows, for
example through RoboStack, but this repo has not been tried there.

## Licence

MIT
