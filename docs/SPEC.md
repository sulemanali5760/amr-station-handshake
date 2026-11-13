# Station handover handshake: specification

Version 0.1. An open, vendor-neutral signal handshake for handing a load between a mobile robot
(AMR/AGV) and a PLC-controlled conveyor station, with rules a station can be tested against.

## Why this exists

VDA 5050 standardises how a fleet controller talks to vehicles. It does not cover the station.
Its scope section says: "Interfaces unrelated to the communication between a fleet control system
and mobile robots are excluded, such as interfaces to peripheral equipment, infrastructure
components, or external IT systems." ([VDA 5050, `VDA5050_EN.md`](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md), read 13 Nov 2026)

So the signal exchange between vehicle and conveyor is defined again in every project. Vendors
solve it with their own means (infrared links, optical I/O couplers, wireless modules, or fleet
manager to PLC over OPC UA or Modbus). Semiconductor fabs have a standard for exactly this step,
SEMI E84, but it is tied to parallel I/O and carrier handling in fabs and is not freely available.

This document writes the handshake down once, as testable rules, and
[`handover-check`](../tools/handover-check) tests a station against them. It borrows the shape of
E84 (request, ready, busy, complete, a time limit on every step) and is not a copy of it. It is a
project specification, not a standard of any body.

## Signals

Eight signals, plain `BOOL` and `INT`, so they map onto a PLC data block or global variable list.
The transport is OPC UA; where the signals live on a given PLC is described by a signal map
(namespace URI plus one node id per signal).

| Signal | Type | Written by | Meaning |
|---|---|---|---|
| `AmrDockRequest` | BOOL | vehicle | The vehicle wants to hand over a load |
| `AmrInPosition` | BOOL | vehicle | The vehicle is docked; the transfer may start |
| `AmrHeartbeat` | INT | vehicle | Counter that changes at least every 100 ms while the vehicle side is alive |
| `StationReady` | BOOL | station | The station accepts the handover; stays on until the request drops |
| `TransferActive` | BOOL | station | The conveyor is moving the load |
| `TransferDone` | BOOL | station | The load has arrived on the station |
| `Fault` | BOOL | station | Station fault, latched until acknowledged |
| `FaultCode` | INT | station | 0 = none; non-zero while `Fault` is on. Reference codes: 1 conveyor blocked, 2 heartbeat lost, 3 vehicle left position |

## Sequence

```
vehicle:  DockRequest ─┐                    InPosition ─┐                         both off ─┐
station:               └─► Ready                        └─► TransferActive ─► TransferDone  └─► all off
```

## Rules for the station

Each rule has a time limit. The defaults below are starting points; a plant specification sets the
real values, and the tester takes them as options.

| Rule | The station shall | Default limit |
|---|---|---|
| R1 | keep all five outputs off while there is no request | observed for 1 s |
| R2 | set `StationReady` after `AmrDockRequest`, and move nothing yet | 5 s |
| R3 | not start a transfer before `AmrInPosition` | observed for 1 s |
| R4 | set `TransferActive` after `AmrInPosition`, then `TransferDone`; `TransferActive` is off when `TransferDone` is on | start 3 s, transfer 15 s |
| R5 | clear all outputs after the vehicle dropped both request bits | 3 s |
| R6 | accept a second handover straight after the first | as R2 to R5 |
| R7 | return to idle, without a transfer, when the request is withdrawn before docking | 3 s |
| R8 | stop the transfer and set `Fault` with a non-zero `FaultCode` when the heartbeat stops changing | 2 s |
| R9 | clear the fault when the request drops (this is the acknowledge) | 3 s |
| R10 | stop the conveyor when `AmrInPosition` goes off during the transfer | 1 s |

Why R8 and R10 are there: the vehicle side can die (process killed, WLAN gone) or the vehicle can
move while the conveyor runs. Without them the station keeps pushing a load at a robot that is no
longer there.

## Rules for the vehicle side

The vehicle side shall: change the heartbeat before and during every request; put a time limit on
every wait; drop both request bits whenever it ends a handover, for any reason; treat `Fault`, or
`StationReady` going off during the transfer, as the end of the handover. The ROS 2 bridge in this
repository implements this side (`handshake.py`, with unit tests for every path).

## What this specification does not do

- **It is not a safety function.** Nothing here protects people. A real station needs its own
  sensing of the docked vehicle, guarding of the transfer opening and a safety controller. These
  signals coordinate; they do not interlock.
- **One vehicle, one station, one direction per handover.** No carrier identification, no separate
  load and unload requests, no multi-lane stations.
- **Passing the rules is not proof of a good station.** It shows the signal behaviour at the
  interface. Mechanics, sensors and the load itself are outside.
