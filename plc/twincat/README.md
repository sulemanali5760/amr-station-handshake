# Station side for TwinCAT 3: `FB_StationHandshake`

The station half of the handshake in [docs/SPEC.md](../../docs/SPEC.md), as one
function block in Structured Text. Add the four files to a PLC project, call the
block once per cycle, and [`handover-check`](../../tools/handover-check) can test
the station over OPC UA.

| File | What it is |
|---|---|
| `FB_StationHandshake.TcPOU` | the block: rules R1 to R10, heartbeat watchdog, conveyor time-out |
| `E_HandoverState.TcDUT` | its states: idle, ready, transferring, done, fault |
| `GVL_Handshake.TcGVL` | the eight signals, published on TF6100 with the names the [example signal map](../../tools/handover-check/maps/twincat3_tf6100.example.yaml) expects |
| `FB_StationHandshake_Test.TcPOU` | TcUnit suite, seven tests (optional) |

## Wire it

```iecst
PROGRAM MAIN
VAR
    fbHandshake : FB_StationHandshake;
    bEndSensor AT %I* : BOOL;      // load at the end of the conveyor
    bConveyor  AT %Q* : BOOL;      // conveyor drive
END_VAR

fbHandshake(
    bAmrDockRequest := GVL_Handshake.AmrDockRequest,
    bAmrInPosition  := GVL_Handshake.AmrInPosition,
    nAmrHeartbeat   := GVL_Handshake.AmrHeartbeat,
    bStationFree    := NOT bEndSensor,          // plus your own "conveyor healthy"
    bLoadArrived    := bEndSensor,
    bStationReady   => GVL_Handshake.StationReady,
    bTransferActive => GVL_Handshake.TransferActive,
    bTransferDone   => GVL_Handshake.TransferDone,
    bFault          => GVL_Handshake.Fault,
    nFaultCode      => GVL_Handshake.FaultCode,
    bConveyorRun    => bConveyor);
```

Then, from any PC that reaches the PLC:

```bash
pip install handover-check
handover-check run opc.tcp://<plc>:4840 --map my_station.yaml
```

`my_station.yaml` is a copy of the
[TwinCAT example map](../../tools/handover-check/maps/twincat3_tf6100.example.yaml).
Check its namespace URI and node IDs in UaExpert first; they depend on the project.

## Fault codes

| Code | Cause | Rule |
|---|---|---|
| 1 | load did not arrive within `tTransferTimeout` (default 15 s) | — |
| 2 | heartbeat unchanged for `tHeartbeatTimeout` (default 1 s) while a request is on | R8 |
| 3 | vehicle left position during the transfer | R10 |

Every fault stops the conveyor and stays latched until the vehicle drops its request (R9).

## What is verified, and how

- **Compiles** with 0 errors, tests included, on TwinCAT 3.1.4026.25 with TcUnit
  1.3.1 (headless build, 14 Nov 2026). A deliberately broken copy fails the same
  build, so the files really are compiled.
- **Unit tests run**: all 7 pass on the local TwinCAT 3.1.4026.25 runtime
  (14 Nov 2026). With the R10 branch removed on purpose, exactly the R10 test
  fails, so the suite does catch a broken block.
- **Not yet run**: `handover-check` against this block over TF6100 (needs the
  TF6100 OPC UA server licence).
- **Not a safety function.** Guarding of the transfer opening and sensing of the
  docked vehicle belong to the safety controller.

A Siemens S7-1500 (SCL) version is planned; it is not written yet because there
is no way here to compile it.
