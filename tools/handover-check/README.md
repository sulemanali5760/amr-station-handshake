# handover-check

Test a conveyor station's **AMR/AGV load-handover handshake** before the robot arrives.

The tool connects to the station PLC over OPC UA and plays the vehicle side: it requests a handover,
docks, releases, and also does what vehicles do when things go wrong (goes silent, leaves mid-transfer,
changes its mind). It checks the station's answers and response times against ten rules and reports
rule by rule.

```
$ handover-check run opc.tcp://192.168.0.10:4840 --map station.yaml --station-is-safe

[PASS] R1  Idle at rest: no output is on without a request
           all outputs off for 1 s without a request
[PASS] R2  StationReady follows AmrDockRequest in time, and nothing moves yet
           StationReady 0.53 s after the request (limit 5 s)
[PASS] R3  No transfer before AmrInPosition
           no transfer for 1 s while the vehicle was not in position
[PASS] R4  Transfer starts after AmrInPosition and completes; TransferActive is off when TransferDone is on
           transfer started after 0.02 s and finished after 4.02 s
[PASS] R5  Station returns to idle after the vehicle releases
           station idle 0.03 s after the release (limit 3 s)
[PASS] R6  A second handover works straight after the first
           two handovers in a row completed
[PASS] R7  A withdrawn request returns the station to idle without a transfer
           station back to idle 0.02 s after the request was withdrawn
[FAIL] R8  Heartbeat watchdog: a silent vehicle stops the transfer with a fault code
           the vehicle went silent for 2 s during the transfer and the station carried on: Ready, TransferActive
[SKIP] R9  Dropping the request acknowledges and clears a fault
           could not bring the station into a fault to acknowledge (see R8): Ready, TransferActive
[PASS] R10 The conveyor stops when the vehicle leaves its position
           conveyor stopped 0.02 s after the vehicle left its position (limit 1 s; station: Fault(code 3))

Station does NOT conform. 8 passed, 1 failed, 1 not testable.
```

*Real output, against the bundled reference station with its watchdog switched off; address changed.*

## Why

VDA 5050 covers fleet controller to vehicle and expressly leaves out "interfaces to peripheral
equipment". The handshake between vehicle and conveyor is therefore written again in every project,
by two teams, and its mistakes show up at commissioning with the robot standing in front of the
station. The rules are written down in [docs/SPEC.md](../../docs/SPEC.md); this tool lets the PLC
programmer test the station side alone, at the desk or on site, days earlier.

## Install

```bash
pip install handover-check
```

Python 3.11 or newer; Windows or Linux. No ROS and no robot needed.
Offline bundles for machines without internet are attached to each
[release](https://github.com/sulemanali5760/amr-station-handshake/releases).

## Use

Try it without a PLC, in two terminals:

```bash
handover-check station                        # reference station on opc.tcp://127.0.0.1:4860/station/
handover-check run opc.tcp://127.0.0.1:4860/station/
```

```bash
handover-check station --defect no_watchdog   # a station with one rule switched off
```

Against your PLC: describe where the eight signals live in a signal map (examples for the reference
station, Siemens S7-1500 and Beckhoff TwinCAT 3 are in [`maps/`](maps)), then:

```bash
handover-check run opc.tcp://192.168.0.10:4840 --map my_station.yaml --station-is-safe
```

| Option | Purpose |
|---|---|
| `--t-ready 5` `--t-start 3` `--t-transfer 15` `--t-release 3` `--t-watchdog 2` `--t-stop 1` | the time limits of your plant specification, in seconds |
| `--only R2 R8` | run selected rules |
| `--report record.md` | Markdown report for the commissioning record |
| `--json` | machine-readable output |
| `--strict` | exit code 2 if a rule could not be tested |

Exit code 0 = no rule failed, 1 = at least one rule failed.

## It moves your conveyor

The tool writes the request signals. On a real station that starts the conveyor.

- For any endpoint that is not this computer it refuses to run without `--station-is-safe`.
- Run it with the station in a test state, clear of people and loads.
- It always writes both request bits back to FALSE when it ends, also after an error.
- It is a commissioning aid. It is **not** a safety function and no substitute for the safety
  validation of the station.

## Tested

The test suite starts the reference station over real OPC UA, once conforming and once for each of
eight built-in defects, and expects the tester to report exactly the broken rule:

| Defect in the station | Reported under |
|---|---|
| Ready on without a request | R1 |
| Ready far too late | R2 |
| conveyor starts before the vehicle is in position | R3 |
| TransferActive still on with TransferDone | R4 |
| outputs not cleared after release | R5 |
| no heartbeat watchdog | R8 |
| fault not cleared by the acknowledge | R9 |
| conveyor keeps running when the vehicle leaves | R10 |

A transfer that is too short to judge R8 or R10 is reported as "not testable", not as a pass.

## Limits

- **Not tested against a real PLC.** Only against the bundled reference station. The S7-1500 and
  TwinCAT maps are examples; node ids depend on the project.
- **No OPC UA security.** The tool connects anonymously without encryption. A PLC that demands
  certificates or a login cannot be tested yet.
- **Polling.** Signals are read every 25 ms by default, so measured times are accurate to a few
  tens of milliseconds plus the network. It cannot see events shorter than that.
- **The rules are this project's own** ([SPEC](../../docs/SPEC.md)), shaped after SEMI E84. They
  are not a standard of any body, and a station built to another handshake will fail them.
- **One vehicle, one station.** No carrier identification, no multi-lane stations.

## Licence

MIT
