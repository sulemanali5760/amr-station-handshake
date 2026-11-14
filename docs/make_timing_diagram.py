"""Draw the handover signal sequence of docs/SPEC.md as a PLC-style timing diagram (docs/handover_timing.png).

The times are an example run, not limits; the limits are the rule table in SPEC.md.
"""
from pathlib import Path

import matplotlib.pyplot as plt

VEHICLE, STATION, INK, MUTED, GRID = "#1f6fb2", "#c2410c", "#1f2328", "#6e7781", "#d0d7de"
T_REQ, T_READY, T_DOCK, T_ACTIVE, T_DONE, T_REL, T_IDLE, T_END = 1.0, 1.6, 3.2, 3.5, 7.4, 8.4, 8.8, 10.0
HEARTBEAT = [(round(k * 0.25, 2), k % 2) for k in range(int(T_END / 0.25))]
# (signal, writer, level changes, line width), top to bottom: vehicle first, then station
SIGNALS = [
    ("AmrDockRequest", VEHICLE, [(0, 0), (T_REQ, 1), (T_REL, 0)], 3),
    ("AmrInPosition", VEHICLE, [(0, 0), (T_DOCK, 1), (T_REL, 0)], 3),
    ("AmrHeartbeat", VEHICLE, HEARTBEAT, 1.6),
    ("StationReady", STATION, [(0, 0), (T_READY, 1), (T_IDLE, 0)], 3),
    ("TransferActive", STATION, [(0, 0), (T_ACTIVE, 1), (T_DONE, 0)], 3),
    ("TransferDone", STATION, [(0, 0), (T_DONE, 1), (T_IDLE, 0)], 3),
    ("Fault", STATION, [(0, 0)], 3),
]
RULES = [  # (label, from, to, signal whose row the bracket sits above, label alignment)
    ("R2  request → Ready ≤ 5 s", T_REQ, T_READY, "StationReady", "left"),
    ("R5  release → idle ≤ 3 s", T_REL, T_IDLE, "StationReady", "right"),
    ("R4  in position → active ≤ 3 s", T_DOCK, T_ACTIVE, "TransferActive", "left"),
    ("R4  transfer ≤ 15 s", T_ACTIVE, T_DONE, "TransferDone", "center"),
]


def steps(changes):
    xs, ys = [], []
    for i, (t, level) in enumerate(changes):
        if i:
            xs.append(t)
            ys.append(ys[-1])
        xs.append(t)
        ys.append(level)
    return xs + [T_END], ys + [ys[-1]]


fig, ax = plt.subplots(figsize=(16, 9), dpi=100)
fig.patch.set_facecolor("white")
row_h, amp = 1.5, 0.62
base = {name: (len(SIGNALS) - 1 - i) * row_h for i, (name, *_ ) in enumerate(SIGNALS)}
for name, color, changes, lw in SIGNALS:
    xs, ys = steps(changes)
    scale = 0.6 if name == "AmrHeartbeat" else 1.0
    ax.plot(xs, [base[name] + y * amp * scale for y in ys], color=color, lw=lw, solid_capstyle="butt")
    ax.text(-0.2, base[name] + amp / 2, name, ha="right", va="center", fontsize=17, color=color, family="monospace")
ax.text(T_END + 0.15, base["AmrHeartbeat"] + amp / 2, "R8: stops changing\n→ station faults ≤ 2 s",
        va="center", fontsize=13, color=MUTED)
ax.text(T_END + 0.15, base["Fault"] + amp / 2, "R8, R10: set, conveyor\nstops; R9: cleared on release",
        va="center", fontsize=13, color=MUTED)

for label, t0, t1, row, align in RULES:
    y = base[row] + amp + 0.3
    ax.annotate("", xy=(t1, y), xytext=(t0, y), arrowprops=dict(arrowstyle="<->", color=INK, lw=1.3))
    x = {"left": t0, "right": t1, "center": (t0 + t1) / 2}[align]
    ax.text(x, y + 0.08, label, ha=align, va="bottom", fontsize=13, color=INK)
top = base["AmrDockRequest"] + row_h
ax.vlines((T_REQ, T_DOCK, T_DONE, T_REL), -0.25, top - 0.3, color=GRID, lw=1, ls=(0, (4, 4)), zorder=0)

ax.text(-3.6, top + 0.75, "Robot to conveyor handover: eight signals, every step has a time limit",
        fontsize=24, weight="bold", color=INK)
ax.text(-3.6, top + 0.2, "■ written by the robot", fontsize=15, color=VEHICLE)
ax.text(0.4, top + 0.2, "■ written by the station PLC", fontsize=15, color=STATION)
ax.text(-3.6, -0.9, "Also checked: R1 all off without a request · R3 nothing moves before docking · "
        "R6 a second handover · R7 a request withdrawn before docking", fontsize=13, color=MUTED)
ax.text(-3.6, -1.45, "pip install handover-check   ·   github.com/sulemanali5760/amr-station-handshake",
        fontsize=14, color=INK, family="monospace")
ax.set_xlim(-3.7, T_END + 2.6)
ax.set_ylim(-1.7, top + 1.3)
ax.axis("off")
out = Path(__file__).with_name("handover_timing.png")
fig.savefig(out, bbox_inches="tight", pad_inches=0.3)
print(out)
