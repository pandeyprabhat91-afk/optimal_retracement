"""Paper figures from results.csv + optimal_results.csv -> output/retrace/figs/."""

import csv
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = str(Path(__file__).resolve().parents[2] / "output" / "retrace")
FIG = os.path.join(OUT, "figs")
os.makedirs(FIG, exist_ok=True)

rows = list(csv.DictReader(open(os.path.join(OUT, "results.csv"))))
opt = list(csv.DictReader(open(os.path.join(OUT, "optimal_results.csv"))))


def sel(log, exp, h):
    return [
        r
        for r in rows
        if r["log"] == log and r["exp"] == exp and float(r["cutoff"]) == h
    ]


def fig1():
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4), sharey=True)
    for k, log in enumerate(("A", "B")):
        r = sel(log, "exp1", 30.0)
        ax[k].bar([x["combo"] for x in r], [float(x["ate"]) for x in r])
        ax[k].set_yscale("log")
        ax[k].set_title(f"Flight {log}, 30 s outage")
        ax[k].set_ylabel("ATE (m, log scale)" if k == 0 else "")
    fig.suptitle("Exp1 ATE by sensor combo")
    plt.savefig(os.path.join(FIG, "fig1_ate30_AB.png"), dpi=150, bbox_inches="tight")
    plt.close()


def fig2():
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    for c in ("C1", "C3", "C4", "C6"):
        hs, vs = [], []
        for h in (10.0, 30.0, 60.0):
            r = sel("A", "exp1", h)
            d = {x["combo"]: float(x["ate"]) for x in r}
            hs.append(h)
            vs.append(d[c])
        ax[0].loglog(hs, vs, marker="o", label=c)
    ax[0].set_xlabel("Outage horizon (s)")
    ax[0].set_ylabel("ATE (m)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Drift growth — Flight A")
    r = sel("A", "exp1", 60.0)
    dA = {x["combo"]: float(x["yaw_rmse"]) for x in r}
    r = sel("B", "exp1", 60.0)
    dB = {x["combo"]: float(x["yaw_rmse"]) for x in r}
    ax[1].bar(
        ["A: C1", "A: C2", "B: C1", "B: C2"],
        [dA["C1"], dA["C2"], dB["C1"], dB["C2"]],
    )
    ax[1].set_ylabel("Yaw RMSE (rad)")
    ax[1].set_title("Compass helps A, hurts B — 60 s")
    plt.savefig(os.path.join(FIG, "fig2_growth_yaw.png"), dpi=150, bbox_inches="tight")
    plt.close()


def fig3():
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    ax[0].bar([r["combo"] for r in opt], [float(r["ate"]) for r in opt])
    ax[0].set_yscale("log")
    ax[0].set_ylabel("ATE (m, log scale)")
    ax[0].set_title("Optimal-world removal ablation")
    r = sel("A", "exp2", 30.0)
    cs = [x["combo"] for x in r]
    x = range(len(cs))
    ax[1].bar(
        [i - 0.2 for i in x],
        [float(v["home_miss"]) for v in r],
        width=0.4,
        label="home",
    )
    ax[1].bar(
        [i + 0.2 for i in x],
        [float(v["cross_track"]) for v in r],
        width=0.4,
        label="x-track",
    )
    ax[1].set_xticks(list(x))
    ax[1].set_xticklabels(cs)
    ax[1].set_ylabel("metres")
    ax[1].legend(fontsize=8)
    ax[1].set_title("Exp2 return, Flight A 30 s (all < 2 m)")
    plt.savefig(os.path.join(FIG, "fig3_opt_exp2.png"), dpi=150, bbox_inches="tight")
    plt.close()


def fig4():
    fig, ax = plt.subplots(figsize=(10, 2.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 3.2)
    ax.axis("off")
    boxes = [
        (0.2, 1.0, 1.6, 1.2, "Logged\nsensors"),
        (2.4, 1.0, 1.6, 1.2, "Static-gated\ncalibration"),
        (4.6, 1.0, 1.6, 1.2, "Quaternion\nstrapdown"),
        (6.8, 1.6, 1.6, 1.2, "Mag / baro /\ndrag correct"),
        (6.8, 0.2, 1.6, 1.2, "2 Hz\nbreadcrumb"),
    ]
    for xb, yb, w, h, txt in boxes:
        ax.add_patch(
            FancyBboxPatch(
                (xb, yb), w, h, boxstyle="round,pad=0.05", fc="#e8eef7", ec="#333333"
            )
        )
        ax.text(xb + w / 2, yb + h / 2, txt, ha="center", va="center", fontsize=9)
    for x1, y1, x2, y2 in [
        (1.8, 1.6, 2.4, 1.6),
        (4.0, 1.6, 4.6, 1.6),
        (6.2, 1.6, 6.8, 2.2),
        (6.2, 1.6, 6.8, 0.8),
    ]:
        ax.add_patch(
            FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="->", mutation_scale=12)
        )
    ax.add_patch(
        FancyArrowPatch((8.4, 2.2), (9.6, 2.2), arrowstyle="->", mutation_scale=12)
    )
    ax.add_patch(
        FancyArrowPatch((8.4, 0.8), (9.6, 0.8), arrowstyle="->", mutation_scale=12)
    )
    ax.text(9.0, 2.45, "Exp1 home vector", ha="center", fontsize=8)
    ax.text(9.0, 1.05, "Exp2 reverse pursuit", ha="center", fontsize=8)
    ax.set_title("Retrace estimation pipeline", fontsize=11)
    plt.savefig(os.path.join(FIG, "fig4_arch.png"), dpi=150, bbox_inches="tight")
    plt.close()


fig1()
fig2()
fig3()
fig4()
print("figs done:", sorted(os.listdir(FIG)))
