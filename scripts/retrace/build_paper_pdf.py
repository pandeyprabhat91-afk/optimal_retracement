"""Research-format paper -> docs/GPS_LOSS_RETRACE_PAPER.pdf (reportlab). Mirrors docx."""

import os
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)
from reportlab.lib import colors

ROOT = Path(__file__).resolve().parents[2]
FIG = str(ROOT / "output" / "retrace" / "figs")
OUT = str(ROOT / "docs" / "GPS_LOSS_RETRACE_PAPER.pdf")

styles = getSampleStyleSheet()
title_s = ParagraphStyle(
    "Title2", parent=styles["Title"], fontSize=20, leading=24, spaceAfter=2
)
sub_s = ParagraphStyle(
    "Sub", parent=styles["Normal"], fontSize=11, leading=14, alignment=1, spaceAfter=2
)
h1 = ParagraphStyle(
    "H1",
    parent=styles["Heading1"],
    fontSize=14,
    leading=17,
    spaceBefore=10,
    spaceAfter=4,
)
h2 = ParagraphStyle(
    "H2",
    parent=styles["Heading2"],
    fontSize=12,
    leading=15,
    spaceBefore=8,
    spaceAfter=4,
)
body = ParagraphStyle(
    "Body",
    parent=styles["Normal"],
    fontSize=10,
    leading=13,
    alignment=4,
    spaceAfter=3,
    spaceBefore=0,
)
cap = ParagraphStyle(
    "Cap",
    parent=styles["Normal"],
    fontSize=8.5,
    leading=11,
    alignment=1,
    spaceBefore=2,
    spaceAfter=8,
)
ref_s = ParagraphStyle(
    "Ref", parent=styles["Normal"], fontSize=9, leading=12, leftIndent=14, spaceAfter=3
)

story = []


def T(t):
    story.append(Paragraph(t, title_s))


def P(t):
    story.append(Paragraph(t, body))


def F(path, caption, w=5.8 * inch):
    story.append(Image(path, width=w, height=w * 0.62, kind="proportional"))
    story.append(Paragraph(caption, cap))


def TB(headers, rows):
    data = [headers] + rows
    tb = Table(data, repeatRows=1)
    tb.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#d9e2f3")),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.grey),
                ("FONTSIZE", (0, 0), (-1, -1), 8),
                ("TOPPADDING", (0, 0), (-1, -1), 2),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                (
                    "ROWBACKGROUNDS",
                    (0, 1),
                    (-1, -1),
                    [colors.white, colors.HexColor("#f2f6fc")],
                ),
            ]
        )
    )
    story.append(tb)
    story.append(Spacer(1, 6))


T("Which Sensors Bring a Drone Home?")
T("GPS-Loss Path Retrace: Real Logs, Perfect Worlds, Live SITL")
story.append(
    Paragraph("MTech Capstone Project, GPS-Denied Drone Navigation, IIT Madras", sub_s)
)
story.append(Paragraph("September 2026, branch <i>optimal-retracement</i>", sub_s))

story.append(Paragraph("Abstract", h1))
P(
    "When a drone loses GPS, it must navigate home on sensors its flight controller logged before the outage. "
    "This paper measures which sensors matter. Six cumulative sensor sets (C1: gyro plus accelerometer, up to C6: "
    "adding compass, barometer, thrust, battery and zero-velocity updates) are tested three ways: replaying two real "
    "ArduPilot logs under 10/30/60 s simulated outages, ablating factors in a perfect analytic world, and flying a "
    "square live in PX4 SITL with raw and with perfect sensors. "
    "Gyro plus accelerometer alone retrace 801 m to 0.21 m when sensors are exact. On real logs the thrust-based drag "
    "model is the biggest win (30 s error 119 to 8.8 m). SITL repeats the ranking and proves the 50x scale gap is "
    "measured sensor dirt, not bad math: same square retraces to 6 mm with exact sensors."
)

story.append(Paragraph("1. The question in plain terms", h1))
P(
    "Picture a quadcopter 500 m out when GPS dies. Two ways home exist. Dead reckoning: integrate gyro and accel "
    "from the last known fix and fly the resulting home arrow (Exp1). Breadcrumb reversal: re-fly the logged trail "
    "backwards, steering each step toward the next crumb (Exp2). Exp1 tests how fast position error grows. Exp2 tests "
    "whether pursuit converges despite that error. Both run without any GPS after the cutoff."
)
P(
    "Jargon used below, defined once. Strapdown: attitude kept by integrating gyro in software, no gimbal. "
    "Tilt-compensated compass: yaw from magnetometer after removing roll/pitch tilt. ZUPT: zero-velocity update, "
    "trusting that near-zero speed means stopped. HDop: GPS geometry quality, lower is better. EKF: the onboard "
    "estimation filter fusing all sensors. OFFBOARD: PX4 mode following external setpoints. ATE: average trajectory "
    "error against truth. VRE: vibration-rectification bias, a DC shift shaking imprints on MEMS sensors."
)

story.append(Paragraph("2. Combos, gates, pass bars", h1))
P(
    "Combos are cumulative: each adds one aid to the previous set. Gates pick clean GPS truth windows (HDop under "
    "1.0, 10 or more satellites, speed under 0.3 m/s for calibration). A 30 s run passes Exp1 under 2 percent drift "
    "(about 5 m on these legs); Exp2 passes under 5 m home miss."
)
TB(
    ["Combo", "Adds", "Why it should help"],
    [
        ["C1", "gyro + accel (core)", "minimum for any inertial navigation"],
        ["C2", "+ compass yaw", "stops yaw random walk"],
        ["C3", "+ barometer", "vertical channel otherwise drifts freely"],
        ["C4", "+ thrust drag model", "rotor drag caps velocity error"],
        ["C5", "+ battery current", "thrust scaling when PWM saturates"],
        ["C6", "+ ZUPT", "kills drift during genuine hover"],
    ],
)

story.append(Paragraph("3. Data and signals", h1))
P(
    "Two ArduPilot DataFlash logs: Flight A (00000061.BIN, 48 MB) and Flight B (June 2026, 79 MB). Both carry IMU, "
    "magnetometer with hard-iron offsets, baro, GPS, attitude, motor PWM, throttle, vibration and battery at native "
    "rates. GPS is truth only; outages are simulated by withholding it. Flights differ in IMU quality, so results stay "
    "per flight. Two traps: attitude logs in degrees while everything else is radians, and pooling would hide the mechanism."
)

story.append(Paragraph("4. Method", h1))
story.append(Paragraph("4.1 One core, three testbeds", h2))
P(
    "A single quaternion strapdown core runs in all three testbeds, so differences in results come from data, not code. "
    "Static windows set gyro/accel biases, hover throttle and one compass offset, then the filter integrates "
    "attitude, rotates specific force, adds gravity and integrates velocity and position. Aids (compass 0.1 gain, "
    "baro 0.1 blend, drag damping, ZUPT) toggle by combo. Figure 1 shows the flow."
)
F(
    os.path.join(FIG, "fig4_arch.png"),
    "Figure 1. Retrace pipeline: calibration, propagation, per-combo aids, Exp1 vector and Exp2 pursuit.",
)
story.append(Paragraph("4.2 Offline replay (real logs)", h2))
P(
    "For each log, clean GPS windows seed 60 s GPS-denied runs per combo, scored at 10/30/60 s horizons against gated "
    "GPS truth (72 rows total: 6 combos by 3 horizons by 2 flights by 2 experiments). Exp2 flies the stored 2 Hz trail "
    "backwards with combo-derived heading noise. Script: run_matrix.py."
)
story.append(Paragraph("4.3 Analytic optimal world (perfect sensors)", h2))
P(
    "A 120 s scripted profile (hover, 8 m/s legs, banked 90 degree turns, climb, hover; 801 m) generates exact "
    "100 Hz gyro, accel, mag, baro and thrust. Removal runs F1-F6 delete one factor each, so leftover error is pure "
    "information loss. Script: optimal_world.py."
)
story.append(Paragraph("4.4 Live SITL square (raw sensors)", h2))
P(
    "PX4 SITL plus Gazebo Garden (taif_test4 world) plus MAVROS, flown as a 20 m square at 5 m (114.5 m; PX4 home "
    "miss 0.36 m). A commander node streams setpoints, engages OFFBOARD, arms and lands through MAVROS services. "
    "Takeoff is ramped because a step climb shook the simulated IMU into a 10 s storm. The estimator node ports the core "
    "with three measured adaptations: FLU/ENU to FRD/NED conversion with full-attitude init, tilt-aware accel bias, "
    "and spike skipping over 25 m/s2 or 3 rad/s. One recorded bag replays per combo against odom truth with "
    "timestamp interpolation. Scripts: commander.py, retrace_node.py, replay_all.sh, RUN_SITL_LOOP.ps1."
)
story.append(
    Paragraph("4.5 SITL optimal control (perfect sensors, flown geometry)", h2)
)
P(
    "Same square as 4.4, but sensors are machine-exact 100 Hz signals differentiated from the commanded profile "
    "(yaw fixed, cosine blends), not from noisy odometry. Same combos, same scorer. Any gap to 4.4 is sensor dirt by "
    "construction. Script: sitl_optimal.py."
)

story.append(Paragraph("5. Results", h1))
story.append(Paragraph("5.1 Perfect world: core pair suffices", h2))
TB(
    ["Run", "Missing", "ATE (m)", "Yaw err (rad)", "Reading"],
    [
        ["F1 full", "nothing", "0.21", "0.00004", "baseline over 801 m"],
        ["F4/F6", "compass / baro", "0.22", "0.00000", "no loss"],
        [
            "F2/F3",
            "gyro / accel",
            "232 / frozen",
            "0.075 / exact",
            "each core sensor necessary",
        ],
        ["F5", "thrust proxy", "365", "exact", "velocity aid matters even here"],
    ],
)
P(
    "With exact sensors, gyro plus accel do everything and compass plus baro add nothing. Each core sensor is "
    "individually necessary. This is the information-content ceiling everything else is judged against."
)
F(
    os.path.join(FIG, "fig3_opt_exp2.png"),
    "Figure 2. Optimal-world ablation, log scale (left); Exp2 return on Flight A 30 s leg (right).",
)
story.append(Paragraph("5.2 Real logs: drag wins", h2))
TB(
    ["Log/combo", "ATE 10 s", "ATE 30 s", "ATE 60 s"],
    [
        ["A C1 (IMU)", "11.9", "120.7", "460.8"],
        ["A C3 (+mag+baro)", "12.9", "119.0", "433.4"],
        ["A C4 (+drag)", "4.3", "8.8", "14.8"],
        ["A C6 (+ZUPT)", "3.9", "4.9", "7.3"],
        ["B C1 (IMU)", "2.4", "30.3", "223.2"],
        ["B C4 (+drag)", "1.7", "11.4", "43.1"],
        ["B C6 (+ZUPT)", "0.3", "0.6", "7.4"],
    ],
)
P(
    "Why drag wins: without it, velocity error integrates twice into position; with it, velocity error is capped, so "
    "position grows linearly instead of quadratically. Hence 13x on A at 30 s. Battery adds nothing anywhere. ZUPT "
    "needs real hover: 19x on B, nil on A. Two B runs pass the 5 m bar."
)
F(
    os.path.join(FIG, "fig1_ate30_AB.png"),
    "Figure 3. Exp1 ATE by combo, 30 s outage, log scale.",
)
F(
    os.path.join(FIG, "fig2_growth_yaw.png"),
    "Figure 4. Drift growth with outage length (left); yaw error at 60 s (right).",
)
story.append(Paragraph("5.3 Compass can hurt", h2))
P(
    "Mechanism: compass corrects yaw drift but injects its own bias. On A (drifty gyro) the trade wins (yaw 0.47 to "
    "0.23 rad). On B (superb gyro, 0.00 to 0.05 rad) an uncalibrated 0.3 rad mag bias loses (0.00 to 0.30 rad). "
    "Calibrate in the field or leave it out."
)
story.append(Paragraph("5.4 Breadcrumb always lands", h2))
P(
    "Why: pursuit re-aims every step, so position error never compounds; only heading error matters. Home miss under "
    "2 m and cross-track within 1.2 m for every combo on both flights (optimal: 0.02 m)."
)
story.append(Paragraph("5.5 Raw SITL repeats the ranking at 50x scale", h2))
TB(
    ["SITL combo (114 m)", "ATE 3D (m)", "Horiz (m)", "Vert (m)"],
    [
        ["C1 (IMU)", "2690", "2690", "23.9"],
        ["C2 (+mag)", "2677", "2677", "24.6"],
        ["C3 (+baro)", "2665", "2665", "0.3"],
        ["C4 (+drag)", "423", "423", "0.3"],
    ],
)
P(
    "Same order as real logs: compass nil, baro fixes vertical only (24 to 0.3 m), drag cuts horizontal 6.3x. "
    "Scale differs for measured reasons: +0.46 m/s2 cruise rectification bias, aliasing spikes to 1137 m/s2 and "
    "37 rad/s at 49 Hz, and EKF height wobble in truth. C5/C6 omitted deliberately (nil offline, no hover here). "
    "Do not quote SITL magnitudes as performance; quote the ranking."
)
story.append(Paragraph("5.6 Perfect SITL: 6 mm, case closed", h2))
TB(
    ["Control combo (80 m)", "ATE (m)", "Home (m)"],
    [
        ["C1 (IMU)", "0.006", "0.00"],
        ["C3 (+baro)", "0.005", "0.00"],
        ["C4 (+drag)", "4.97", "0.21"],
        ["C6 (+ZUPT)", "4.99", "0.00"],
    ],
)
P(
    "Same geometry, exact sensors: 6 mm. Drag costs 5 m here only because the truth has no drag physics and "
    "stop-and-go corners keep it transient (long-cruise F1 costs 0.2 m). Raw-vs-perfect on identical geometry is "
    "2690 m against 0.006 m. The gap is sensor dirt, and now it is a measured quantity rather than a suspicion."
)

story.append(Paragraph("6. Bugs found while testing", h1))
P(
    "Five, each with a number attached. Degrees consumed as radians (57x; fix collapsed errors 10 to 20x). "
    "World-frame quaternion delta used as body rate (yaw 1.54 to 0.001 rad). Unwrapped yaw RMSE near 35 rad, fixed "
    "circularly. A tautological Exp2 scorer rebuilt as pursuit. SITL accel bias assumed level rest against 4 to 6 "
    "degrees of tilt, leaking about 1 m/s2 after the first turn; the fix rotates gravity by init attitude."
)
P(
    "Limits: blind dead reckoning (8 to 43 m at 30 to 60 s) is still too poor to fly home on; breadcrumbs plus a "
    "velocity aid (flow or VIO) is the operational answer. Drag gain is untuned per airframe. Mag was not re-tested "
    "in SITL."
)

story.append(Paragraph("7. Conclusion", h1))
P(
    "Log gyro and accel fast; they are necessary and sufficient when clean. Log motor commands; the drag model on "
    "them is the biggest real-world win here. Calibrate the compass or drop it. Keep baro and a 2 Hz breadcrumb trail. "
    "Analytic truth, two real logs and a live square all vote the same way."
)

story.append(Paragraph("References", h1))
for ref in [
    "[1] ArduPilot, Dead Reckoning Failsafe, Copter documentation, ardupilot.org.",
    "[2] D. Titterton and J. Weston, <i>Strapdown Inertial Navigation Technology</i>, 2nd ed., IET, 2004.",
    "[3] P. Groves, <i>Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems</i>, 2nd ed., Artech House, 2013.",
    "[4] RIOTU Lab, gps_denied_navigation_sim: PX4 SITL and Gazebo GPS-denied benchmark, github.com/riotu-lab.",
]:
    story.append(Paragraph(ref, ref_s))

doc = SimpleDocTemplate(OUT, pagesize=A4, topMargin=36, bottomMargin=36)
doc.build(story)
print("saved", OUT)
