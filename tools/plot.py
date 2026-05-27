import pandas as pd
import matplotlib.pyplot as plt


CSV_PATH = "test/finaldata.csv"


def normalize_name(name: str) -> str:
    return "".join(ch for ch in str(name).lower() if ch.isalnum())


def find_column(df: pd.DataFrame, aliases):
    normalized = {normalize_name(c): c for c in df.columns}
    for alias in aliases:
        key = normalize_name(alias)
        if key in normalized:
            return normalized[key]
    return None


def minmax_0_1(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    smin = s.min()
    smax = s.max()
    if pd.isna(smin) or pd.isna(smax) or smax == smin:
        return s * 0.0
    return (s - smin) / (smax - smin)


def plot_group(ax, time_s, df, specs, title, ylabel):
    plotted = 0
    for label, aliases, style in specs:
        col = find_column(df, aliases)
        if col is None:
            continue

        y = pd.to_numeric(df[col], errors="coerce")
        if label == "hall_norm":
            y = minmax_0_1(y)

        ax.plot(time_s, y, label=label, **style)
        plotted += 1

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.axhline(0.0, color="gray", linewidth=0.4, linestyle="--")

    if plotted > 0:
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "Nessuna colonna trovata", ha="center", va="center", transform=ax.transAxes)


df = pd.read_csv(CSV_PATH)

time_col = find_column(df, ["Program Time [s]", "ProgramTime[s]", "Timestamp", "time", "t"])
if time_col is None:
    time_s = pd.Series(range(len(df)), dtype="float64")
else:
    time_s = pd.to_numeric(df[time_col], errors="coerce")
    t0 = time_s.dropna().iloc[0] if time_s.notna().any() else 0.0
    time_s = time_s - t0

fig1, ax1 = plt.subplots(figsize=(14, 4))
plot_group(
    ax1,
    time_s,
    df,
    specs=[
        ("AccelZ", ["AccelZ", "accel z", "accel_z"], {"linewidth": 0.9}),
        ("emaZslow", ["emaZslow", "ema slow z", "ema_slow_z"], {"linewidth": 0.9}),
        ("emaZfast", ["emaZfast", "ema fast z", "ema_fast_z"], {"linewidth": 0.9}),
        ("hall_norm", ["emaHall", "hall", "floorHall"], {"linewidth": 1.0}),
    ],
    title="Plot 1: AccelZ, emaZslow, emaZfast, hall normalizzato",
    ylabel="g / norm",
)
ax1.set_xlabel("Tempo [s]")
fig1.tight_layout()
fig1.savefig("plot_1_z_hall.png", dpi=120)

fig2, ax2 = plt.subplots(figsize=(14, 4))
plot_group(
    ax2,
    time_s,
    df,
    specs=[
        ("AccelX", ["AccelX", "accel x", "accel_x"], {"linewidth": 0.9}),
        ("AccelY", ["AccelY", "accel y", "accel_y"], {"linewidth": 0.9}),
        ("emaXslow", ["emaXslow", "ema slow x", "ema_slow_x"], {"linewidth": 0.9}),
        ("emaYslow", ["emaYslow", "ema slow y", "ema_slow_y"], {"linewidth": 0.9}),
        ("emaXfast", ["emaXfast", "ema fast x", "ema_fast_x"], {"linewidth": 0.9}),
        ("emaYfast", ["emaYfast", "ema fast y", "ema_fast_y"], {"linewidth": 0.9}),
    ],
    title="Plot 2: X/Y raw + EMA slow/fast",
    ylabel="g",
)
ax2.set_xlabel("Tempo [s]")
fig2.tight_layout()
fig2.savefig("plot_2_xy_ema.png", dpi=120)

fig3, ax3 = plt.subplots(figsize=(14, 4))
plot_group(
    ax3,
    time_s,
    df,
    specs=[
        ("speed", ["speed", "cum_vZ", "velocity", "vel"], {"linewidth": 1.0}),
        ("position", ["pos", "positionZ", "position", "dist"], {"linewidth": 1.0}),
    ],
    title="Plot 3: Velocita e posizione",
    ylabel="m/s, m",
)
ax3.set_xlabel("Tempo [s]")
fig3.tight_layout()
fig3.savefig("plot_3_speed_position.png", dpi=120)

plt.show()


