import pandas as pd
import matplotlib.pyplot as plt


CSV_PATH = "test/finaldata.csv"


df = pd.read_csv(CSV_PATH)
# drop first and last row
df = df.iloc[1:-100]

time_s = pd.to_numeric(df["Program Time [s]"], errors="coerce")
time_s = time_s - time_s.iloc[0]



fig, ax = plt.subplots(figsize=(14, 4))

ax.plot(time_s, pd.to_numeric(df["accel z"], errors="coerce"), label="AccelZ", linewidth=0.9)
ax.plot(time_s, pd.to_numeric(df["ema slow z"], errors="coerce"), label="emaZslow", linewidth=0.9)
ax.plot(time_s, pd.to_numeric(df["ema fast z"], errors="coerce"), label="emaZfast", linewidth=0.9)
hall = pd.to_numeric(df["hall"], errors="coerce")
hall_norm = (hall - hall.min()) / (hall.max() - hall.min())
ax.plot(time_s, hall_norm, label="emaHall (norm)", linewidth=1.1)

ax.set_title("AccelZ, emaZslow, emaZfast, emaHall")
ax.set_xlabel("Tempo [s]")
ax.set_ylabel("Valore")
ax.grid(True, linewidth=0.3, alpha=0.5)
ax.legend(loc="upper right", fontsize=8)

fig.tight_layout()
fig.savefig("plot_z_hall_simple.png", dpi=120)
plt.show()


