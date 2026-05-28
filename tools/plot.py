import pandas as pd
import matplotlib.pyplot as plt


CSV_PATH = "test\\0210suugiugiu-1000hz.csv"

# Program Time [s],accel x,accel y,accel z,ema slow x,ema slow y,ema slow z,ema fast x,ema fast y,ema fast z,data 9

df = pd.read_csv(CSV_PATH, header=0)
# drop first and last row
df = df.iloc[1:-100]

time_s = pd.to_numeric(df["Program Time [s]"], errors="coerce")
time_s = time_s - time_s.iloc[0]


#print min and max of all columns
print("=== MIN/MAX ===")
for col in df.columns:
    min_val = pd.to_numeric(df[col], errors="coerce").min()
    max_val = pd.to_numeric(df[col], errors="coerce").max()
    print(f"{col}: min={min_val:.3f}, max={max_val:.3f}")
    
# clamp values to min/max    
df[col] = pd.to_numeric(df[col], errors="coerce").clip(lower=min_val, upper=max_val)


fig, ax = plt.subplots(figsize=(14, 4))

ax.plot(time_s, pd.to_numeric(df["z"]), label="AccelZ", linewidth=0.9)
ax.plot(time_s, pd.to_numeric(df["slow z"]), label="emaZslow", linewidth=0.9)
ax.plot(time_s, pd.to_numeric(df["fast z"]), label="emaZfast", linewidth=0.9)
#hall = pd.to_numeric(df["hall"], errors="coerce")
#hall_norm = (hall - hall.min()) / (hall.max() - hall.min())
#ax.plot(time_s, hall_norm, label="emaHall (norm)", linewidth=1.1)

ax.set_title("AccelZ, emaZslow, emaZfast, emaHall")
ax.set_xlabel("Tempo [s]")
ax.set_ylabel("Valore")
ax.grid(True, linewidth=0.3, alpha=0.5)
ax.legend(loc="upper right", fontsize=8)

fig.tight_layout()
fig.savefig("plot_z_hall_simple.png", dpi=120)
plt.show()


