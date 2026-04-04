import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

small = "tools\sampling_elevate_safe_2026-04-03_22-15-18.csv"
onoff = "tools\sampling_2026-04-04_17-18-33_on_off_boarding.csv"
updown = "tools\sampling_2026-04-04_17-20-04_up_and_down.csv"
choose = small # CAMBIA QUI PER CAMBIARE GRAFICO
names = ["Timestamp","AccelX","AccelY","AccelZ","DoorHall","FloorHall"]
df = pd.read_csv(choose, names=names, header=0)

df["Timestamp"] = (df["Timestamp"] - df["Timestamp"].iloc[0]) * 1

pixels_per_second = 1  # aumenta per più zoom orizzontale
fig_width = max(14, df["Timestamp"].iloc[-1] * pixels_per_second)
fig, axes = plt.subplots(3, 1, figsize=(fig_width, 7), sharex=True)
fig.suptitle(f"Accelerometro — {choose[35:-4]}", fontsize=13)
 
axes[0].plot(df["Timestamp"], df["AccelX"], color="#185FA5", linewidth=0.8)
axes[0].set_ylabel("AccelX (g)")
axes[0].axhline(0, color="gray", linewidth=0.4, linestyle="--")
 
axes[1].plot(df["Timestamp"], df["AccelY"], color="#0F6E56", linewidth=0.8)
axes[1].set_ylabel("AccelY (g)")
axes[1].axhline(0, color="gray", linewidth=0.4, linestyle="--")
 
# normalizziamo la z per sicurezza in [-1, 1]
#  x -min / max - min no in questo caso
df["AccelZ"] /=  df["AccelZ"].abs().max() 
axes[2].plot(df["Timestamp"], df["AccelZ"], color="#854F0B", linewidth=0.8)
axes[2].set_ylabel("AccelZ (g)")
axes[2].axhline(0, color="gray", linewidth=0.4, linestyle="--")
axes[2].set_xlabel("Tempo (s)")
print((df["AccelZ"]==0.0).sum()/len(df["AccelZ"])) #PROBLEMO = dovrebbe avvicinarsi a 0
 
for ax in axes:
    ax.grid(True, linewidth=0.3, alpha=0.5)
 
plt.tight_layout()
plt.savefig(f"plot_{choose[35:-4]}.png", dpi=100)
plt.show()
 