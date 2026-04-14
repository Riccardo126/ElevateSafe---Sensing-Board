import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

nuovo = "tools\sampling_2026-04-15_01-10-08.csv"

choose = nuovo
idx = 0 # 0 none, 1 avg, 2 gauss
window = 51
names = ["Timestamp","AccelX","AccelY","AccelZ","DoorHall","FloorHall"]
df = pd.read_csv(choose, names=names, header=0)

df["Timestamp"] = (df["Timestamp"] - df["Timestamp"].iloc[0]) * 1


def moving_average(signal, window):
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(signal, kernel, mode="same")


def gaussian_filter(signal, window, sigma=None):
    if sigma is None:
        sigma = window / 6.0
    x = np.arange(window) - (window - 1) / 2
    kernel = np.exp(-(x ** 2) / (2 * sigma ** 2))
    kernel /= kernel.sum()
    return np.convolve(signal, kernel, mode="same")


def apply_filter(signal, filter_type, window):
    if filter_type == "none":
        return signal
    if window < 3:
        return signal
    if window % 2 == 0:
        window += 1

    if filter_type == "avg":
        return moving_average(signal, window)
    if filter_type == "gauss":
        return gaussian_filter(signal, window)
    return signal

choices=["none", "avg", "gauss"]
for col in ["AccelX", "AccelY", "AccelZ"]:
    df[f"{col}_f"] = apply_filter(df[col].to_numpy(), choices[idx], window)

pixels_per_second = 1  # aumenta per più zoom orizzontale
fig_width = max(14, df["Timestamp"].iloc[-1] * pixels_per_second)
fig, axes = plt.subplots(3, 1, figsize=(fig_width, 7), sharex=True)
fig.suptitle(f"Accelerometro — {choose[35:-4]}", fontsize=13)
 
axes[0].plot(df["Timestamp"], df["AccelX"], color="#9EB8D3", linewidth=0.6, alpha=0.5, label="raw")
axes[0].plot(df["Timestamp"], df["AccelX_f"], color="#185FA5", linewidth=0.9, label="filtered")
axes[0].set_ylabel("AccelX (g)")
axes[0].axhline(0, color="gray", linewidth=0.4, linestyle="--")
axes[0].legend(loc="upper right", fontsize=8)
 
axes[1].plot(df["Timestamp"], df["AccelY"], color="#A7D4C8", linewidth=0.6, alpha=0.5, label="raw")
axes[1].plot(df["Timestamp"], df["AccelY_f"], color="#0F6E56", linewidth=0.9, label="filtered")
axes[1].set_ylabel("AccelY (g)")
axes[1].axhline(0, color="gray", linewidth=0.4, linestyle="--")
axes[1].legend(loc="upper right", fontsize=8)
 
# normalizziamo la z per sicurezza in [-1, 1]
#  x -min / max - min no in questo caso
#df["AccelZ"] /=  df["AccelZ"].abs().max() 
axes[2].plot(df["Timestamp"], df["AccelZ"], color="#D3B79E", linewidth=0.6, alpha=0.5, label="raw")
axes[2].plot(df["Timestamp"], df["AccelZ_f"], color="#854F0B", linewidth=0.9, label="filtered")
axes[2].set_ylabel("AccelZ (g)")
axes[2].axhline(0, color="gray", linewidth=0.4, linestyle="--")
axes[2].set_xlabel("Tempo (s)")
axes[2].legend(loc="upper right", fontsize=8)
print("ratio of 0 samples: ", (df["AccelZ"]==0.0).sum()/len(df["AccelZ"])) #PROBLEMO = dovrebbe avvicinarsi a 0
 
for ax in axes:
    ax.grid(True, linewidth=0.3, alpha=0.5)
 
plt.tight_layout()
plt.savefig(f"plot_lollo_in_mano_gauss_w11.png", dpi=100)
plt.show()
 


