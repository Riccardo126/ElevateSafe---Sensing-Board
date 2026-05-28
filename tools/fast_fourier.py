import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch, get_window, spectrogram, find_peaks

choose = "test\\0210suugiugiu-1000hz.csv"
names = ['Program Time [s]', 'z', 'slow z', 'fast z', 'hall', 'v']
df = pd.read_csv(choose, header=0)

# Estrai segnale e rimuovi offset
segnale = df['fast z'].dropna().values.astype(float)
segnale = segnale - np.mean(segnale)

# CALCOLA fs dal CSV stesso
timestamps = df['Program Time [s]'].dropna().values
dt_array = np.diff(timestamps)
dt_mean = np.mean(dt_array)
fs = 1.0 / dt_mean
ts = 1.0 / fs
N = len(segnale)

print(f"\n=== ACQUISIZIONE ===")
print(f"Sample rate misurato: {fs:.2f} Hz")
print(f"Campioni totali: {N}")
print(f"Durata: {timestamps[-1] - timestamps[0]:.2f} s")

if N < 16:
    raise SystemExit('Segnale troppo corto')

# Windowing
win_name = 'hann'
win = get_window(win_name, N)

# === Welch PSD (robusto per segnali non periodici) ===
nperseg = min(512, N)
noverlap = nperseg // 2
f, Pxx = welch(segnale, fs=fs, window=win_name, nperseg=nperseg, 
                noverlap=noverlap, scaling='density')
Pxx_db = 10.0 * np.log10(Pxx + 1e-20)

# Peak picking migliorato
threshold = np.percentile(Pxx_db, 85)
peaks, props = find_peaks(Pxx_db, height=threshold, prominence=1.5)
if len(peaks) > 0:
    f_max = f[peaks[np.argmax(props['peak_heights'])]]
else:
    f_max = f[np.argmax(Pxx_db)]

print(f"Frequenza dominante: {f_max:.2f} Hz")

# === True FFT con normalizzazione corretta ===
pad_len = 2 ** int(np.ceil(np.log2(N)))
X = np.fft.rfft(segnale * win, n=pad_len)
freqs_fft = np.fft.rfftfreq(pad_len, d=1.0 / fs)

# Normalizzazione corretta col windowing
window_correction = np.sum(win) / N
mag_db = 20.0 * np.log10(np.abs(X) / np.sum(win) + 1e-20)

# Calcola magnitudine lineare single-sided corretta per la window
# X è il risultato di rfft su (segnale*win) con lunghezza pad_len
mag = np.abs(X) * 2.0 / np.sum(win)
# DC non va raddoppiato
mag[0] /= 2.0
# Se pad_len è pari, l'ultimo bin è Nyquist e non va raddoppiato
if pad_len % 2 == 0:
    mag[-1] /= 2.0

# === Spectrogram (anomalie time-varying) ===
f_s, t_s, Sxx = spectrogram(segnale, fs=fs, window='hann', 
                             nperseg=256, noverlap=200, mode='magnitude')
Sxx_db = 10.0 * np.log10(Sxx + 1e-20)

# Rileva anomalie
anomaly_threshold = np.percentile(Sxx_db, 95)
anomaly_times = t_s[np.max(Sxx_db, axis=0) > anomaly_threshold]
print(f"Anomalie rilevate: {len(anomaly_times)} tempi")

# === Plot ===
fig, axes = plt.subplots(3, 1, figsize=(14, 10))

# PSD
axes[0].plot(f, Pxx_db, 'b-', linewidth=1.5, label='Welch PSD')
axes[0].plot(freqs_fft, mag_db, 'orange', alpha=0.5, label='FFT (rfft)')
axes[0].axvline(f_max, color='g', linestyle='--', label=f'f_max={f_max:.1f}Hz')
axes[0].set_ylabel('PSD (dB)')
axes[0].set_title('Power Spectral Density')
axes[0].legend()
axes[0].grid(True, alpha=0.3)
axes[0].set_xlim(0, min(fs/2, f_max * 3))

# Spectrogram
pcm = axes[1].pcolormesh(t_s, f_s, Sxx_db, shading='auto', cmap='viridis')
axes[1].set_ylabel('Frequency (Hz)')
axes[1].set_title('Spectrogram — anomalie in bianco/giallo')
axes[1].set_ylim(0, min(fs/2, f_max * 3))
plt.colorbar(pcm, ax=axes[1], label='dB')

# FFT Magnitude (ampiezza lineare)
axes[2].plot(freqs_fft, mag, 'r-', linewidth=1.0)
axes[2].set_xlabel('Frequency (Hz)')
axes[2].set_ylabel('Amplitude')
axes[2].set_title('FFT Magnitude (linear)')
axes[2].grid(True, alpha=0.3)
axes[2].set_xlim(0, fs/2)

plt.tight_layout()
plt.show()