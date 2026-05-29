import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch, butter, sosfiltfilt, get_window, spectrogram, find_peaks

# Improve plot readability: larger fonts for titles, labels, ticks and legends
plt.rcParams.update({
    'font.size': 12,
    'axes.titlesize': 16,
    'axes.labelsize': 13,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 12,
    'figure.titlesize': 15
})

choose = "test\\012susudiag-1000hz-migliore.csv"
names = ['Program Time [s]', 'z', 'slow z', 'fast z', 'hall', 'v']
df = pd.read_csv(choose, header=0)

# Estrai segnale
segnale = df['fast z'].dropna().values.astype(float)

# Calcola fs dal CSV
timestamps = df['Program Time [s]'].dropna().values
fs = 1.0 / np.mean(np.diff(timestamps))
N = len(segnale)

print(f"\n=== ACQUISIZIONE ===")
print(f"Sample rate misurato: {fs:.2f} Hz")
print(f"Campioni totali: {N}")
print(f"Durata: {timestamps[-1] - timestamps[0]:.2f} s")

# HIGH-PASS FILTER (rimuovi drift a bassissima frequenza)
sos = butter(4, 0.5, btype='high', fs=fs, output='sos')
segnale = sosfiltfilt(sos, segnale)

# Detrend
segnale = segnale - np.mean(segnale)

if N < 16:
    raise SystemExit('Segnale troppo corto')

# ============= FFT migliore =============
# Applica una finestra (Hann) per ridurre sidelobes
win = np.hanning(N)
segnale_windowed = segnale * win

# FFT diretta
X = np.fft.rfft(segnale_windowed)
freqs = np.fft.rfftfreq(N, d=1.0/fs)

# Magnitudine single-sided corretta per la finestra
magnitude = np.abs(X) * 2.0 / np.sum(win)
magnitude[0] /= 2.0
if N % 2 == 0:
    magnitude[-1] /= 2.0

# dB
magnitude_db = 20.0 * np.log10(magnitude + 1e-20)

# Frequenza dominante dalla FFT migliorata (windowed, single-sided)
f_fft_peak = freqs[np.argmax(magnitude)]
print(f"Frequenza dominante (FFT migliorato): {f_fft_peak:.2f} Hz")

# ============= WELCH (la bella) =============
f, Pxx = welch(segnale, fs=fs, window='hann', nperseg=512, noverlap=256)
Pxx_db = 10.0 * np.log10(Pxx + 1e-20)

# Rileva picchi
threshold = np.percentile(Pxx_db, 85)
peaks, props = find_peaks(Pxx_db, height=threshold, prominence=1.5)
if len(peaks) > 0:
    f_max = f[peaks[np.argmax(props['peak_heights'])]]
else:
    f_max = f[np.argmax(Pxx_db)]

print(f"Frequenza dominante (Welch): {f_max:.2f} Hz")

# ============= SPECTROGRAM =============
f_s, t_s, Sxx = spectrogram(segnale, fs=fs, window='hann', 
                             nperseg=256, noverlap=200, mode='magnitude')
Sxx_db = 10.0 * np.log10(Sxx + 1e-20)

anomaly_threshold = np.percentile(Sxx_db, 95)
anomaly_times = t_s[np.max(Sxx_db, axis=0) > anomaly_threshold]
print(f"Anomalie rilevate: {len(anomaly_times)} tempi")

# Frequenza suggerita dallo spettrogramma: bin con energia media massima
spectrogram_profile = np.mean(Sxx_db, axis=1)
f_spec = f_s[np.argmax(spectrogram_profile)]
print(f"Frequenza suggerita dallo spettrogramma: {f_spec:.2f} Hz")

# ============= PLOT =============
fig, axes = plt.subplots(3, 1, figsize=(14, 11))

'''
# Plot 1: Confronto FFT semplice vs Welch 
axes[0].plot(freqs, magnitude_db, 'orange', linewidth=0.5, alpha=0.7, label='FFT (hann windowed, db magnitudes)')
axes[0].plot(f, Pxx_db, 'b-', linewidth=2.0, label='Welch PSD')
axes[0].axvline(f_max, color='g', linestyle='--', linewidth=2, label=f'f_max={f_max:.2f}Hz')
axes[0].axvline(f_spec, color='m', linestyle=':', linewidth=2, label=f'f_spec={f_spec:.2f}Hz')
axes[0].set_ylabel('Magnitude (dB)')
axes[0].set_title('FFT & Welch')
axes[0].legend(loc='upper right', fontsize=12)
axes[0].grid(True, alpha=0.3)
axes[0].set_xlim(0, min(fs/2, 10))
axes[0].set_ylim(-80, -20) '''

# Plot 1: Segnale filtrato
time = np.arange(N) * (1/fs)
axes[0].plot(time, segnale, color='tab:red')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Acceleration (m/s²),')
axes[0].set_title('Original Signal (detrended)')

# Plot 2: Spectrogram
pcm = axes[1].pcolormesh(t_s, f_s, Sxx_db, shading='auto', cmap='viridis')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Frequency (Hz)')
axes[1].set_title('Spectrogram')
axes[1].set_ylim(0, 10)
cbar = plt.colorbar(pcm, ax=axes[1], label='dB')
cbar.set_label('dB', fontsize=12)
cbar.ax.tick_params(labelsize=11)

# Plot 3: FFT normale (base, no dB)
fft_basic = np.fft.fft(segnale)
freqs_basic = np.fft.fftfreq(N, d=1.0 / fs)
pos_mask = freqs_basic >= 0
freqs_basic = freqs_basic[pos_mask]
amp_basic = np.abs(fft_basic[pos_mask]) / N
# Rileva picchi: usare le frequenze della FFT 'basic' (freqs_basic)
# Scegli un percentile più alto per ignorare rumore di fondo (es. 90th)
fftthreshold = np.percentile(amp_basic, 90)
peaks, props = find_peaks(amp_basic, height=fftthreshold, prominence=(fftthreshold * 0.5))
if len(peaks) > 0:
    # scegli il picco con ampiezza maggiore tra quelli rilevati
    peak_idx = peaks[np.argmax(props['peak_heights'])]
    f_max = freqs_basic[peak_idx]
else:
    # fallback: prende il bin di massima ampiezza
    peak_idx = np.argmax(amp_basic)
    f_max = freqs_basic[peak_idx]
print(f"Peak basic amp: {amp_basic[peak_idx]:.6e}")

axes[2].plot(freqs_basic, amp_basic, 'r-', linewidth=0.8)
axes[2].set_xlabel('Frequency (Hz)')
axes[2].set_ylabel('Amplitude')
axes[2].set_title('normal FFT')
axes[2].axvline(f_max, color='g', linestyle='--', linewidth=2, label=f'best={f_max:.2f}Hz')
axes[2].grid(True, alpha=0.3)
axes[2].set_xlim(0, fs / 6)
print(f"Frequenza dominante (normal FFT): {freqs_basic[np.argmax(amp_basic)]:.2f} Hz")
print(f"Frequenza migliore 700percentile (normal FFT, picchi): {f_max:.2f} Hz")



plt.tight_layout()
plt.show()