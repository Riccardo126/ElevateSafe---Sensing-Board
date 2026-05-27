import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, ifft

choose = "test/sugiudiag.csv"
names = ['Program Time [s]', 'x', 'y', 'z', 'slow x', 'slow y', 'slow z', 'fast x', 'fast y', 'fast z', 'hall']
df = pd.read_csv(choose, names=names, header=0)

# 1. Estraiamo i dati, togliamo i NaN e rimuoviamo l'offset (la media/gravità)
segnale = df['fast z'].dropna().values
segnale = segnale - np.mean(segnale)

# 2. Parametri di campionamento
sr = 1000.0
ts = 1.0 / sr
N = len(segnale)

# 3. Creiamo l'asse del tempo t dinamico (lungo esattamente quanto N)
t = np.arange(N) * ts

# 4. Calcolo FFT 
X = fft(segnale)
n = np.arange(N)
T = N / sr
freq = n / T 

half = N // 2
magnitudes = np.abs(X)[:half]
frequencies = freq[:half]

# ==========================================
# NUOVO: CALCOLO F_MAX (TEOREMA DI NYQUIST)
# ==========================================
picco_massimo = np.max(magnitudes)

# Impostiamo il rumore di fondo al 5% del picco più alto.
# Se il tuo grafico è molto rumoroso, puoi alzare questo valore a 0.08 (8%) o 0.10 (10%)
soglia_rumore = picco_massimo * 0.05 

# Trovo tutti gli indici in cui il segnale supera il rumore
indici_sopra_soglia = np.where(magnitudes > soglia_rumore)[0]

# Prendo l'ULTIMO indice utile (la frequenza più alta che non è rumore)
if len(indici_sopra_soglia) > 0:
    indice_fmax = indici_sopra_soglia[-1]
    f_max = frequencies[indice_fmax]
else:
    f_max = 0.0

# Calcolo Nyquist
campionamento_teorico = f_max * 2

print("\n=== RISULTATI TEOREMA DI NYQUIST ===")
print(f"Frequenza Massima (f_max):   {f_max:.2f} Hz")
print(f"Campionamento Nyquist (>2x): {campionamento_teorico:.2f} Hz")
print(f"Campionamento Consigliato:   {campionamento_teorico * 1.2:.2f} Hz (con margine di sicurezza)")
print("====================================\n")

# ==========================================
# 5. Plotting
# ==========================================
plt.figure(figsize=(14, 6))

plt.subplot(211)
plt.stem(frequencies, magnitudes, 'b', markerfmt=" ", basefmt="-b")

# Aggiungo le linee guida per farti capire cosa ha deciso l'algoritmo
plt.axhline(y=soglia_rumore, color='r', linestyle='--', alpha=0.7, label='Soglia Rumore (5%)')
plt.axvline(x=f_max, color='g', linestyle='-', linewidth=2, label=f'f_max ({f_max:.1f} Hz)')

plt.xlabel('Freq (Hz)', fontsize=18)
plt.ylabel('FFT Amplitude |X(freq)|', fontsize=18)
plt.title('Spettro Frequenze e Taglio Nyquist', fontweight='bold', fontsize=20)
plt.tick_params(axis='both', which='major', labelsize=16)

# Faccio uno zoom intelligente: inquadro da 0 fino a poco dopo la f_max
# (uso max tra 50 e f_max+20 per evitare che il grafico sia troppo stretto se f_max è piccola)
plt.xlim(0, max(50, f_max + 20)) 
plt.legend(fontsize=14)

plt.subplot(212)
# Usiamo np.real() per plottare solo l'ampiezza fisica, evitando il warning
plt.plot(t, np.real(ifft(X)), 'r')
plt.xlabel('Time (s)', fontsize=18)
plt.ylabel('Amplitude', fontsize=18)
plt.title('Segnale Ricostruito (IFFT)', fontweight='bold', fontsize=20)
plt.tick_params(axis='both', which='major', labelsize=16)


plt.tight_layout()
plt.show()