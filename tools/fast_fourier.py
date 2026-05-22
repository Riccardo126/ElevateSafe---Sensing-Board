import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, ifft





choose = "tools\sampling_2026-04-15_01-10-08.csv"
names = ["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
df = pd.read_csv(choose, names=names, header=0)

sr = 1000
ts = 1.0/sr
t = np.arange(0,1,ts)
X = fft(df['acc_z'])
N = len(X)
n = np.arange(N)
T = N/sr
freq = n/T 

plt.figure(figsize = (12, 6))
plt.subplot(121)

plt.stem(freq, np.abs(X), 'b', \
         markerfmt=" ", basefmt="-b")
plt.xlabel('Freq (Hz)')
plt.ylabel('FFT Amplitude |X(freq)|')
plt.xlim(0, 10)

plt.subplot(122)
plt.plot(t, ifft(X), 'r')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.tight_layout()
plt.show()