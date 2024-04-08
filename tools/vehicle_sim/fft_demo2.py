import numpy as np
import matplotlib.pyplot as plt

sampling_rate = 1000
time = np.arange(0, 1, 1.0/sampling_rate)
frequency = 5 
delay = 0.3

signal1 = np.sin(2 * np.pi * frequency * time)
signal2 = np.sin(2 * np.pi * frequency * (time - delay))

cross_corr = np.correlate(signal1, signal2, mode='full')

lags = np.arange(-len(signal1) + 1, len(signal1))


plt.figure()
plt.plot(lags / sampling_rate, cross_corr)
plt.title("Cross-Correlation Function")
plt.xlabel("Time Lag (s)")
plt.ylabel("Correlation")
plt.grid(True)
plt.show()

estimated_delay = lags[np.argmax(cross_corr)] / sampling_rate
print("Estimated Delay:", estimated_delay, "seconds")
