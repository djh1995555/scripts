import numpy as np
import matplotlib.pyplot as plt

sample_rate = 1000 
duration = 1.0 
t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
frequency1 = 50 
frequency2 = 120 
x = 0.7 * np.sin(2 * np.pi * frequency1 * t) + 0.3 * np.sin(2 * np.pi * frequency2 * t)
y = np.roll(x, 50)

x = np.array([0,0,1,2,3,7,9,8,0,0])
y = np.array([1,2,3,7,9,8,0,0,0,0])

fft_result = np.fft.fft(x)
frequencies = np.fft.fftfreq(len(x), d=1.0/sample_rate)

Nfft = x.shape[0]+y.shape[0]-1
X = np.fft.fft(x,Nfft)
Y = np.fft.fft(y,Nfft)
Sxy = Y * np.conj(X)
sxy=np.fft.ifftshift(np.fft.ifft(Sxy))
print('delay:{}'.format(max(sxy)))

plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(x, 'r', label='original')
plt.plot(y, 'b', label='delayed')
plt.title('Signals')
plt.xlabel('Time')
plt.ylabel('Amplitude')

plt.subplot(3, 1, 2)
plt.plot(frequencies, np.abs(fft_result))
plt.title('Frequency Spectrum')
plt.xlabel('Frequency')
plt.ylabel('Amplitude')

plt.subplot(3, 1, 3)
plt.plot(sxy)

plt.tight_layout()
plt.show()
