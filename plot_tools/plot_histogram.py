import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np

data = pd.read_csv("/home/mi/debug/scripts/tools/min_pitch.csv")
pitches = data['pitch']
bins = 10

n, bins, patches = plt.hist(pitches,
														bins=bins,
														color='skyblue', 
														edgecolor='black')
total = sum(n)
for i in range(len(n)):
        plt.text(bins[i]+(bins[1]-bins[0])/2, n[i]*1.01, f"{round(n[i]/total*100,2)}%" , ha='center', va= 'bottom')

plt.title(f"Histogram of pitch of stereo slot {int(total)} parking lot")
plt.show()