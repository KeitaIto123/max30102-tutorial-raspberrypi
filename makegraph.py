#-*-coding:utf-8-*-

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# load log data
name_id = "default_400_av2_thumb/thumb_"
name_id = "thumb_"
red = []
with open("./" + name_id + "red.log", "r") as f:
    for r in f:
        red.append(int(r))

ir = []
with open("./" + name_id + "ir.log", "r") as f:
    for r in f:
        ir.append(int(r))

filter1 = signal.firwin(numtaps=401, cutoff=[0.4,4], fs=400, pass_zero=False)
normal_ir = signal.lfilter(filter1, 1, ir)

peaks, _ = signal.find_peaks(normal_ir)

zero_ir = signal.filtfilt(filter1, 1, ir)

# x-axis values
x = np.arange(len(ir))

fig = plt.figure()
ax = fig.add_subplot(111)

# RED LED
ax.plot(x, normal_ir, c="orange", label="FIR")
ax.plot(peaks, normal_ir[peaks],linestyle='None', marker='o')
ax.plot(x, ir, c="red", label="RAW")

# modify limits
# ax.set_ylim(153000, 190000)
# ax.set_xlim(0,350)

# show legend
ax.legend(loc="best")

plt.show()
