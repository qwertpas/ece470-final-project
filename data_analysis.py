import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc


import re



def extract_nums(text):
    # text.replace('[', ' ')
    # text.replace(']', ' ')
    p = re.compile(r'\d+\.\d+')  # Compile a pattern to capture float values
    floats = [float(i) for i in p.findall(text)]  # Convert strings to float
    return np.array(floats)

with open('data.txt') as f:
    lines = f.readlines()

trials = []
for line in lines:
    if line == '\n':
        continue
    if 'start kill' in line:
        trials.append({})
        continue
    if 'actual pos' in line:
        nums = list(extract_nums(line))
        trials[-1]['actual'] = nums
    if 'detect pos' in line:
        nums = list(extract_nums(line))
        trials[-1]['detect'] = nums
    if 'camera err' in line:
        nums = list(extract_nums(line))
        trials[-1]['camera'] = nums
    
actuals = []
detects = []
cameras = []
lines = []
for trial in trials:
    actuals.append(trial['actual'])
    detects.append(trial['detect'])
    cameras.append(trial['camera'])
    lines.append((trial['actual'], trial['detect']))
actuals = np.array(actuals)
detects = np.array(detects)
cameras = np.array(cameras)


fig, ax = plt.subplots()

ax.scatter(x=actuals[:,0], y=actuals[:,1], label='Actual position')
ax.scatter(x=detects[:,0], y=detects[:,1], label='Detected position')
ax.scatter(x=0, y=0, label='Robot base')


# lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
lc = mc.LineCollection(lines, color='black', linewidths=1)
ax.add_collection(lc)

print(len(actuals))

ax.autoscale()

accuracy = np.mean(cameras)
ax.set_title(f"Cockroach detection accuracy with average error: {np.round(accuracy, 3)} m")
ax.set_xlabel("Global X axis (m)")
ax.set_ylabel("Global Y axis (m)")
ax.legend()
plt.show()