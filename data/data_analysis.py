import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc


import re



def extract_nums(text):
    # text.replace('[', ' ')
    # text.replace(']', ' ')
    p = re.compile(r'-?\d+\.?\d*')  # Compile a pattern to capture float values
    floats = [float(i) for i in p.findall(text)]  # Convert strings to float
    return np.array(floats)

with open('data/data.txt') as f:
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
    if 'kill time' in line:
        nums = list(extract_nums(line))
        trials[-1]['time'] = nums
    
actuals = []
detects = []
cameras = []
lines = []
times = []
for trial in trials:
    actuals.append(trial['actual'])
    detects.append(trial['detect'])
    cameras.append(trial['camera'])
    lines.append((trial['actual'], trial['detect']))
    times.append(trial['time'])
actuals = np.array(actuals)
detects = np.array(detects)
cameras = np.array(cameras)
times = np.array(times)


fig, ax = plt.subplots()

ax.scatter(x=actuals[:,0], y=actuals[:,1], label='Actual position')
ax.scatter(x=detects[:,0], y=detects[:,1], label='Detected position')
ax.scatter(x=0, y=0, label='Robot base', s=100)
ax.scatter(x=0.5, y=0, label='Trash can', s=100)

major_ticks = np.arange(-1, 1, 0.1)
ax.set_xticks(major_ticks)
ax.set_yticks(major_ticks)
plt.grid(True)

ax.set_aspect('equal')



# lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
lc = mc.LineCollection(lines, color='black', linewidths=1)
ax.add_collection(lc)

ax.autoscale()

accuracy = np.mean(cameras)
ax.set_title(f"Cockroach detection accuracy:: {np.round(accuracy, 3)}±{np.round(np.std(cameras), 3)}m")
ax.set_xlabel("Global X axis (m)")
ax.set_ylabel("Global Y axis (m)")
ax.legend()



# set up the figure
fig, ax = plt.subplots()


trash = np.array([0.5, 0])
dists = []
for actual in actuals:
    dist = np.linalg.norm(actual - trash)
    dists.append(dist)

ax.scatter(dists, times)
ax.set_title('Time taken vs distance away')
ax.set_ylabel('Time until cockroach disposal (s)')
ax.set_xlabel('Actual distance from trash (m)')


plt.show()
