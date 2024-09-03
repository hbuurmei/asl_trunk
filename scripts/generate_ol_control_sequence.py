import os
import numpy as np
import pandas as pd  # type: ignore


# Get desired file location
data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')
control_inputs_file = os.path.join(data_dir, f'trajectories/dynamic/control_inputs_controlled.csv')

# Parameters
n_u = 6
sample_interval = 5  # [s]
total_duration = 100  # [s]
sampling_rate = 100  # [Hz]
u_min, u_max = -0.25, 0.25

# Number of samples
num_samples = total_duration // sample_interval
total_samples = total_duration * sampling_rate

# Randomly sample control inputs at the specified interval
us_rnd = np.random.uniform(u_min, u_max, (num_samples, n_u))

# Time vector for interpolation
ts = np.arange(0, total_duration, 1/sampling_rate)

# Interpolated
us_interp = np.zeros((total_samples, n_u))
for i in range(n_u):
    us_interp[:, i] = np.interp(ts, np.arange(0, total_duration, sample_interval), us_rnd[:, i])

IDs = np.arange(total_samples)
df = pd.DataFrame(us_interp, columns=[f'u{i+1}' for i in range(n_u)])

df['ID'] = IDs
df = df[['ID'] + [f'u{i+1}' for i in range(n_u)]]
df.to_csv(control_inputs_file, index=False)
