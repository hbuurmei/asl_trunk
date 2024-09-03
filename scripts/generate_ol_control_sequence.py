import os
import numpy as np
import pandas as pd  # type: ignore


def single_random_trajectory():
    # Parameters
    n_u = 6
    sample_interval = 1   # [s]
    total_duration = 150  # [s]
    sampling_rate = 100   # [Hz]
    u_min, u_max = -0.35, 0.35
    num_trajectories = 10  # Number of sinusoidal trajectories

    # Number of samples
    num_samples = total_duration // sample_interval
    total_samples = total_duration * sampling_rate

    # Generate sinusoidal trajectories
    us_sin = np.zeros((num_trajectories, total_samples, n_u))
    for i in range(num_trajectories):
        for j in range(n_u):
            amplitude = np.random.uniform(u_min, u_max)  # Random amplitude
            frequency = np.random.uniform(0.01, 0.1)  # Random frequency
            phase = np.random.uniform(0, 2 * np.pi)  # Random phase
            us_sin[i, :, j] = amplitude * np.sin(2 * np.pi * frequency * ts + phase)

    # Time vector for interpolation
    ts = np.arange(0, total_duration, 1/sampling_rate)

    # Interpolated sinusoidal trajectories
    us_interp = np.zeros((num_trajectories, total_samples, n_u))
    for i in range(num_trajectories):
        for j in range(n_u):
            us_interp[i, :, j] = np.interp(ts, np.arange(0, total_duration, sample_interval), us_sin[i, :, j])

    # Create DataFrames for each trajectory
    dfs = []
    for i in range(num_trajectories):
        IDs = np.arange(total_samples)
        df = pd.DataFrame(us_interp[i, :, :], columns=[f'u{i+1}' for i in range(n_u)])
        df['ID'] = IDs
        df = df[['ID'] + [f'u{i+1}' for i in range(n_u)]]
        dfs.append(df)

    # Concatenate all DataFrames into a single DataFrame
    df_combined = pd.concat(dfs, ignore_index=True)
    return df


def several_sine_trajectories():
    # Parameters
    n_u = 6
    sample_interval = 1  # [s]
    total_duration = 150  # [s]
    sampling_rate = 100  # [Hz]
    u_min, u_max = -0.35, 0.35

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
    return df





def main(control_inputs_file, control_type):
    if control_type == 'random':
        df = single_random_trajectory()
    elif control_type == 'sinusoidal':
        df = several_sine_trajectories()
    df.to_csv(control_inputs_file, index=False)

if __name__ == '__main__':
    control_type = 'sinusoidal'
    
    # Get desired file location
    data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')
    control_inputs_file = os.path.join(data_dir, f'trajectories/dynamic/control_inputs_controlled_{control_type}.csv')
    main(control_inputs_file, control_type='random')
