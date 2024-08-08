import os
import numpy as np
import pandas as pd

# Parameters
points_per_period = 13
amplitude = 0.25
control_variables = ['u1', 'u2', 'u3', 'u4', 'u5', 'u6']

num_controls = len(control_variables)
data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')
control_inputs_file = os.path.join(data_dir, 'trajectories/steady_state/control_inputs.csv')

# Create a dataframe to store the control inputs
df = pd.DataFrame(columns=['ID'] + control_variables)

# Generate sinusoidal values
for i, control_var in enumerate(control_variables):
    for j in range(points_per_period):
        values = np.zeros(num_controls)
        t = np.linspace(0, 2 * np.pi, points_per_period)
        values[i] = amplitude * np.sin(t[j])
        df = df.append(dict(zip(['ID'] + control_variables, [j + i * points_per_period] + list(values))), ignore_index=True)

# Save to CSV
df.to_csv(control_inputs_file, index=False)

print(f'Control inputs have been saved to {control_inputs_file}')
