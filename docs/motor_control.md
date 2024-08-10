


To simply run a command once to test the motor control, the following command can be used:

```bash
ros2 topic pub --once /all_motors_control interfaces/msg/AllMotorsControl "{motors_control: [{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0}]}"
```


## Motor control modes
$u$ is the control input to the motor, and the modes are as follows:

## Motor control limits
The motor control limits empirically established as follows:

$$
\operatorname{norm}\left(0.75\left(\vec{u}_3+\vec{u}_4\right)+1.0\left(\vec{u}_2+\vec{u}_5\right)+1.25\left(\vec{u}_1+\vec{u}_6\right)\right) \leq 0.6
$$
