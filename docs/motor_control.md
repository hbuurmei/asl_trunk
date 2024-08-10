


To simply run a command once to test the motor control, the following command can be used:

```bash
ros2 topic pub --once /all_motors_control interfaces/msg/AllMotorsControl "{motors_control: [{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0},{mode: 0, value: 0}]}"
```
